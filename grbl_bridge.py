#!/usr/bin/env python3
"""
GRBL WebSocket & HTTP Streaming Bridge Daemon (v2)
Platforma: Raspberry Pi / Linux

Logika sterowania: grbl_core.py. Ten plik to wyłącznie warstwa HTTP/WebSocket.

WebSocket /ws
  Serwer -> klient: co 100 ms stan maszyny (JSON, pola MachineState).
  Klient -> serwer:
    * JSON (v2): {"cmd": "pause"|"resume"|"cancel"|"unlock"|"gcode"|"ack_error", "line": "...", "id": ...}
      Odpowiedź: {"type": "ack", "cmd": ..., "id": ..., "ok": bool, "message": ...}
    * Tekst (v1, zgodność wstecz): "!" = anuluj (bezpiecznie, z czyszczeniem bufora),
      "~" = wznów, "\\x18"/"\\x85" = znaki czasu rzeczywistego, pozostałe = G-code.

HTTP
  GET  /api/status
  POST /api/files/local      (multipart: file, print=true -> start zadania; 409 gdy zadanie trwa)
  POST /api/job/pause | /api/job/resume | /api/job/cancel
  POST /api/unlock
  POST /api/command          (form: command)
"""

from __future__ import annotations
import asyncio
import json
import os
import threading
from contextlib import asynccontextmanager
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File, Form, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from grbl_core import (
    GRBLBridge, CommandRejected, BRIDGE_VERSION,
    RT_FEED_HOLD, RT_CYCLE_START, RT_SOFT_RESET, RT_JOG_CANCEL,
)


SERIAL_PORT = os.getenv("GRBL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.getenv("GRBL_BAUD", "115200"))
HTTP_HOST = os.getenv("BRIDGE_HOST", "::")  # "::" = IPv6 + IPv4 (dual-stack); mDNS na Windows często daje tylko IPv6
HTTP_PORT = int(os.getenv("BRIDGE_PORT", "8000"))
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

bridge = GRBLBridge(SERIAL_PORT, BAUD_RATE)
clients: Set[WebSocket] = set()


async def broadcast_state():
    if not clients:
        return
    data = bridge.snapshot()
    dead = set()
    for ws in list(clients):
        try:
            await ws.send_json(data)
        except Exception:
            dead.add(ws)
    clients.difference_update(dead)


@asynccontextmanager
async def lifespan(app: FastAPI):
    bridge.connect()
    rx_thread = threading.Thread(target=bridge.serial_rx_worker, daemon=True)
    rx_thread.start()

    async def ws_broadcast_task():
        while True:
            await broadcast_state()
            await asyncio.sleep(0.1)

    task = asyncio.create_task(ws_broadcast_task())
    yield
    task.cancel()
    bridge.close()


app = FastAPI(title="GRBL Bridge Server", version=BRIDGE_VERSION, lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

if os.path.exists(WEB_DIR):
    app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")


# ==============================================================================
# WSPÓLNE AKCJE (wykonywane poza pętlą asyncio – mogą czekać na maszynę)
# ==============================================================================
async def run_action(cmd: str, line: str = "") -> str:
    if cmd == "pause":
        return await asyncio.to_thread(bridge.pause_job)
    if cmd == "resume":
        return await asyncio.to_thread(bridge.resume_job)
    if cmd == "cancel":
        return await asyncio.to_thread(bridge.cancel_job)
    if cmd == "unlock":
        return await asyncio.to_thread(bridge.unlock)
    if cmd == "ack_error":
        bridge.acknowledge_error()
        return "ok"
    if cmd == "gcode":
        return await asyncio.to_thread(bridge.execute_manual, line)
    raise CommandRejected(f"Nieznana komenda: {cmd}")


def _http_result(fn_result: str):
    if fn_result.startswith("error"):
        return JSONResponse(status_code=502, content={"ok": False, "message": fn_result})
    return {"ok": True, "message": fn_result}


# ==============================================================================
# HTTP
# ==============================================================================
@app.get("/")
async def serve_index():
    idx = os.path.join(WEB_DIR, "index.html")
    if os.path.isfile(idx):
        return FileResponse(idx)
    return JSONResponse(status_code=404, content={"error": "Brak web/index.html"})


@app.get("/api/status")
async def get_status():
    return bridge.snapshot()


@app.post("/api/files/local")
async def upload_gcode(file: UploadFile = File(...), print: bool = Form(False)):
    content = (await file.read()).decode("utf-8", errors="ignore")
    if not print:
        return {"status": "uploaded"}
    try:
        total = await asyncio.to_thread(bridge.start_job, content)
    except CommandRejected as e:
        raise HTTPException(status_code=409, detail=str(e))
    return {"status": "started", "total_lines": total}


@app.post("/api/job/{action}")
async def job_action(action: str):
    if action not in ("pause", "resume", "cancel"):
        raise HTTPException(status_code=404, detail="Nieznana akcja")
    try:
        return _http_result(await run_action(action))
    except CommandRejected as e:
        raise HTTPException(status_code=409, detail=str(e))


@app.post("/api/unlock")
async def unlock():
    try:
        return _http_result(await run_action("unlock"))
    except CommandRejected as e:
        raise HTTPException(status_code=409, detail=str(e))


@app.post("/api/command")
async def execute_command(command: str = Form(...)):
    cmd = command.strip()
    try:
        if cmd == RT_FEED_HOLD:
            return {"result": await run_action("pause")}
        if cmd == RT_CYCLE_START:
            return {"result": await run_action("resume")}
        if cmd in (RT_SOFT_RESET, RT_JOG_CANCEL):
            bridge.send_realtime(cmd)
            return {"result": "ok"}
        return {"result": await run_action("gcode", cmd)}
    except CommandRejected as e:
        raise HTTPException(status_code=409, detail=str(e))


# ==============================================================================
# WEBSOCKET
# ==============================================================================
async def handle_legacy_text(cmd: str):
    """Protokół v1 (surowy tekst) – zachowany dla starszych klientów."""
    try:
        if cmd == RT_FEED_HOLD:
            await run_action("cancel")      # v1: "!" oznaczało STOP – teraz bezpieczny
        elif cmd == RT_CYCLE_START:
            await run_action("resume")
        elif cmd in (RT_SOFT_RESET, RT_JOG_CANCEL):
            bridge.send_realtime(cmd)
        else:
            await run_action("gcode", cmd)
    except CommandRejected as e:
        print(f"[WS v1] Odrzucono '{cmd[:40]}': {e}")


async def handle_json(ws: WebSocket, msg: dict):
    cmd = str(msg.get("cmd", ""))
    reply = {"type": "ack", "cmd": cmd, "id": msg.get("id"), "ok": True, "message": "ok"}
    try:
        result = await run_action(cmd, str(msg.get("line", "")))
        reply["message"] = result
        reply["ok"] = not str(result).startswith("error")
    except CommandRejected as e:
        reply.update(ok=False, message=str(e))
    except Exception as e:  # nie zrywaj połączenia z powodu jednej komendy
        reply.update(ok=False, message=f"Błąd mostka: {e}")
    try:
        await ws.send_json(reply)
    except Exception:
        pass


_CONTROL_CMDS = ("pause", "resume", "cancel", "unlock", "ack_error")


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    # G-code z jednego połączenia wykonywany ściśle po kolei (kolejność ruchów jog);
    # komendy sterujące (pause/resume/cancel…) omijają kolejkę i działają natychmiast.
    gcode_queue: asyncio.Queue = asyncio.Queue()
    control_tasks = set()

    async def gcode_consumer():
        while True:
            coro = await gcode_queue.get()
            try:
                await coro
            finally:
                gcode_queue.task_done()

    consumer = asyncio.create_task(gcode_consumer())
    try:
        while True:
            data = await ws.receive_text()
            text = data if data in (RT_SOFT_RESET, RT_JOG_CANCEL) else data.strip()
            if not text:
                continue

            if text.startswith("{"):
                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    await ws.send_json({"type": "ack", "ok": False, "message": "Niepoprawny JSON"})
                    continue
                is_control = str(msg.get("cmd", "")) in _CONTROL_CMDS
                coro = handle_json(ws, msg)
            else:
                is_control = text in (RT_FEED_HOLD, RT_CYCLE_START, RT_SOFT_RESET, RT_JOG_CANCEL)
                coro = handle_legacy_text(text)

            if is_control:
                t = asyncio.create_task(coro)
                control_tasks.add(t)
                t.add_done_callback(control_tasks.discard)
            else:
                await gcode_queue.put(coro)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        clients.discard(ws)
        consumer.cancel()
        while not gcode_queue.empty():  # nie wykonuj komend rozłączonego klienta
            gcode_queue.get_nowait().close()


if __name__ == "__main__":
    uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT, log_level="warning")
