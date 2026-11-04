#!/usr/bin/env python3
"""
Rdzeń mostka GRBL: port szeregowy, odpytywanie stanu, strumieniowanie zadania
oraz bezpieczne sterowanie zadaniem (pauza / wznowienie / anulowanie z czyszczeniem bufora).

Moduł nie zależy od FastAPI – może być testowany z symulowanym portem szeregowym.

Najważniejsza zasada bezpieczeństwa:
  Po przerwaniu zadania GRBL wciąż trzyma w buforze planisty i bufora RX kilkanaście
  przyjętych już linii. Zwykłe wznowienie ('~') wykonałoby je z włączonym laserem.
  Anulowanie robi więc: '!' (feed hold) -> czekanie na pełne zatrzymanie (Hold:0)
  -> soft-reset (Ctrl-X), który czyści bufory GRBL bez utraty pozycji.
"""

from __future__ import annotations
import re
import threading
import time
from dataclasses import dataclass, asdict, field
from typing import Callable, List, Optional, Dict, Any

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - pozwala testować bez pyserial
    serial = None


BRIDGE_VERSION = "2.0.0"
CAPABILITIES = ["json_commands", "job_control", "safe_cancel", "unlock", "door_resume"]

RT_FEED_HOLD = "!"
RT_CYCLE_START = "~"
RT_SOFT_RESET = "\x18"
RT_JOG_CANCEL = "\x85"
REALTIME_COMMANDS = (RT_FEED_HOLD, RT_CYCLE_START, RT_SOFT_RESET, RT_JOG_CANCEL)

# Stany zadania mostka (niezależne od stanu GRBL)
JOB_IDLE = "idle"
JOB_RUNNING = "running"
JOB_PAUSED = "paused"
JOB_CANCELLING = "cancelling"
JOB_ERROR = "error"

_COMMENT_RE = re.compile(r";.*|\(.*?\)")


@dataclass
class MachineState:
    status: str = "Unknown"
    mpos_x: float = 0.0
    mpos_y: float = 0.0
    mpos_z: float = 0.0
    wpos_x: float = 0.0
    wpos_y: float = 0.0
    wpos_z: float = 0.0
    feed_rate: float = 0.0
    spindle_speed: float = 0.0
    job_running: bool = False
    job_progress: float = 0.0
    job_line: int = 0
    job_total: int = 0
    # --- v2 ---
    job_state: str = JOB_IDLE
    alarm: Optional[int] = None
    last_error: str = ""
    serial_connected: bool = False
    bridge_version: str = BRIDGE_VERSION
    capabilities: List[str] = field(default_factory=lambda: list(CAPABILITIES))


class CommandRejected(Exception):
    """Komenda odrzucona w bieżącym stanie (np. ręczny G-code w trakcie zadania)."""


class GRBLBridge:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 115200,
        serial_factory: Optional[Callable[[], Any]] = None,
        poll_interval: float = 0.1,
    ):
        self.port = port
        self.baud = baud
        self.serial = None
        self.state = MachineState()
        self.poll_interval = poll_interval
        self._serial_factory = serial_factory

        self._wco = [0.0, 0.0, 0.0]
        self._running = True

        # Zapis do portu (linie i znaki czasu rzeczywistego)
        self._write_lock = threading.Lock()
        # Jedna linia "w locie" na raz: wysłanie + oczekiwanie na ok/error są atomowe,
        # więc odpowiedź nie trafi do niewłaściwego nadawcy (zadanie vs. komenda ręczna).
        self._line_lock = threading.Lock()
        self._response_event = threading.Event()
        self._last_response = ""
        self._reset_event = threading.Event()
        self._status_event = threading.Event()

        # Zadanie
        self._job_cancel = threading.Event()
        self._job_thread: Optional[threading.Thread] = None
        self._control_lock = threading.Lock()  # serializuje pause/resume/cancel/start

    # ==========================================================================
    # PORT SZEREGOWY
    # ==========================================================================
    def connect(self) -> bool:
        try:
            if self._serial_factory:
                self.serial = self._serial_factory()
            else:
                if serial is None:
                    raise RuntimeError("Brak modułu pyserial")
                self.serial = serial.Serial(self.port, self.baud, timeout=0.05)
                time.sleep(1.0)
            with self._write_lock:
                self.serial.write(b"\r\n\r\n")
                self.serial.flush()
            self.state.serial_connected = True
            print(f"[SERIAL] Połączono z GRBL na {self.port} ({self.baud})")
            return True
        except Exception as e:
            print(f"[SERIAL BŁĄD] Nie można otworzyć {self.port}: {e}")
            self.serial = None
            self.state.serial_connected = False
            return False

    def close(self):
        self._running = False
        self._job_cancel.set()
        self._response_event.set()
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass

    def _port_open(self) -> bool:
        return self.serial is not None and getattr(self.serial, "is_open", False)

    # ==========================================================================
    # WĄTEK ODCZYTU
    # ==========================================================================
    def serial_rx_worker(self):
        """Odczyt linii z GRBL i cykliczne odpytywanie '?'."""
        last_poll = 0.0
        buf = bytearray()

        while self._running:
            if not self._port_open():
                self.state.serial_connected = False
                time.sleep(1.0)
                if self._running:
                    self.connect()
                continue

            try:
                now = time.time()
                if now - last_poll >= self.poll_interval:
                    with self._write_lock:
                        self.serial.write(b"?")
                    last_poll = now

                raw = self.serial.read(self.serial.in_waiting or 1)
                if not raw:
                    time.sleep(0.005)
                    continue
                buf.extend(raw)
                while b"\n" in buf:
                    line_bytes, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    line = line_bytes.decode("latin-1", errors="ignore").strip()
                    if line:
                        self._handle_line(line)
            except Exception as e:
                print(f"[SERIAL RX BŁĄD] {e}")
                self.state.serial_connected = False
                try:
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None
                self._response_event.set()  # odblokuj oczekujących
                time.sleep(0.5)

    def _handle_line(self, line: str):
        if line.startswith("<"):
            self._parse_status(line)
            self._status_event.set()
        elif line.startswith("ok") or line.startswith("error:"):
            self._last_response = line
            self._response_event.set()
        elif line.startswith("ALARM:"):
            try:
                self.state.alarm = int(line.split(":", 1)[1])
            except ValueError:
                self.state.alarm = -1
            self.state.status = "Alarm"
            self.state.last_error = line
        elif line.startswith("Grbl ") or line.startswith("GrblHAL"):
            # Baner po resecie: wszystkie linie w locie przepadły
            self._last_response = "error: reset"
            self._response_event.set()
            self._reset_event.set()

    def _parse_status(self, report: str):
        if not report.endswith(">"):
            return
        parts = report[1:-1].split("|")
        if not parts:
            return

        self.state.status = parts[0]
        if not parts[0].startswith("Alarm"):
            self.state.alarm = None
        for p in parts[1:]:
            try:
                if p.startswith("MPos:"):
                    c = [float(v) for v in p[5:].split(",")[:3]]
                    self.state.mpos_x, self.state.mpos_y, self.state.mpos_z = c
                    self.state.wpos_x = round(c[0] - self._wco[0], 3)
                    self.state.wpos_y = round(c[1] - self._wco[1], 3)
                    self.state.wpos_z = round(c[2] - self._wco[2], 3)
                elif p.startswith("WPos:"):
                    c = [float(v) for v in p[5:].split(",")[:3]]
                    self.state.wpos_x, self.state.wpos_y, self.state.wpos_z = c
                    self.state.mpos_x = round(c[0] + self._wco[0], 3)
                    self.state.mpos_y = round(c[1] + self._wco[1], 3)
                    self.state.mpos_z = round(c[2] + self._wco[2], 3)
                elif p.startswith("WCO:"):
                    self._wco = [float(v) for v in p[4:].split(",")[:3]]
                elif p.startswith("FS:"):
                    vals = [float(v) for v in p[3:].split(",")]
                    if len(vals) >= 2:
                        self.state.feed_rate, self.state.spindle_speed = vals[0], vals[1]
                elif p.startswith("F:"):
                    self.state.feed_rate = float(p[2:])
            except ValueError:
                continue

    # ==========================================================================
    # WYSYŁANIE
    # ==========================================================================
    def send_realtime(self, char: str):
        """Natychmiastowy znak kontrolny – omija kolejkę linii."""
        if not self._port_open():
            return
        with self._write_lock:
            try:
                self.serial.write(char.encode("latin-1"))
                self.serial.flush()
            except Exception as e:
                print(f"[RT BŁĄD] {e}")

    def send_gcode_line(self, line: str, timeout: float = 3.0, abort: Optional[threading.Event] = None) -> str:
        """
        Wysyła jedną linię i czeka na ok/error. Oczekiwanie jest przerywane przez `abort`
        oraz wydłużane, gdy maszyna stoi w Hold/Door (bufor GRBL jest wtedy pełny).
        """
        if not self._port_open():
            return "error: Port zamknięty"

        cmd = line.strip()
        if not cmd:
            return "ok"

        with self._line_lock:
            self._response_event.clear()
            with self._write_lock:
                try:
                    self.serial.write((cmd + "\n").encode("latin-1"))
                    self.serial.flush()
                except Exception as e:
                    return f"error: {e}"

            deadline = time.time() + timeout
            while True:
                if self._response_event.wait(timeout=0.05):
                    return self._last_response
                if abort is not None and abort.is_set():
                    return "error: aborted"
                if not self._port_open() or not self._running:
                    return "error: Port zamknięty"
                if self._is_held():
                    deadline = time.time() + timeout  # pauza nie jest błędem komunikacji
                elif time.time() > deadline:
                    return "error: Timeout odpowiedzi GRBL"

    def _is_held(self) -> bool:
        s = self.state.status
        return s.startswith("Hold") or s.startswith("Door")

    def execute_manual(self, line: str) -> str:
        """Ręczna komenda (jog, $X, G10…). Odrzucana w trakcie zadania."""
        if self.state.job_state in (JOB_RUNNING, JOB_PAUSED, JOB_CANCELLING):
            raise CommandRejected("Trwa zadanie – ręczne komendy są zablokowane.")
        resp = "ok"
        for part in line.splitlines():
            if part.strip():
                resp = self.send_gcode_line(part)
                if resp.startswith("error"):
                    break
        return resp

    # ==========================================================================
    # ZADANIE
    # ==========================================================================
    @staticmethod
    def prepare_lines(gcode_text: str) -> List[str]:
        out = []
        for raw in gcode_text.splitlines():
            clean = _COMMENT_RE.sub("", raw).strip()
            if clean:
                out.append(clean)
        return out

    def start_job(self, gcode_text: str) -> int:
        with self._control_lock:
            if self._job_thread and self._job_thread.is_alive():
                raise CommandRejected("Inne zadanie jest w toku – najpierw je anuluj.")
            if not self._port_open():
                raise CommandRejected("Brak połączenia z GRBL.")
            if self.state.status.startswith(("Alarm", "Hold", "Door")):
                raise CommandRejected(f"Maszyna w stanie {self.state.status} – odblokuj/wyczyść przed startem.")

            lines = self.prepare_lines(gcode_text)
            if not lines:
                raise CommandRejected("Plik nie zawiera poleceń G-code.")

            self.state.job_total = len(lines)
            self.state.job_line = 0
            self.state.job_progress = 0.0
            self.state.job_running = True
            self.state.job_state = JOB_RUNNING
            self.state.last_error = ""
            self._job_cancel.clear()

            self._job_thread = threading.Thread(target=self._job_worker, args=(lines,), daemon=True)
            self._job_thread.start()
            return len(lines)

    def _job_worker(self, lines: List[str]):
        print(f"[JOB] Start: {len(lines)} linii.")
        failed = False
        for idx, line in enumerate(lines, start=1):
            if self._job_cancel.is_set():
                break
            resp = self.send_gcode_line(line, timeout=30.0, abort=self._job_cancel)
            if self._job_cancel.is_set():
                break
            if resp.startswith("error"):
                self.state.last_error = f"Linia {idx}: {line} -> {resp}"
                print(f"[JOB] {self.state.last_error}")
                failed = True
                break
            self.state.job_line = idx
            self.state.job_progress = round(idx / len(lines) * 100.0, 1)

        if failed:
            # Błąd GRBL: zatrzymaj bezpiecznie resztę bufora i zgłoś błąd
            threading.Thread(target=self.cancel_job, kwargs={"final_state": JOB_ERROR}, daemon=True).start()
            return
        if self._job_cancel.is_set():
            return

        # 'ok' oznacza przyjęcie linii do planisty, nie jej wykonanie. Zadanie trwa,
        # dopóki maszyna nie opróżni bufora i nie wróci do Idle (pauza/drzwi wydłużają czekanie).
        self._status_event.clear()
        time.sleep(max(0.05, 2 * self.poll_interval))  # świeży raport po ostatniej linii
        while not self._job_cancel.is_set() and self._running:
            status = self.state.status
            if status.startswith("Idle"):
                break
            if status.startswith("Alarm"):
                self.state.last_error = f"Alarm w trakcie zadania: {status}"
                self.state.job_running = False
                self.state.job_state = JOB_ERROR
                return
            self._status_event.clear()
            self._status_event.wait(timeout=0.2)

        if not self._job_cancel.is_set():
            self.state.job_progress = 100.0
            self.state.job_running = False
            self.state.job_state = JOB_IDLE
            print("[JOB] Zakończono.")

    def pause_job(self) -> str:
        """Feed hold. Zadanie pozostaje w pamięci i może zostać wznowione."""
        self.send_realtime(RT_FEED_HOLD)
        if self.state.job_state == JOB_RUNNING:
            self.state.job_state = JOB_PAUSED
        return "ok"

    def resume_job(self) -> str:
        """Cycle start. Dozwolone tylko po pauzie lub z Hold/Door bez anulowanego zadania."""
        if self.state.job_state in (JOB_CANCELLING, JOB_ERROR):
            raise CommandRejected("Zadanie zostało anulowane – wznowienie jest zablokowane.")
        if self.state.status.startswith("Alarm"):
            raise CommandRejected("Alarm – użyj Odblokuj ($X).")
        self.send_realtime(RT_CYCLE_START)
        if self.state.job_state == JOB_PAUSED:
            self.state.job_state = JOB_RUNNING
        return "ok"

    def _wait_status(self, predicate: Callable[[str], bool], timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate(self.state.status):
                return True
            self._status_event.clear()
            self._status_event.wait(timeout=min(0.2, max(0.01, deadline - time.time())))
        return predicate(self.state.status)

    def cancel_job(self, final_state: str = JOB_IDLE, stop_timeout: float = 8.0) -> str:
        """
        Anuluje zadanie i usuwa z pamięci GRBL wszystkie przyjęte linie:
        '!' -> czekanie na Hold:0 / Idle -> soft-reset -> czekanie na baner.
        Bezpieczne także bez aktywnego zadania (czyści stan Hold po przycisku bezpieczeństwa).
        """
        with self._control_lock:
            self.state.job_state = JOB_CANCELLING
            self._job_cancel.set()
            self._response_event.set()  # przerwij oczekiwanie wątku zadania

            message = "Zadanie anulowane, bufor GRBL wyczyszczony."
            if self._port_open():
                self.send_realtime(RT_JOG_CANCEL)
                self.send_realtime(RT_FEED_HOLD)
                stopped = self._wait_status(
                    lambda s: s.startswith(("Hold:0", "Idle", "Alarm", "Door:0", "Door:1")) or s == "Hold",
                    stop_timeout,
                )
                if not stopped:
                    message = "Maszyna nie zatrzymała się w czasie – wykonano reset (sprawdź pozycję/bazowanie)."
                self._reset_event.clear()
                self.send_realtime(RT_SOFT_RESET)
                if not self._reset_event.wait(timeout=3.0):
                    message += " Brak potwierdzenia resetu od GRBL."
                # Poczekaj na pierwszy raport stanu po resecie
                self._wait_status(lambda s: not s.startswith(("Hold", "Run", "Jog")), 2.0)
            else:
                message = "Zadanie anulowane (brak połączenia z GRBL)."

            if self._job_thread and self._job_thread.is_alive() and self._job_thread is not threading.current_thread():
                self._job_thread.join(timeout=2.0)

            self.state.job_running = False
            self.state.job_progress = 0.0 if final_state != JOB_ERROR else self.state.job_progress
            self.state.job_state = final_state
            if final_state == JOB_ERROR and self.state.last_error:
                message = f"Zadanie przerwane błędem: {self.state.last_error}"
            print(f"[JOB] {message}")
            return message

    def unlock(self) -> str:
        """Kasuje stan alarmu ($X). Po alarmie z utratą pozycji zalecane bazowanie ($H)."""
        if self.state.job_state in (JOB_RUNNING, JOB_PAUSED, JOB_CANCELLING):
            raise CommandRejected("Trwa zadanie – najpierw je anuluj.")
        resp = self.send_gcode_line("$X")
        if not resp.startswith("error"):
            self.state.alarm = None
            if self.state.job_state == JOB_ERROR:
                self.state.job_state = JOB_IDLE
        return resp

    def acknowledge_error(self):
        if self.state.job_state == JOB_ERROR:
            self.state.job_state = JOB_IDLE

    def snapshot(self) -> Dict[str, Any]:
        return asdict(self.state)
