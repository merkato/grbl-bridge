"""
Symulator GRBL 1.1 do testów mostka (bez sprzętu).

Modeluje to, co istotne dla bezpieczeństwa sterowania zadaniem:
- bufor planisty (domyślnie 15 bloków): 'ok' dopiero, gdy linia trafi do planisty,
- wykonywanie ruchów w czasie (stan Run), feed hold z hamowaniem (Hold:1 -> Hold:0),
- cycle start '~', soft-reset Ctrl-X (czyści bufory; w ruchu -> ALARM:3),
- M3/M4/M5/G4 synchronizowane z planistą ('ok' po wykonaniu wszystkich ruchów),
- alarm i $X, błąd 'error:20' dla linii zaczynających się od "BAD",
- stan Door (otwarcie drzwi / przycisk bezpieczeństwa) wywoływany z testu.
Lista `executed` zawiera linie faktycznie wykonane przez "maszynę".
"""

from __future__ import annotations
import threading
import time
from collections import deque


class FakeGrbl:
    def __init__(self, planner_size: int = 15, move_time: float = 0.02, decel_time: float = 0.15):
        self.planner_size = planner_size
        self.move_time = move_time
        self.decel_time = decel_time
        self.is_open = True

        self._lock = threading.RLock()
        self._out = bytearray()
        self._rx_line = bytearray()
        self._pending = deque()      # linie przyjęte przez port, czekające na miejsce w planiście
        self._planner = deque()      # linie w planiście
        self.executed = []
        self.status = "Idle"
        self.alarm = False
        self.hold_since = None
        self.pos = [0.0, 0.0, 0.0]
        self.resets = 0

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ port API
    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._out)

    def read(self, n: int = 1) -> bytes:
        deadline = time.time() + 0.02
        while True:
            with self._lock:
                if self._out:
                    data = bytes(self._out[:n])
                    del self._out[:n]
                    return data
            if time.time() > deadline:
                return b""
            time.sleep(0.001)

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def write(self, data: bytes):
        with self._lock:
            for byte in data:
                ch = chr(byte)
                if ch == "?":
                    self._emit(self._status_report())
                elif ch == "!":
                    if self.status in ("Run", "Jog"):
                        self.status = "Hold:1"
                        self.hold_since = time.time()
                elif ch == "~":
                    if self.status.startswith("Hold:0") or self.status == "Door:0":
                        self.status = "Run" if self._planner else "Idle"
                elif ch == "\x18":
                    self._soft_reset()
                elif ch == "\x85":
                    pass
                elif ch == "\n":
                    line = self._rx_line.decode("latin-1").strip()
                    self._rx_line.clear()
                    if line:
                        self._pending.append(line)
                elif ch != "\r":
                    self._rx_line.append(byte)

    # ------------------------------------------------------------------ test hooks
    def open_door(self):
        with self._lock:
            self.status = "Door:1"

    def close_door(self):
        with self._lock:
            if self.status.startswith("Door"):
                self.status = "Door:0"

    # ------------------------------------------------------------------ wewnętrzne
    def _emit(self, text: str):
        self._out.extend((text + "\r\n").encode("latin-1"))

    def _status_report(self) -> str:
        x, y, z = self.pos
        return f"<{self.status}|MPos:{x:.3f},{y:.3f},{z:.3f}|FS:0,0>"

    def _soft_reset(self):
        moving = self.status in ("Run", "Jog", "Hold:1")
        self._pending.clear()
        self._planner.clear()
        self._rx_line.clear()
        self.resets += 1
        self._emit("Grbl 1.1h ['$' for help]")
        if moving:
            self.alarm = True
            self.status = "Alarm"
            self._emit("ALARM:3")
        else:
            self.status = "Alarm" if self.alarm else "Idle"

    def _accept_pending(self):
        while self._pending and len(self._planner) < self.planner_size:
            line = self._pending[0]
            # Jak w GRBL: M3/M4/M5/G4 czekają, aż planista się opróżni (synchronizacja), 'ok' dopiero potem
            if self._planner and line.split()[0] in ("M3", "M4", "M5", "G4"):
                break
            self._pending.popleft()
            if line.startswith("BAD"):
                self._emit("error:20")
                continue
            if line == "$X":
                self.alarm = False
                if self.status == "Alarm":
                    self.status = "Idle"
                self._emit("ok")
                continue
            if self.alarm:
                self._emit("error:9")
                continue
            if line.startswith(("G0", "G1", "$J=")):
                self._planner.append(line)
                if self.status == "Idle":
                    self.status = "Jog" if line.startswith("$J=") else "Run"
            else:
                # Komendy niemodalne / modalne bez ruchu wykonują się "od razu"
                self.executed.append(line)
            self._emit("ok")

    def _run(self):
        last_step = time.time()
        while self.is_open:
            time.sleep(0.002)
            with self._lock:
                self._accept_pending()
                now = time.time()
                if self.status == "Hold:1" and now - (self.hold_since or now) >= self.decel_time:
                    self.status = "Hold:0"
                if self.status in ("Run", "Jog"):
                    if self._planner and now - last_step >= self.move_time:
                        line = self._planner.popleft()
                        self.executed.append(line)
                        for word in line.replace("$J=", "").split():
                            if word[0] in "XYZ":
                                try:
                                    self.pos["XYZ".index(word[0])] = float(word[1:])
                                except ValueError:
                                    pass
                        last_step = now
                    if not self._planner and not self._pending:
                        self.status = "Idle"
                else:
                    last_step = now
