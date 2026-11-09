"""
Testy rdzenia mostka na symulatorze GRBL (bez sprzętu i bez FastAPI).
Uruchomienie z katalogu projektu:  python -m unittest discover -s tests -v
"""

from __future__ import annotations
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from grbl_core import GRBLBridge, CommandRejected, JOB_IDLE, JOB_RUNNING, JOB_PAUSED, JOB_ERROR  # noqa: E402
from fake_grbl import FakeGrbl  # noqa: E402


def job_text(n: int, prefix: str = "G1") -> str:
    lines = ["G21", "G90", "M4 S500"]
    lines += [f"{prefix} X{i + 1}.000 Y0.000 F1000" for i in range(n)]
    lines.append("M5")
    return "\n".join(lines)


def wait_for(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class BridgeTestCase(unittest.TestCase):
    def setUp(self):
        self.fake = FakeGrbl()
        self.bridge = GRBLBridge(serial_factory=lambda: self.fake, poll_interval=0.02)
        self.bridge.connect()
        self.rx = threading.Thread(target=self.bridge.serial_rx_worker, daemon=True)
        self.rx.start()
        self.assertTrue(wait_for(lambda: self.bridge.state.status == "Idle"))

    def tearDown(self):
        self.bridge.close()
        self.fake.close()

    def moves_executed(self):
        return [l for l in self.fake.executed if l.startswith("G1")]


class TestJobLifecycle(BridgeTestCase):
    def test_job_completes(self):
        self.bridge.start_job(job_text(40))
        self.assertTrue(wait_for(lambda: self.bridge.state.job_state == JOB_IDLE, 10))
        self.assertEqual(len(self.moves_executed()), 40)
        self.assertEqual(self.bridge.state.job_progress, 100.0)
        self.assertFalse(self.bridge.state.job_running)

    def test_long_moves_before_m5_do_not_time_out(self):
        # M5 dostaje 'ok' dopiero po opróżnieniu planisty: 15 ruchów x 0.2 s = 3 s >> limit 0.5 s
        self.fake.move_time = 0.2
        self.bridge.job_line_timeout = 0.5
        self.bridge.start_job(job_text(15))
        self.assertTrue(wait_for(lambda: self.bridge.state.job_state != JOB_RUNNING, 10))
        self.assertEqual(self.bridge.state.last_error, "")
        self.assertEqual(self.bridge.state.job_state, JOB_IDLE)
        self.assertEqual(len(self.moves_executed()), 15)
        self.assertEqual(self.fake.executed[-1], "M5")

    def test_second_job_rejected_while_running(self):
        self.bridge.start_job(job_text(200))
        with self.assertRaises(CommandRejected):
            self.bridge.start_job(job_text(5))
        self.bridge.cancel_job()

    def test_manual_gcode_rejected_during_job(self):
        self.bridge.start_job(job_text(200))
        with self.assertRaises(CommandRejected):
            self.bridge.execute_manual("$J=G91 X10 F1000")
        self.bridge.cancel_job()


class TestPauseResume(BridgeTestCase):
    def test_pause_then_resume_finishes_all_lines(self):
        self.bridge.start_job(job_text(60))
        self.assertTrue(wait_for(lambda: len(self.moves_executed()) >= 5))
        self.bridge.pause_job()
        self.assertEqual(self.bridge.state.job_state, JOB_PAUSED)
        self.assertTrue(wait_for(lambda: self.bridge.state.status == "Hold:0"))
        frozen = len(self.moves_executed())
        time.sleep(0.3)
        self.assertEqual(len(self.moves_executed()), frozen, "maszyna nie może ruszać się w pauzie")

        self.bridge.resume_job()
        self.assertTrue(wait_for(lambda: self.bridge.state.job_state == JOB_IDLE, 10))
        self.assertEqual(len(self.moves_executed()), 60)

    def test_door_hold_waits_and_resumes(self):
        self.bridge.start_job(job_text(40))
        self.assertTrue(wait_for(lambda: len(self.moves_executed()) >= 3))
        self.fake.open_door()
        time.sleep(0.5)  # dłużej niż czas na 'ok' – nie może być uznane za błąd
        self.assertEqual(self.bridge.state.job_state, JOB_RUNNING)
        self.fake.close_door()
        self.bridge.resume_job()
        self.assertTrue(wait_for(lambda: self.bridge.state.job_state == JOB_IDLE, 10))
        self.assertEqual(len(self.moves_executed()), 40)


class TestSafeCancel(BridgeTestCase):
    def test_cancel_flushes_grbl_buffer(self):
        """Najważniejszy test: po anulowaniu i '~' nie może wykonać się żadna linia z bufora."""
        self.bridge.start_job(job_text(300))
        self.assertTrue(wait_for(lambda: len(self.moves_executed()) >= 5))
        self.assertTrue(wait_for(lambda: len(self.fake._planner) >= 10), "planista powinien być zapełniony")

        msg = self.bridge.cancel_job()
        executed_at_cancel = len(self.moves_executed())

        self.assertIn("wyczyszczony", msg)
        self.assertEqual(self.fake.resets, 1)
        self.assertFalse(self.fake.alarm, "reset po pełnym zatrzymaniu nie może wywołać alarmu")
        self.assertEqual(len(self.fake._planner), 0)
        self.assertEqual(self.bridge.state.job_state, JOB_IDLE)

        # Próba wznowienia (bezpośrednio do GRBL) – bufor jest pusty, nic się nie wykona
        self.bridge.send_realtime("~")
        time.sleep(0.4)
        self.assertEqual(len(self.moves_executed()), executed_at_cancel)
        self.assertLess(executed_at_cancel, 300)

    def test_cancel_from_pause(self):
        self.bridge.start_job(job_text(300))
        self.assertTrue(wait_for(lambda: len(self.moves_executed()) >= 5))
        self.bridge.pause_job()
        self.assertTrue(wait_for(lambda: self.bridge.state.status == "Hold:0"))
        self.bridge.cancel_job()
        n = len(self.moves_executed())
        self.bridge.resume_job()  # bufor pusty – wznowienie nie może niczego wykonać
        time.sleep(0.3)
        self.assertEqual(len(self.moves_executed()), n)
        self.assertFalse(self.fake.alarm)
        self.assertEqual(self.bridge.state.job_state, JOB_IDLE)

    def test_resume_blocked_after_error(self):
        self.bridge.state.job_state = JOB_ERROR
        with self.assertRaises(CommandRejected):
            self.bridge.resume_job()

    def test_cancel_without_job_clears_hold(self):
        """Przycisk bezpieczeństwa / ręczny feed hold bez zadania mostka."""
        for i in range(20):
            self.fake.write(f"G1 X{i} F100\n".encode())
        self.assertTrue(wait_for(lambda: self.fake.status == "Run"))
        self.fake.write(b"!")
        self.assertTrue(wait_for(lambda: self.bridge.state.status == "Hold:0"))
        self.bridge.cancel_job()
        self.assertEqual(len(self.fake._planner), 0)
        self.assertTrue(wait_for(lambda: self.bridge.state.status == "Idle"))

    def test_manual_commands_work_after_cancel(self):
        self.bridge.start_job(job_text(200))
        self.assertTrue(wait_for(lambda: len(self.moves_executed()) >= 3))
        self.bridge.cancel_job()
        self.assertTrue(self.bridge.execute_manual("$J=G91 X1 F500").startswith("ok"))


class TestErrorsAndAlarms(BridgeTestCase):
    def test_grbl_error_stops_job_safely(self):
        text = job_text(10) + "\nBAD LINE\n" + "\n".join(f"G1 X{100 + i} F1000" for i in range(50))
        self.bridge.start_job(text)
        self.assertTrue(wait_for(lambda: self.bridge.state.job_state == JOB_ERROR, 10))
        self.assertIn("BAD LINE", self.bridge.state.last_error)
        time.sleep(0.3)
        after_error = [l for l in self.moves_executed() if float(l.split()[1][1:]) >= 100]
        self.assertEqual(after_error, [], "linie po błędzie nie mogą się wykonać")
        self.assertEqual(len(self.fake._planner), 0)

    def test_unlock_after_alarm(self):
        with self.fake._lock:
            self.fake.alarm = True
            self.fake.status = "Alarm"
            self.fake._emit("ALARM:1")
        self.assertTrue(wait_for(lambda: self.bridge.state.alarm == 1))
        with self.assertRaises(CommandRejected):
            self.bridge.resume_job()
        with self.assertRaises(CommandRejected):
            self.bridge.start_job(job_text(3))
        self.assertTrue(self.bridge.unlock().startswith("ok"))
        self.assertTrue(wait_for(lambda: self.bridge.state.status == "Idle"))
        self.assertIsNone(self.bridge.state.alarm)


class TestParsing(unittest.TestCase):
    def test_prepare_lines_strips_comments(self):
        lines = GRBLBridge.prepare_lines("G21 ; mm\n; komentarz\n(nawias) G0 X1\n\nM5")
        self.assertEqual(lines, ["G21", "G0 X1", "M5"])

    def test_status_with_wco(self):
        b = GRBLBridge(serial_factory=lambda: None)
        b._parse_status("<Idle|MPos:10.000,20.000,0.000|FS:0,0|WCO:5.000,5.000,0.000>")
        b._parse_status("<Idle|MPos:10.000,20.000,0.000|FS:0,0>")
        self.assertEqual((b.state.wpos_x, b.state.wpos_y), (5.0, 15.0))


if __name__ == "__main__":
    unittest.main()
