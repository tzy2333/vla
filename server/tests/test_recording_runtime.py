import gc
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collector.recording_runtime import CaptureTrace, RecordingRuntime


class RecordingRuntimeTests(unittest.TestCase):
    def test_restores_gc_on_failure_and_releases_ownership(self):
        previously = gc.isenabled()
        gc.enable()
        runtime = RecordingRuntime()
        try:
            try:
                runtime.begin()
                self.assertFalse(gc.isenabled())
                with self.assertRaisesRegex(RuntimeError, 'Another recording'):
                    RecordingRuntime().begin()
                raise ValueError('simulated capture failure')
            finally:
                runtime.close()
        except ValueError:
            pass
        try:
            self.assertTrue(gc.isenabled())
            other = RecordingRuntime()
            other.begin()
            other.close()
            self.assertTrue(gc.isenabled())
        finally:
            if not previously:
                gc.disable()

    def test_preserves_preexisting_disabled_state(self):
        previously = gc.isenabled()
        gc.disable()
        runtime = RecordingRuntime()
        try:
            runtime.begin()
            runtime.close()
            runtime.close()
            self.assertFalse(gc.isenabled())
        finally:
            if previously:
                gc.enable()

    def test_trace_is_bounded_and_snapshot_is_detached(self):
        trace = CaptureTrace(3)
        for index in range(9):
            trace.record('feedback', packet_id=index)
        snapshot = trace.snapshot()
        self.assertEqual([r['packet_id'] for r in snapshot], [6, 7, 8])
        trace.record('camera')
        self.assertEqual([r['packet_id'] for r in snapshot], [6, 7, 8])


if __name__ == '__main__':
    unittest.main()
