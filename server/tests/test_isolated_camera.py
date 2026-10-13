import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collector.isolated_camera import IsolatedCamera
from collector.adapters import CameraFrame
from collector.clock import monotonic_ns


def synthetic_camera(kind,settings,connection,stop):
    index=0
    try:
        while not stop.is_set():
            index+=1;received=monotonic_ns()
            connection.send(('pending',(index,received)))
            frame=CameraFrame(kind,index,received,received,b'jpeg',ready_ns=monotonic_ns(),
                device_timestamp_ms=index*16.7,timing_metadata={'values':{}})
            connection.send(('frame',frame,{'jpeg_encoding':0.01}))
            stop.wait(.0167)
    finally:connection.close()


def failed_camera(kind,settings,connection,stop):
    connection.send(('error','test SDK connection failure'))
    connection.close()


class ProcessTests(unittest.TestCase):
    def test_timestamp_provenance_and_shutdown(self):
        c=IsolatedCamera('head',{'serial':'fixture','width':10,'height':10,'fps':60},worker=synthetic_camera)
        try:
            f=c.latest()
            self.assertIsNotNone(f)
            self.assertGreaterEqual(f.ready_ns,f.timing_metadata['transport']['encode_ready_ns'])
            self.assertGreaterEqual(f.ready_ns,f.received_ns)
            self.assertIsNone(f.measurement_host_ns)
            self.assertEqual(f.jpeg,b'jpeg')
            self.assertTrue(c.process.is_alive())
            self.assertFalse(c.diagnostics.snapshot()['physical_sync_verified'])
            self.assertIn('ipc_delivery',c.diagnostics.snapshot()['stages_ms'])
        finally:c.close()
        self.assertFalse(c.process.is_alive())
        self.assertFalse(c.reader.is_alive())

    def test_startup_failure_is_explicit(self):
        with self.assertRaisesRegex(RuntimeError,'test SDK connection failure'):
            IsolatedCamera('head',{},worker=failed_camera)


if __name__=='__main__':unittest.main()
