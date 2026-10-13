import copy
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collector.host_arrival import HostArrivalBridge,validate_image_arrival
from collector.adapters import CameraFrame
from collector.timing import frame_time
from collector.camera_diagnostics import latest_ready_frame

class ArrivalTests(unittest.TestCase):
    def fixture(self):
        wall=1_800_000_000_000_000_000;receipt=10_000_000_000
        sample={'before_ns':receipt+100,'wall_ns':wall,'after_ns':receipt+300}
        bridge=HostArrivalBridge('realsense_time_of_arrival_ms','session')
        raw=wall//1_000_000-4
        stamp,proof=bridge.map(raw,receipt,sample)
        return bridge,stamp,proof,receipt

    def test_conservative_quantization_and_non_exposure_semantics(self):
        _,stamp,proof,receipt=self.fixture()
        self.assertEqual(stamp,receipt-3_000_000+300)
        frame=CameraFrame('left_wrist',1,receipt,receipt,b'',sdk_arrival_host_ns=stamp)
        self.assertEqual(frame_time(frame),stamp)
        self.assertIsNone(frame.measurement_host_ns)
        self.assertIs(latest_ready_frame([frame],receipt-1_000_000),frame)
        self.assertIsNone(latest_ready_frame([frame],stamp-1))

    def test_proof_recomputed_and_tampering_rejected(self):
        _,stamp,proof,receipt=self.fixture()
        image={'sdk_arrival_host_ns':stamp,'received_ns':receipt,'camera_session':'session',
               'timestamp_basis':'sdk_host_arrival_unmapped_exposure',
               'timing_metadata':{'host_arrival_proof':proof,'values':{'time_of_arrival':proof['raw']}}}
        self.assertEqual(validate_image_arrival(image),stamp)
        for key,value in [('sdk_arrival_host_ns',stamp-1),('camera_session','another')]:
            bad=copy.deepcopy(image);bad[key]=value
            with self.assertRaises(ValueError):validate_image_arrival(bad)
        bad=copy.deepcopy(image);bad['timing_metadata']['values']['time_of_arrival']+=1
        with self.assertRaises(ValueError):validate_image_arrival(bad)
        for key,value in [('version',2),('selection','exposure')]:
            bad=copy.deepcopy(image);bad['timing_metadata']['host_arrival_proof'][key]=value
            with self.assertRaises(ValueError):validate_image_arrival(bad)

    def test_clock_step_and_repeated_arrival_rejected(self):
        bridge,stamp,proof,receipt=self.fixture()
        with self.assertRaises(ValueError):bridge.map(proof['raw'],receipt,proof['clock_sample'])
        s=dict(proof['clock_sample']);s['wall_ns']+=10_000_000
        with self.assertRaisesRegex(ValueError,'stepped'):bridge.map(proof['raw']+10,receipt,s)

    def test_wrong_clock_domain_rejected(self):
        bridge=HostArrivalBridge('orbbec_system_timestamp_us','session')
        with self.assertRaises(ValueError):bridge.map(123,10_000_000_000,{'before_ns':10_000_000_000,'wall_ns':1_800_000_000_000_000_000,'after_ns':10_000_000_100})

if __name__=='__main__':unittest.main()
