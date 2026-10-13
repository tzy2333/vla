import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image, JpegImagePlugin
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collector.jpeg_encoder import RGBJPEGEncoder,configure_frame_queue,restore_frame_queue


class EncoderTests(unittest.TestCase):
    def test_dimensions_channels_and_quality_contract(self):
        rgb=np.zeros((240,424,3),dtype=np.uint8)
        rgb[:,:140]=[240,15,10];rgb[:,140:280]=[10,240,15];rgb[:,280:]=[10,15,240]
        old,_=RGBJPEGEncoder('opencv').encode(rgb)
        new,stages=RGBJPEGEncoder('simplejpeg').encode(rgb)
        a=Image.open(io.BytesIO(old));b=Image.open(io.BytesIO(new));pixels=np.asarray(b)
        self.assertEqual(b.size,(424,240));self.assertEqual(a.quantization,b.quantization)
        self.assertEqual(JpegImagePlugin.get_sampling(a),JpegImagePlugin.get_sampling(b))
        for x,channel in [(50,0),(200,1),(350,2)]:self.assertEqual(int(pixels[100,x].argmax()),channel)
        self.assertGreaterEqual(stages['jpeg_encoding'],0)
        self.assertLess(np.abs(np.asarray(a,dtype=np.int16)-pixels).mean(),1)

    def test_noncontiguous_rgb(self):
        rgb=np.random.default_rng(10).integers(0,256,(32,80,3),dtype=np.uint8)[:,::2]
        encoder=RGBJPEGEncoder()
        a,_=encoder.encode(rgb);b,_=encoder.encode(np.ascontiguousarray(rgb))
        self.assertEqual(a,b)

    def test_invalid_input_is_rejected(self):
        encoder=RGBJPEGEncoder()
        for a in [np.zeros((2,2)),np.zeros((2,2,4),dtype=np.uint8)]:
            with self.assertRaises(ValueError):encoder.encode(a)
        with self.assertRaises(ValueError):RGBJPEGEncoder('unknown')

    def test_queue_setting_and_restore(self):
        class Sensor:
            value=16
            def supports(self,option):return True
            def get_option(self,option):return self.value
            def set_option(self,option,value):self.value=value
        sensor=Sensor();device=SimpleNamespace(query_sensors=lambda:[sensor])
        rs=SimpleNamespace(option=SimpleNamespace(frames_queue_size='queue'))
        results,restore=configure_frame_queue(device,rs,2)
        self.assertEqual(results[0]['actual'],2);self.assertEqual(sensor.value,2)
        restore_frame_queue(restore,rs);self.assertEqual(sensor.value,16)
        with self.assertRaises(ValueError):configure_frame_queue(device,rs,0)

    def test_unavailable_queue_is_reported(self):
        class Sensor:
            def supports(self,option):return True
            def get_option(self,option):return 16
            def set_option(self,option,value):raise RuntimeError('read only')
        rs=SimpleNamespace(option=SimpleNamespace(frames_queue_size=1))
        results,_=configure_frame_queue(SimpleNamespace(query_sensors=lambda:[Sensor()]),rs,2)
        self.assertIn('error',results[0]);self.assertNotIn('actual',results[0])

if __name__=='__main__':unittest.main()
