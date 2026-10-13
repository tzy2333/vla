import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collector.realsense_camera import yuyv_to_rgb


class NativeColorTests(unittest.TestCase):
    def test_black_white_and_rgb_order(self):
        # BT.601 limited-range reference blocks: black, white, red, green, blue.
        yuv=[(16,128,128),(235,128,128),(81,90,240),(145,54,34),(41,240,110)]
        packed=np.array([[[y,u],[y,v]] for y,u,v in yuv],dtype=np.uint8)
        rgb=yuyv_to_rgb(packed,2,5)
        expected=np.array([[0,0,0],[255,255,255],[255,0,0],[0,255,0],[0,0,255]])
        self.assertLessEqual(np.abs(rgb[:,0].astype(int)-expected).max(),1)
        self.assertEqual(rgb.dtype,np.uint8)
        np.testing.assert_array_equal(rgb[:,0],rgb[:,1])

    def test_sdk_byte_and_uint16_views_agree(self):
        raw=np.random.default_rng(1).integers(0,256,(4,8,2),dtype=np.uint8)
        np.testing.assert_array_equal(yuyv_to_rgb(raw,8,4),yuyv_to_rgb(raw.view(np.uint16).reshape(4,8),8,4))

    def test_invalid_layout_rejected(self):
        for raw,w,h in [(np.zeros((4,8),dtype=np.uint8),8,4),(np.zeros((4,8),dtype=np.float32),8,4)]:
            with self.assertRaises(ValueError):yuyv_to_rgb(raw,w,h)


if __name__=='__main__':unittest.main()
