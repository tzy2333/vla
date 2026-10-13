import copy
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from collector.camera_inventory import CameraInventory
from collector.platform_management import PlatformManagement
from collector.storage import atomic_json,load_json

class InventoryTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.data=self.root/'data';self.data.mkdir()
  self.old=dict(driver='realsense',serial='existing',width=848,height=480,fps=60)
  cfg=dict(mode='real',fps=50,camera_bindings={'head':self.old},camera_shapes={'head':[480,848,3]},camera_native_fps={'head':60},device_sources={'head':'realsense_rgb'})
  atomic_json(self.root/'config.json',cfg)
  self.app=SimpleNamespace(data=self.data,phase='IDLE',writer=None,lock=threading.RLock(),config=copy.deepcopy(cfg),
    cameras=SimpleNamespace(bindings={'head':self.old},errors={},devices={'head':Mock()}),_save_camera_state=Mock())
  self.management=PlatformManagement(self.app,self.root,self.root/'config.json');self.management.settings=Mock(return_value={})
  self.inv=CameraInventory(self.app,self.management);self.inv.last=time.monotonic();self.profile=dict(width=848,height=480,fps=60,stream_format='rgb8')
  self.inv.cache=dict(devices=[dict(driver='realsense',serial='new',label='new',profiles=[self.profile])],errors=[],scanned_at='test')
 def tearDown(self):self.temp.cleanup()
 def body(self):return dict(revision=self.management._disk_config()[1],key='wrist',label='wrist',serial='new',driver='realsense',profile=self.profile)
 def test_incremental_add_never_reopens_existing(self):
  camera=self.app.cameras.devices['head'];self.inv.bind(self.body());camera.close.assert_not_called()
  self.assertEqual(self.app.cameras.errors['wrist'],'相机已释放');self.assertEqual(self.app.config['camera_shapes']['wrist'],[480,848,3])
  self.assertIn('wrist',load_json(self.root/'config.json')['camera_bindings']);self.app._save_camera_state.assert_called_once_with(['wrist'],True)
 def test_recording_defers_changes_and_enumeration(self):
  self.app.phase='RECORDING'
  with self.assertRaises(ValueError):self.inv.bind(self.body())
  with patch('collector.camera_inventory.subprocess.run') as run:
   self.assertTrue(self.inv.refresh()['paused']);run.assert_not_called()
 def test_stale_discovery_rejected(self):
  self.inv.last-=60
  with self.assertRaises(ValueError):self.inv.bind(self.body())
 def test_unsupported_profile_rejected(self):
  body=self.body();body['profile']=dict(width=1,height=1,fps=60)
  with self.assertRaises(ValueError):self.inv.bind(body)
 def test_low_camera_rate_not_silently_lowered(self):
  self.profile['fps']=30
  with self.assertRaises(ValueError):self.inv.bind(self.body())
  self.assertEqual(self.app.config['fps'],50)
 def test_enumeration_timeout_is_visible(self):
  import subprocess
  self.inv.last=0
  with patch('collector.camera_inventory.subprocess.run',side_effect=subprocess.TimeoutExpired('sdk',8)):
   self.inv.refresh();self.inv.thread.join(2)
  self.assertFalse(self.inv.view()['scanning']);self.assertTrue(self.inv.view()['errors'])
 def test_invalid_id_rejected(self):
  body=self.body();body['key']='../escape'
  with self.assertRaises(ValueError):self.inv.bind(body)

if __name__=='__main__':unittest.main()
