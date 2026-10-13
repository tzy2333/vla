import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch,Mock
from types import SimpleNamespace
from collector.orbbec_camera import RealCameras
from collector.platform_contract import interface_fields,validate_interface_options,camera_bindings

class DevicesTests(unittest.TestCase):
    def test_end_effector_descriptor_follows_imported_configuration(self):
        from collector.rf_robot_adapter import RFObservationAdapter
        from collector.rf_kinematics import CandidateKinematics
        adapter=RFObservationAdapter.__new__(RFObservationAdapter);adapter.fk=CandidateKinematics()
        self.assertEqual(adapter.describe()['end_effectors'],[])
        adapter.end_effector_config={'left':'gripper','right':'dexterous_hand'}
        groups=adapter.describe()['end_effectors']
        self.assertEqual([g['id'] for g in groups],['gripper','right_hand'])
        self.assertEqual(groups[1]['channels'][0]['packet_path'],['raw_observation',68])
        adapter.end_effector_config={'left':'none','right':'gripper'}
        groups=adapter.describe()['end_effectors']
        self.assertEqual(len(groups),1);self.assertEqual(groups[0]['kind'],'gripper')

    def test_terminal_follows_descriptor_and_selected_capture_without_fake_events(self):
        from monitor import Feed
        desc=dict(channels=[dict(id='a',index=0,unit='rad')],modules=[dict(id='arm',label='测试臂',channels=['a'])],end_effectors=[dict(id='tool',label='测试夹爪',channels=[dict(packet_path=['tool_value'])])])
        status=dict(capabilities={'robot':desc},phase='RECORDING',capture_selection={'modules':[],'cameras':[]},feedback_valid=True,robot_feedback={'packet_id':1,'session':'s','received_ns':1,'state':[.1],'tool_value':.4},health={'robot':{'healthy':True},'cameras':{}})
        feed=Feed();events=feed.events(status)
        self.assertFalse(any(e[0]=='JOINT' for e in events));self.assertTrue(any(e[1]=='测试夹爪' for e in events));self.assertFalse(any('灵巧手' in str(e) for e in events))
        self.assertEqual(feed.events(status),[])
        status['feedback_valid']=False;status['robot_feedback']['packet_id']=2
        self.assertFalse(any(e[0]=='HAND' for e in feed.events(status)))
    def test_expired_fifo_frames_are_dropped_not_retimestamped(self):
        from collector.realsense_camera import queued_arrival_expired
        sample={'wall_ns':1_800_000_000_000_000_000}
        now=sample['wall_ns']//1_000_000
        self.assertTrue(queued_arrival_expired(now-800,sample))
        self.assertFalse(queued_arrival_expired(now-10,sample))
        self.assertFalse(queued_arrival_expired(now+100,sample))
        self.assertFalse(queued_arrival_expired(123,sample))
        self.assertFalse(queued_arrival_expired(None,sample))
    def test_independent_lifecycle_never_reopens_another_camera(self):
        config={'camera_bindings':{k:dict(driver='orbbec',serial=k,width=640,height=480,fps=30) for k in ('head','wrist')}}
        config['released_cameras']=['wrist']
        first=Mock();first.health_check.return_value={'healthy':True}
        second=Mock();second.health_check.return_value={'healthy':True}
        with patch('collector.orbbec_camera.OrbbecRGB',side_effect=[first,second]) as factory:
            manager=RealCameras(None,config)
            self.assertEqual(factory.call_count,1)
            manager.connect('wrist');self.assertEqual(factory.call_count,2)
            manager.release('head');first.close.assert_called_once();second.close.assert_not_called()
            self.assertEqual(manager.health_check()['head']['error'],'相机已释放')
            self.assertTrue(manager.health_check()['wrist']['healthy'])
            manager.close();second.close.assert_called_once()

    def test_unknown_camera_does_not_close_hardware(self):
        manager=RealCameras(None,{'camera_bindings':{}})
        with self.assertRaises(ValueError):manager.release('unknown')
        with self.assertRaises(ValueError):manager.connect('unknown')

    def test_plugin_configuration_is_typed_and_independent_of_rf(self):
        factory=SimpleNamespace(configuration_fields=[dict(key='endpoint',label='SDK address',type='text',default='local'),dict(key='dof',type='integer',min=1,max=40,default=7)])
        entry=SimpleNamespace(load=lambda:factory)
        with patch('collector.platform_contract.importlib.metadata.entry_points',return_value=[entry]):
            self.assertEqual(validate_interface_options('seven',{'endpoint':'sdk-host','dof':7}),{'endpoint':'sdk-host','dof':7})
            for values in ({'dof':0},{'dof':True},{'dof':7,'module_path':'bad'}):
                with self.assertRaises(ValueError):validate_interface_options('seven',values)

    def test_duplicate_serial_and_invalid_dimensions_rejected(self):
        binding=dict(driver='realsense',serial='one',width=848,height=480,fps=60)
        with self.assertRaises(ValueError):camera_bindings({'camera_bindings':{'a':binding,'b':binding}})
        with self.assertRaises(ValueError):camera_bindings({'camera_bindings':{'a':dict(binding,width=0)}})

if __name__=='__main__':unittest.main()
