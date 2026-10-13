"""Management checks never use real camera hardware or send robot commands."""
import copy
import json
from pathlib import Path
import socket
import struct
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch, MagicMock

from collector.platform_management import PlatformManagement, summarize
from collector.rf_kinematics import CandidateKinematics
from collector.storage import atomic_json
from collector.engine import Collector
from collector.unavailable_camera import UnavailableCameras

ROOT=Path(__file__).resolve().parents[1]


class ManagementTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.config=json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
        self.config['rf_robot'].update(host='127.0.0.1',port=9)
        self.cfg=self.root/'config.json';atomic_json(self.cfg,self.config)
        self.robot=SimpleNamespace(fk=CandidateKinematics(),health_check=lambda:{'healthy':True})
        self.rows=[]
        self.app=SimpleNamespace(data=self.root,config=copy.deepcopy(self.config),robot=self.robot,
                                 lock=threading.RLock(),phase='IDLE',writer=None,episodes=lambda:self.rows)
        self.manager=PlatformManagement(self.app,self.root,self.cfg)

    def tearDown(self):self.tmp.cleanup()

    def test_metrics_do_not_invent_success_or_mix_mock_and_legacy_labels(self):
        def ep(**kw):return dict(mock=False,device_sources={'robot':'rf_udp_20010'},fps=50,**kw)
        rows=[ep(status='COMMITTED',frame_count=100,success=True),
              ep(status='REVIEW',frame_count=50),
              ep(status='FAULT',frame_count=0,fault='camera'),
              ep(status='DISCARDED',frame_count=20,task_outcome='failure',outcome_source='human_review_v1'),
              ep(status='COMMITTED',frame_count=30,task_outcome='success',outcome_source='human_review_v1'),
              {'mock':True,'status':'COMMITTED','success':True,'frame_count':999}]
        s=summarize(rows)
        self.assertEqual((s['total'],s['completed'],s['completion_rate']),(5,4,80))
        self.assertEqual((s['labeled'],s['successes'],s['task_success_rate']),(2,1,50))
        self.assertEqual(s['excluded_legacy_or_simulated'],1)
        self.assertIsNone(summarize([])['task_success_rate'])
        self.assertIsNone(summarize(rows[:2])['task_success_rate'])

    def test_task_persistence_revision_and_linked_progress(self):
        t=self.manager.save_task({'name':'放入托盘','target_episodes':10})
        self.rows.append(dict(task_id=t['task_id'],mock=False,device_sources={'robot':'rf_udp_20010'},status='COMMITTED',frame_count=100,fps=50))
        other=PlatformManagement(self.app,self.root,self.cfg)
        self.assertEqual(other.tasks()['tasks'][0]['metrics']['kept'],1)
        with self.assertRaises(ValueError):other.save_task({'task_id':t['task_id'],'name':'new','revision':'old'})
        with self.assertRaises(ValueError):other.start_capture({'task_id':'unknown'})

    def test_settings_reject_incompatible_rate_unknown_controls_and_stale_writes(self):
        rev=self.manager.settings()['revision'];before=self.cfg.read_bytes()
        data={'revision':rev,'group':'capture','name':'Robot','location':'Lab','fps':50,'camera_profile':'dual720'}
        with self.assertRaises(ValueError):self.manager.save_settings(data)
        self.assertEqual(before,self.cfg.read_bytes())
        with self.assertRaises(ValueError):self.manager.save_settings({'revision':rev,'group':'controller','speed':100})
        result=self.manager.save_settings(dict(data,fps=30))
        self.assertTrue(result['settings']['camera_pending'])
        self.assertEqual(self.app.config['fps'],50)
        self.assertFalse(json.loads(self.cfg.read_text(encoding='utf-8'))['compute_ee_online'])
        with self.assertRaises(ValueError):self.manager.save_settings(dict(data,fps=30))
        self.assertEqual(len(list((self.root/'management/settings_backups').glob('*.json'))),1)

    def test_independent_camera_configuration_is_saved_without_touching_live_devices(self):
        binding=dict(driver='realsense',serial='right-camera',label='右腕',width=848,height=480,fps=60)
        before=copy.deepcopy(self.app.config)
        result=self.manager.save_settings(dict(revision=self.manager.settings()['revision'],group='cameras',camera_bindings={'right_wrist':binding}))
        self.assertEqual(result['settings']['camera_bindings'],{'right_wrist':binding})
        self.assertEqual(self.app.config,before)
        self.assertTrue(result['settings']['camera_pending'])

    def test_interface_parameters_are_validated_saved_and_pending(self):
        from collector.platform_contract import validate_interface_options
        options=validate_interface_options('rf_udp_20010',{'host':'192.168.11.101'})
        result=self.manager.save_settings(dict(revision=self.manager.settings()['revision'],group='interface',robot_adapter='rf_udp_20010',options=options))
        self.assertEqual(json.loads(self.cfg.read_text())['rf_robot']['host'],'192.168.11.101')
        self.assertEqual(self.app.config['rf_robot']['host'],'127.0.0.1')
        self.assertTrue(result['settings']['camera_pending'])

    def test_end_effector_choices_are_scoped_to_adapter_and_do_not_send_commands(self):
        self.app.descriptor={'end_effector_slots':[{'id':'right','label':'右侧','options':[{'id':'none'},{'id':'dexterous_hand'}]}]}
        with patch('collector.platform_management.socket.socket') as sock:
            result=self.manager.save_settings(dict(revision=self.manager.settings()['revision'],group='end_effectors',choices={'right':'dexterous_hand'}))
            sock.assert_not_called()
        cfg=json.loads(self.cfg.read_text())
        self.assertEqual(cfg['robot_end_effectors']['rf_udp_20010'],{'right':'dexterous_hand'})
        self.assertTrue(result['settings']['camera_pending'])
        with self.assertRaises(ValueError):self.manager.save_settings(dict(revision=result['settings']['revision'],group='end_effectors',choices={'right':'unknown'}))

    def test_manual_model_edit_is_consistent_pending_and_not_sampled_calibration(self):
        original=copy.deepcopy(self.robot.fk.model)
        arms={s:{'zero_raw':list(a['encoder_to_model']['zero_raw']),
                 'sign':list(a['encoder_to_model']['sign']),
                 'origins_m':[list(j['origin_m']) for j in a['joints']]} for s,a in original['arms'].items()}
        arms['left']['zero_raw'][0]=.01;arms['left']['sign'][0]*=-1
        with patch.object(CandidateKinematics,'forward',side_effect=AssertionError('No FK')):
            result=self.manager.save_settings({'revision':self.manager.settings()['revision'],'group':'model',
                      'tcp_translation_m':original['tcp_translation_m'],'arms':arms})
            cfg=json.loads(self.cfg.read_text(encoding='utf-8'));candidate=CandidateKinematics()
            candidate.model=cfg['robot_model_override']['model'];candidate.mapping=cfg['robot_model_override']['mapping']
            converted=candidate.angles_from_observation([0.0]*96)
        self.assertTrue(result['settings']['model_pending'])
        self.assertFalse(converted['left']['zero_calibration_verified'])
        self.assertEqual(self.robot.fk.model,original)
        self.assertEqual(candidate.mapping['arm_zero_pose']['encoder_offsets']['left'][0],.01)

    def test_reset_confirmation_busy_and_stale_feedback_do_not_send(self):
        data={'request_id':'a'*32,'confirmed':True}
        with patch('collector.platform_management.socket.socket') as sock:
            with self.assertRaises(ValueError):self.manager.reset_robot(dict(data,confirmed=False))
            with self.assertRaises(ValueError):self.manager.reset_robot(dict(data,code=2005))
            self.app.phase='RECORDING'
            with self.assertRaises(ValueError):self.manager.reset_robot(data)
            self.app.phase='IDLE';self.robot.health_check=lambda:{'healthy':False}
            with self.assertRaises(ValueError):self.manager.reset_robot(data)
            sock.assert_not_called()

    def test_reset_exact_code_durable_dedup_and_no_completion_claim(self):
        data={'request_id':'a'*32,'confirmed':True}
        with patch('collector.platform_management.socket.socket') as factory:
            send=factory.return_value.__enter__.return_value.sendto;send.return_value=4
            result=self.manager.reset_robot(data)
            send.assert_called_once_with(struct.pack('!i',2001),('127.0.0.1',9))
            self.assertFalse(result['completion_confirmed'])
            self.assertEqual(result['status'],'udp_sent_unconfirmed')
            fresh=PlatformManagement(self.app,self.root,self.cfg)
            self.assertEqual(fresh.reset_robot(data),result)
            with self.assertRaises(ValueError):fresh.reset_robot(dict(data,request_id='b'*32))
            self.assertEqual(send.call_count,1)

    def test_udp_error_is_recorded_without_retry(self):
        data={'request_id':'f'*32,'confirmed':True}
        with patch('collector.platform_management.socket.socket') as factory:
            send=factory.return_value.__enter__.return_value.sendto;send.side_effect=OSError('test transport failure')
            result=self.manager.reset_robot(data)
            self.assertEqual(result['status'],'send_error_unconfirmed')
            self.manager.reset_robot(data)
            self.assertEqual(send.call_count,1)

    def test_unavailable_camera_has_no_images_and_recording_is_blocked(self):
        robot=MagicMock();robot.calibration_snapshot.return_value={};robot.health_check.return_value={'healthy':True}
        from collector.rf_robot_adapter import RFObservationAdapter
        robot.fk=CandidateKinematics()
        robot.describe.return_value=RFObservationAdapter.describe(robot)
        with patch('collector.orbbec_camera.RealCameras',side_effect=RuntimeError('camera is busy')),patch('collector.engine.RFObservationAdapter',return_value=robot):
            app=Collector(copy.deepcopy(self.config),self.root)
        try:
            self.assertIsInstance(app.cameras,UnavailableCameras)
            self.assertTrue(all(not h['healthy'] and not h['mock'] for h in app.cameras.health_check().values()))
            self.assertIsNone(app.cameras.latest('head'))
            # Automatic start may rebuild the unavailable manager; keep that retry isolated too.
            with patch('collector.orbbec_camera.RealCameras',side_effect=ValueError('camera is busy')):
                with self.assertRaises(ValueError):app.start('blocked','tester',fps=50)
        finally:app.close()

    def test_unlabeled_review_is_not_converted_to_failure(self):
        app=Collector.__new__(Collector);app.lock=threading.RLock();app.phase='REVIEW';app.raw=self.root/'raw';app.raw.mkdir()
        ep='20261013_123456_aaaaaaaa';folder=app.raw/ep;folder.mkdir();app.last_episode=ep
        atomic_json(folder/'manifest.json',{'status':'REVIEW','frame_count':1,'fault':None})
        result=app.review(ep,True,None)
        self.assertIsNone(result['success']);self.assertEqual(result['task_outcome'],'unknown')
        self.assertEqual(app.phase,'IDLE')


if __name__=='__main__':unittest.main()
