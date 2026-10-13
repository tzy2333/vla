"""Automatic preparation regression, using local packet/camera fixtures only."""
import json,threading,time,unittest
from unittest.mock import patch
import test_modular_capture as fixtures

class AutomaticCaptureTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.ModularCaptureTests();self.fixture.setUp();self.app=self.fixture.connect()
        self.calls=[];self.released=[];c=self.app.cameras;c.errors={}
        def connect(key):self.calls.append(key);c.bad.discard(key);c.errors.pop(key,None)
        def release(key):self.released.append(key);c.bad.add(key);c.errors[key]='released';c.frames[key].clear()
        c.connect=connect;c.release=release
        self.choice={'modules':['left_arm'],'cameras':['head']}
    def tearDown(self):self.fixture.tearDown()
    def start(self):return self.app.start('auto preparation test','qa',fps=10,capture_selection=self.choice)
    def stop(self):
        time.sleep(.25)
        if self.app.phase=='RECORDING':self.app.stop()
        self.assertEqual(self.app.phase,'REVIEW',self.app.last_error)
    def test_start_connects_only_selected_offline_camera(self):
        self.app.cameras.bad.update(['head','left_wrist']);self.start();self.stop()
        self.assertEqual(self.calls,['head']);self.assertIn('left_wrist',self.app.cameras.bad)
    def test_healthy_stream_not_reopened(self):
        self.start();self.stop();self.assertEqual(self.calls,[])
    def test_transient_open_failure_retries_once(self):
        c=self.app.cameras;c.bad.add('head');normal=c.connect
        def flaky(key):
            if not self.calls:self.calls.append(key);c.errors[key]='startup timeout'
            else:normal(key)
        c.connect=flaky;self.start();self.stop();self.assertEqual(self.calls,['head','head'])
    def test_permanent_failure_does_not_create_episode_and_can_retry(self):
        c=self.app.cameras;c.bad.add('head');normal=c.connect
        def fail(key):self.calls.append(key);c.errors[key]='startup timeout'
        c.connect=fail;before=list(self.app.raw.iterdir())
        with self.assertRaisesRegex(ValueError,'已重试'):self.start()
        self.assertEqual(self.app.phase,'IDLE');self.assertEqual(before,list(self.app.raw.iterdir()))
        self.assertIn('head',self.app.config['released_cameras']);self.assertTrue(self.app.preparation_error)
        c.connect=normal;self.start();self.stop();self.assertIsNone(self.app.preparation_error)
    def test_next_start_keeps_previous_review_manifest_unchanged(self):
        ep=self.start()['episode_id'];self.stop();path=self.app.episode_path(ep)/'manifest.json';before=path.read_bytes()
        ep2=self.start()['episode_id'];self.stop();self.assertNotEqual(ep,ep2);self.assertEqual(path.read_bytes(),before)
    def test_failure_restores_review_and_preserves_previous_episode(self):
        ep=self.start()['episode_id'];self.stop();path=self.app.episode_path(ep)/'manifest.json';before=path.read_bytes()
        c=self.app.cameras;c.bad.add('head')
        def fail(key):c.errors[key]='offline'
        c.connect=fail
        with self.assertRaises(ValueError):self.start()
        self.assertEqual(self.app.phase,'REVIEW');self.assertEqual(self.app.last_episode,ep);self.assertEqual(path.read_bytes(),before)
    def test_status_readable_and_mutations_rejected_during_preparation(self):
        entered=threading.Event();finish=threading.Event();out=[]
        def prepare(keys):entered.set();finish.wait(3)
        def start_thread():
            try:out.append(self.start())
            except Exception as e:out.append(e)
        with patch.object(self.app,'_prepare_camera_streams',prepare):
            thread=threading.Thread(target=start_thread);thread.start();self.assertTrue(entered.wait(1))
            stamp=time.monotonic();self.assertEqual(self.app.status()['phase'],'PREPARING');self.assertLess(time.monotonic()-stamp,.2)
            for operation in [self.start,lambda:self.app.release_cameras('head'),lambda:self.app.reconnect_cameras('head'),self.app.reconnect_robot]:
                with self.assertRaises(ValueError):operation()
            finish.set();thread.join(3)
        self.assertIsInstance(out[0],dict);self.stop()
    def test_invalid_request_does_not_open_devices(self):
        self.app.cameras.bad.add('head')
        with self.assertRaises(ValueError):self.app.start('', 'qa',capture_selection=self.choice)
        with self.assertRaises(ValueError):self.app.start('qa','qa',fps=999,capture_selection=self.choice)
        self.assertEqual(self.calls,[]);self.assertEqual(self.app.phase,'IDLE')

if __name__=='__main__':unittest.main()
