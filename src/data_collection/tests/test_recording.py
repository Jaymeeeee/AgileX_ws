import importlib.util
from pathlib import Path
import time
import unittest
import tempfile

path=Path(__file__).resolve().parents[1]/'scripts/bag_recorder.py'
spec=importlib.util.spec_from_file_location('bag_recorder_test',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class RecordingTests(unittest.TestCase):
    def test_completed_output_allowed_but_active_output_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'episode';path.mkdir()
            bag=path/'data.bag';bag.write_bytes(b'previous recording')
            self.assertEqual(module.output_error(path),'')
            (path/'data.bag.active').touch()
            self.assertIn('.active',module.output_error(path))
            self.assertEqual(bag.read_bytes(),b'previous recording')

    def test_output_and_metadata_symlinks_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'episode';target=Path(tmp)/'other';target.write_text('keep')
            path.symlink_to(target)
            self.assertTrue(module.output_error(path));path.unlink()
            path.mkdir()
            (path/'manifest.json').symlink_to(target)
            self.assertTrue(module.output_error(path))
            self.assertEqual(target.read_text(),'keep')

    def test_output_rejects_old_bag_path_and_regular_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(module.output_error(Path(tmp)/'episode.bag'))
            path=Path(tmp)/'episode';path.write_text('keep')
            self.assertTrue(module.output_error(path))
            self.assertEqual(path.read_text(),'keep')

    def state(self):
        return dict(state='ready',stamp=time.time(),arms=['left','right'],cameras=['cam_h','cam_l','cam_r'],
                    topics=[dict(ok=True)],record_topics=['/cam_h/color/image_raw'])
    def check(self,state,age=0,partial=False):
        return module.ready_error(state,time.monotonic()-age,3,['left','right'],['cam_h','cam_l','cam_r'],partial)
    def test_live_ready(self):self.assertEqual(self.check(self.state()),'')
    def test_missing_leader_targets_block_recording_even_with_partial_selection(self):
        state=self.state();state.update(state='degraded',reason='leader targets not received')
        self.assertIn('leader targets',self.check(state))
        self.assertIn('leader targets',self.check(state,partial=True))
    def test_stale_status(self):self.assertTrue(self.check(self.state(),age=5))
    def test_ready_flag_not_enough(self):
        state=self.state();state['topics'][0]['ok']=False
        self.assertTrue(self.check(state))
    def test_partial_requires_opt_in(self):
        state=self.state();state.update(arms=[],cameras=['cam_l'])
        self.assertTrue(self.check(state))
        self.assertEqual(self.check(state,partial=True),'')
    def test_clock_stale(self):
        state=self.state();state['stamp']-=10
        self.assertTrue(self.check(state))
    def test_running_recording_rejects_config_change(self):
        recorder=module.Recorder.__new__(module.Recorder)
        recorder.hardware=self.state()
        recorder.hardware['config']={'fps':15}
        recorder.last_received=time.monotonic();recorder.timeout=3
        recorder.required_arms=['left','right'];recorder.required_cameras=['cam_h','cam_l','cam_r']
        recorder.allow_partial=False;recorder.proc=object()
        recorder.manifest={'selection':{'arms':['left','right'],'cameras':['cam_h','cam_l','cam_r']},
                           'hardware':{'config':{'fps':15}}}
        self.assertEqual(recorder.reason(),'')
        recorder.hardware['config']['fps']=30
        self.assertIn('changed',recorder.reason())


if __name__=='__main__':unittest.main()
