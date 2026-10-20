import importlib.util
from pathlib import Path
import time
import unittest
import tempfile
import threading
from unittest.mock import Mock, patch

path=Path(__file__).resolve().parents[1]/'scripts/bag_recorder.py'
spec=importlib.util.spec_from_file_location('bag_recorder_test',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class RecordingTests(unittest.TestCase):
    def test_selected_arm_filters_all_unselected_arm_topics(self):
        state = self.state()
        state['topics'] += [{'topic': '/piper/'+s+'_bus/joint_states', 'ok': True} for s in ('left','right')]
        state['topics'][0]['topic'] = '/cam_h/color/image_raw'
        state['record_topics'] += ['/piper/'+s+'_bus/joint_states' for s in ('left','right')]
        state['record_topics'] += ['/agilex/teleop/right_leader/status', '/piper/right_leader/command']
        for side in ('left','right'):
            chosen = module.selected_hardware(state, [side])
            self.assertEqual(chosen['arms'], [side])
            self.assertEqual(module.ready_error(chosen,time.monotonic(),3,[side],state['cameras'],False), '')
            other = 'left' if side == 'right' else 'right'
            self.assertFalse(any('/'+other+'_' in topic for topic in chosen['record_topics']))
        self.assertEqual(state['arms'], ['left','right'])

    def test_cli_exits_after_external_stop_releases_the_episode(self):
        recorder=module.Recorder.__new__(module.Recorder)
        recorder.guard=threading.RLock()
        recorder.output_file=Path('/tmp/episode/data.bag')
        recorder.proc=None;recorder.manifest=None
        recorder.root=Path('/tmp');recorder.minimum=0
        recorder.reason=Mock(return_value='')
        def start(_):
            recorder.proc=Mock();recorder.proc.poll.return_value=None
            recorder.manifest={'state':'recording'}
            return Mock(success=True)
        def remote_stop(_):
            recorder.proc=None;recorder.manifest['state']='complete'
        recorder.start=start
        with patch.object(module.rospy,'is_shutdown',side_effect=[False,False,True]) as shutdown, \
             patch.object(module.rospy,'get_param',return_value=120), \
             patch.object(module.rospy,'loginfo'), patch.object(module.time,'sleep',side_effect=remote_stop) as sleep:
            recorder.run()
        self.assertEqual(shutdown.call_count,2)
        sleep.assert_called_once()

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
