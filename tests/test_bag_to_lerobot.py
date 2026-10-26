"""Regression checks for bag value semantics and source-time alignment."""
import copy
import json
from types import SimpleNamespace
import unittest

import numpy as np

from piper_control.recording.bag_to_lerobot import (
    MOTOR_KEYS,
    Series,
    build_plan,
    decode_rgb,
    feature_schema,
    header_ns,
    joint_vector,
    motor_samples,
    numeric_arrays,
    numeric_frame,
    sample_indices,
    verify_actions,
)


NAMES = ["right_joint_%d" % i for i in range(1, 7)] + ["right_gripper_width"]


def joint_message(values=None, names=None):
    return SimpleNamespace(
        name=list(NAMES if names is None else names),
        position=list([0.] * 7 if values is None else values),
    )


def rgb_message(data, width=2, height=2, step=8, encoding="rgb8"):
    return SimpleNamespace(data=bytes(data), width=width, height=height,
                           step=step, encoding=encoding, is_bigendian=0)


def source_series(stamps, values=None):
    stamps = np.array(stamps, dtype=np.int64)
    if values is None:
        values = [np.full(7, index / 10., dtype=np.float32) for index in range(len(stamps))]
    series = Series(stamps=list(stamps), bag_stamps=list(stamps + 10_000_000),
                    ordinals=list(range(len(stamps))), values=values)
    series.finish()
    return series


def evidence_row(values, names=None):
    return {"joint_names": list(NAMES if names is None else names), "action": list(values),
            "source": "HUMAN", "action_evidence": "forwarded_can_target"}


def diagnostic_field(key, value):
    return SimpleNamespace(key=key, value=json.dumps(value))


def motor_diagnostics():
    statuses = []
    for index in range(1, 7):
        values = {"current": index * .25, "torque": index * -.125,
                  "stamp": 1. + index * .01, "source": "feedback", "valid": True,
                  "speed": index * .5}
        statuses.append(SimpleNamespace(name="right/follower/motor_feedback/%d" % index,
                                        values=[diagnostic_field(key, value)
                                                for key, value in values.items()]))
    # The aggregate header is intentionally different from each motor's stamp.
    return SimpleNamespace(status=statuses, header=SimpleNamespace(
        stamp=SimpleNamespace(sec=99, nanosec=0)))


def replace_diagnostic_field(status, key, value):
    for field in status.values:
        if field.key == key:
            field.value = json.dumps(value)
            return
    raise AssertionError("Unknown fixture field " + key)


class JointVectorTests(unittest.TestCase):
    def test_joint_names_reorder_values_without_rescaling_or_clipping(self):
        expected = np.array([.1, -.2, .3, -.4, .5, -.6, -.0001], dtype=np.float32)
        order = [6, 2, 0, 5, 1, 4, 3]
        msg = joint_message(expected[order], [NAMES[i] for i in order])
        result = joint_vector(msg, NAMES)
        self.assertEqual(result.dtype, np.dtype("float32"))
        np.testing.assert_array_equal(result, expected)

    def test_missing_extra_duplicate_and_misaligned_names_fail(self):
        malformed = [
            joint_message([0.] * 6, NAMES[:-1]),
            joint_message([0.] * 8, NAMES + ["left_joint_1"]),
            joint_message([0.] * 7, NAMES[:-1] + [NAMES[0]]),
            joint_message([0.] * 6, NAMES),
        ]
        for msg in malformed:
            with self.subTest(names=msg.name, dimension=len(msg.position)):
                with self.assertRaises(ValueError):
                    joint_vector(msg, NAMES)

    def test_nonfinite_joint_and_gripper_values_fail(self):
        for bad in (float("nan"), float("inf"), -float("inf")):
            for index in (0, 6):
                values = [0.] * 7
                values[index] = bad
                with self.subTest(value=bad, index=index):
                    with self.assertRaises(ValueError):
                        joint_vector(joint_message(values), NAMES)


class RGBTests(unittest.TestCase):
    def test_rgb_row_padding_is_not_interpreted_as_pixels(self):
        data = [10, 20, 30, 40, 50, 60, 222, 223,
                70, 80, 90, 100, 110, 120, 224, 225]
        expected = np.array([[[10, 20, 30], [40, 50, 60]],
                             [[70, 80, 90], [100, 110, 120]]], dtype=np.uint8)
        result = decode_rgb(rgb_message(data))
        np.testing.assert_array_equal(result, expected)
        self.assertTrue(result.flags.c_contiguous)

    def test_bgr_is_swapped_once_after_removing_padding(self):
        msg = rgb_message([30, 20, 10, 60, 50, 40, 222, 223],
                          height=1, encoding="bgr8")
        np.testing.assert_array_equal(
            decode_rgb(msg), np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8))

    def test_invalid_shape_short_payload_and_depth_encoding_fail(self):
        malformed = [
            rgb_message([0] * 16, width=0),
            rgb_message([0] * 10, step=5),
            rgb_message([0] * 15),
            rgb_message([0] * 16, encoding="32FC1"),
        ]
        for msg in malformed:
            with self.subTest(width=msg.width, step=msg.step, encoding=msg.encoding):
                with self.assertRaises(ValueError):
                    decode_rgb(msg)


class SourceTimeTests(unittest.TestCase):
    def test_source_timestamp_preserves_integer_nanoseconds(self):
        msg = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1792824722, nanosec=239762306)))
        self.assertEqual(header_ns(msg), 1792824722239762306)

    def test_zero_header_does_not_fall_back_to_bag_receipt(self):
        msg = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0)))
        with self.assertRaises(ValueError):
            header_ns(msg)

    def test_previous_sampling_never_uses_future_samples(self):
        stamps = np.array([100, 200, 400], dtype=np.int64)
        grid = np.array([99, 100, 150, 399, 400, 500], dtype=np.int64)
        np.testing.assert_array_equal(sample_indices(stamps, grid, "previous"),
                                      [-1, 0, 0, 1, 2, 2])

    def test_nearest_sampling_tie_prefers_earlier_source_frame(self):
        stamps = np.array([100, 200, 400], dtype=np.int64)
        grid = np.array([90, 150, 151, 300, 301, 500], dtype=np.int64)
        np.testing.assert_array_equal(sample_indices(stamps, grid, "nearest"),
                                      [0, 0, 1, 1, 2, 2])


class SeriesTests(unittest.TestCase):
    def test_sorting_keeps_numeric_values_receipt_times_and_ordinals_together(self):
        series = Series(stamps=[300, 100, 200], bag_stamps=[330, 110, 220],
                        ordinals=[0, 1, 2], values=[np.array([3.]), np.array([1.]), np.array([2.])])
        series.finish()
        np.testing.assert_array_equal(series.stamps, [100, 200, 300])
        np.testing.assert_array_equal(series.bag_stamps, [110, 220, 330])
        np.testing.assert_array_equal(series.ordinals, [1, 2, 0])
        np.testing.assert_array_equal(series.values, [[1.], [2.], [3.]])

    def test_identical_duplicate_numeric_samples_do_not_create_extra_frames(self):
        series = Series(stamps=[100, 100, 200], bag_stamps=[110, 120, 220],
                        ordinals=[0, 1, 2], values=[np.array([1.]), np.array([1.]), np.array([2.])])
        series.finish()
        np.testing.assert_array_equal(series.stamps, [100, 200])
        np.testing.assert_array_equal(series.values, [[1.], [2.]])

    def test_conflicting_values_at_one_source_time_are_rejected(self):
        series = Series(stamps=[100, 100], bag_stamps=[110, 120],
                        ordinals=[0, 1], values=[np.array([1.]), np.array([2.])])
        with self.assertRaises(ValueError):
            series.finish()

    def test_inconsistent_numeric_dimensions_are_rejected(self):
        series = Series(stamps=[100, 200], bag_stamps=[110, 220],
                        ordinals=[0, 1], values=[np.array([1.]), np.array([2., 3.])])
        with self.assertRaises(ValueError):
            series.finish()


class FramePlanTests(unittest.TestCase):
    def config(self):
        return {"dataset": {"fps": 10}, "sync": {"min_frames": 2, "max_age_sec": .15},
                "sources": {key: {"topic": "/test/" + key}
                            for key in ("state", "action", "cam_h", "cam_r")}}

    def streams(self):
        return {key: source_series(np.arange(5) * 100_000_000 + start)
                for key, start in (("state", 1_000_000_000), ("action", 1_050_000_000),
                                   ("cam_h", 1_020_000_000), ("cam_r", 1_080_000_000))}

    def test_state_and_action_come_from_distinct_sources_and_selections(self):
        feedback = np.array([.2] * 6 + [.001], dtype=np.float32)
        target = np.array([-.7] * 6 + [.08], dtype=np.float32)
        series = {
            "state": source_series([1_000_000_000, 1_100_000_000],
                                   [np.zeros(7, dtype=np.float32), feedback]),
            "action": source_series([1_000_000_000, 1_100_000_000],
                                    [target, np.ones(7, dtype=np.float32)]),
        }
        frame = numeric_frame(series, {"state": np.array([1]), "action": np.array([0])}, 0)
        np.testing.assert_array_equal(frame["observation.state"], feedback)
        np.testing.assert_array_equal(frame["action"], target)

    def test_grid_stays_in_common_interval_and_all_sources_are_causal(self):
        series = self.streams()
        bounds = {"bag_start_ns": 990_000_000, "bag_end_ns": 1_500_000_000}
        grid, selected, summary = build_plan(series, bounds, self.config())
        np.testing.assert_array_equal(grid, [1_080_000_000, 1_180_000_000,
                                             1_280_000_000, 1_380_000_000])
        for key, source in series.items():
            self.assertTrue(np.all(source.stamps[selected[key]] <= grid))
            self.assertLessEqual(summary[key]["max_age_ms"], 150.)

    def test_internal_missing_interval_is_rejected_without_compressing_time(self):
        series = self.streams()
        series["action"] = source_series([1_050_000_000, 1_150_000_000, 1_450_000_000])
        bounds = {"bag_start_ns": 990_000_000, "bag_end_ns": 1_500_000_000}
        with self.assertRaisesRegex(ValueError, "action.*refusing to compress a gap"):
            build_plan(series, bounds, self.config())

    def test_nonoverlapping_streams_fail(self):
        series = self.streams()
        series["action"] = source_series([2_050_000_000, 2_150_000_000])
        bounds = {"bag_start_ns": 990_000_000, "bag_end_ns": 2_500_000_000}
        with self.assertRaisesRegex(ValueError, "no common time interval"):
            build_plan(series, bounds, self.config())


class ActionProvenanceTests(unittest.TestCase):
    def actions(self):
        return source_series(np.arange(5) * 100_000_000 + 1_000_000_000)

    def test_unverified_edges_are_trimmed_with_an_explicit_count(self):
        actions = self.actions()
        original = {key: getattr(actions, key).copy()
                    for key in ("stamps", "bag_stamps", "ordinals", "values")}
        metadata = {int(actions.stamps[i]): evidence_row(actions.values[i]) for i in (1, 2, 3)}
        report = verify_actions(actions, metadata, NAMES)
        self.assertEqual(report["unverified_prefix_actions"], 1)
        self.assertEqual(report["unverified_suffix_actions"], 1)
        self.assertEqual(report["verified_actions"], 3)
        for key, values in original.items():
            np.testing.assert_array_equal(getattr(actions, key), values[1:4])

    def test_internal_missing_provenance_cannot_be_trimmed_or_bridged(self):
        actions = self.actions()
        metadata = {int(actions.stamps[i]): evidence_row(actions.values[i]) for i in (0, 1, 3, 4)}
        with self.assertRaisesRegex(ValueError, "missing inside the episode"):
            verify_actions(actions, metadata, NAMES)

    def test_matching_timestamp_with_different_action_is_rejected(self):
        actions = self.actions()
        metadata = {int(stamp): evidence_row(value) for stamp, value in zip(actions.stamps, actions.values)}
        metadata[int(actions.stamps[2])]["action"][6] += .001
        with self.assertRaisesRegex(ValueError, "action and its metadata disagree"):
            verify_actions(actions, metadata, NAMES)

    def test_metadata_reordering_and_float_timestamp_roundoff_are_allowed(self):
        actions = self.actions()
        order = [6, 3, 1, 5, 0, 4, 2]
        metadata = {int(stamp) + 100: evidence_row(value[order], [NAMES[i] for i in order])
                    for stamp, value in zip(actions.stamps, actions.values)}
        report = verify_actions(actions, metadata, NAMES)
        self.assertEqual(report["verified_actions"], 5)
        self.assertEqual(report["unverified_prefix_actions"], 0)
        self.assertEqual(report["unverified_suffix_actions"], 0)

    def test_no_matching_evidence_fails(self):
        actions = self.actions()
        metadata = {9_000_000_000: evidence_row(np.zeros(7))}
        with self.assertRaisesRegex(ValueError, "No actions have matching"):
            verify_actions(actions, metadata, NAMES)


class MotorDiagnosticsTests(unittest.TestCase):
    def test_shuffled_motors_keep_canonical_order_units_and_individual_source_time(self):
        message = motor_diagnostics()
        message.status = [message.status[index] for index in (5, 2, 0, 4, 1, 3)]
        message.status.append(SimpleNamespace(name="right/follower/driver_feedback/1", values=[]))
        samples = motor_samples(message)
        self.assertEqual(MOTOR_KEYS, tuple("motor_%d" % i for i in range(1, 7)))
        self.assertEqual(len(samples), 6)
        for index, (stamp, values) in enumerate(samples, 1):
            self.assertEqual(stamp, 1_000_000_000 + index * 10_000_000)
            self.assertEqual(values.dtype, np.dtype("float32"))
            np.testing.assert_array_equal(values, np.array([index * .25, index * -.125], dtype=np.float32))

    def test_missing_duplicate_or_wrong_role_motor_entries_are_rejected(self):
        missing, duplicate, wrong_role = (motor_diagnostics() for _ in range(3))
        missing.status.pop()
        duplicate.status.append(copy.deepcopy(duplicate.status[0]))
        wrong_role.status[-1].name = "right/leader/motor_feedback/6"
        for message in (missing, duplicate, wrong_role):
            with self.subTest(names=[status.name for status in message.status]):
                with self.assertRaises(ValueError):
                    motor_samples(message)

    def test_required_diagnostic_keys_cannot_be_missing_or_duplicated(self):
        for key in ("current", "torque", "stamp", "source", "valid"):
            missing, duplicate = motor_diagnostics(), motor_diagnostics()
            missing.status[0].values = [field for field in missing.status[0].values if field.key != key]
            original = next(field for field in duplicate.status[0].values if field.key == key)
            duplicate.status[0].values.append(copy.deepcopy(original))
            for label, message in (("missing", missing), ("duplicate", duplicate)):
                with self.subTest(key=key, failure=label):
                    with self.assertRaises(ValueError):
                        motor_samples(message)

    def test_only_explicit_valid_measured_feedback_is_accepted(self):
        for key, value in (("source", "command"), ("source", "leader"),
                           ("valid", False), ("valid", 1), ("valid", "true")):
            message = motor_diagnostics()
            replace_diagnostic_field(message.status[2], key, value)
            with self.subTest(key=key, value=value):
                with self.assertRaises(ValueError):
                    motor_samples(message)

    def test_current_and_torque_require_finite_numeric_scalars(self):
        for key in ("current", "torque"):
            for value in (float("nan"), float("inf"), -float("inf"), True, "1.0", [1.], None):
                message = motor_diagnostics()
                replace_diagnostic_field(message.status[4], key, value)
                with self.subTest(key=key, value=value):
                    with self.assertRaises(ValueError):
                        motor_samples(message)

    def test_invalid_motor_time_is_not_replaced_with_aggregate_header(self):
        for stamp in (0, -1, float("nan"), float("inf"), True, "1.0", [1.], None):
            message = motor_diagnostics()
            replace_diagnostic_field(message.status[5], "stamp", stamp)
            with self.subTest(stamp=stamp):
                with self.assertRaises(ValueError):
                    motor_samples(message)

    def test_invalid_json_is_rejected(self):
        message = motor_diagnostics()
        message.status[0].values[0].value = "unquoted invalid JSON"
        with self.assertRaises(ValueError):
            motor_samples(message)

    def test_conflicting_feedback_at_the_same_motor_source_time_is_rejected(self):
        with self.assertRaises(ValueError):
            source_series([1_000_000_000, 1_000_000_000],
                          [np.array([.5, .1], dtype=np.float32), np.array([.5, .2], dtype=np.float32)])


class MotorFramePlanTests(unittest.TestCase):
    def config(self):
        cfg = FramePlanTests().config()
        cfg["joint_names"] = list(NAMES)
        cfg["motor_state"] = {"topic": "/piper/arms/right/follower/diagnostics"}
        return cfg

    def streams(self):
        series = FramePlanTests().streams()
        for index, key in enumerate(MOTOR_KEYS, 1):
            stamps = np.arange(5) * 100_000_000 + 1_000_000_000 + index * 7_000_000
            values = [np.array([index * .25 + sample, index * -.125 - sample], dtype=np.float32)
                      for sample in range(5)]
            series[key] = source_series(stamps, values)
        for key in ("cam_h", "cam_r"):
            series[key].shape = (480, 640, 3)
        return series

    def test_state_appends_all_currents_then_all_torques_and_action_stays_seven(self):
        series = self.streams()
        selected = {key: np.array([index % 2]) for index, key in enumerate(series)}
        frame = numeric_frame(series, selected, 0)
        position = series["state"].values[selected["state"][0]]
        currents = [series[key].values[selected[key][0]][0] for key in MOTOR_KEYS]
        torques = [series[key].values[selected[key][0]][1] for key in MOTOR_KEYS]
        expected = np.concatenate((position, np.asarray(currents, dtype=np.float32),
                                   np.asarray(torques, dtype=np.float32)))
        self.assertEqual(frame["observation.state"].shape, (19,))
        self.assertEqual(frame["observation.state"].dtype, np.dtype("float32"))
        np.testing.assert_array_equal(frame["observation.state"], expected)
        self.assertEqual(frame["action"].shape, (7,))
        np.testing.assert_array_equal(frame["action"], series["action"].values[selected["action"][0]])

    def test_feature_names_describe_the_nineteen_dimensional_state_order(self):
        features = feature_schema(self.config(), self.streams())
        expected = (NAMES + ["right_joint_%d_current" % i for i in range(1, 7)]
                    + ["right_joint_%d_torque" % i for i in range(1, 7)])
        self.assertEqual(features["observation.state"]["shape"], (19,))
        self.assertEqual(features["observation.state"]["names"], expected)
        self.assertEqual(features["action"]["shape"], (7,))
        self.assertEqual(features["action"]["names"], NAMES)

    def test_bulk_numeric_arrays_match_frame_values_without_transposing_motor_fields(self):
        series = self.streams()
        selected = {key: (np.arange(3) + index) % 5 for index, key in enumerate(series)}
        arrays = numeric_arrays(series, selected)
        self.assertEqual(arrays["observation.state"].shape, (3, 19))
        self.assertEqual(arrays["action"].shape, (3, 7))
        for frame_index in range(3):
            frame = numeric_frame(series, selected, frame_index)
            for name in ("observation.state", "action"):
                np.testing.assert_array_equal(arrays[name][frame_index], frame[name])
            for motor_index, key in enumerate(MOTOR_KEYS):
                sample = series[key].values[selected[key][frame_index]]
                self.assertEqual(arrays["observation.state"][frame_index, 7 + motor_index], sample[0])
                self.assertEqual(arrays["observation.state"][frame_index, 13 + motor_index], sample[1])

    def test_schema_rejects_motor_config_and_stream_contract_disagreement(self):
        cfg, series = self.config(), self.streams()
        without_motors = {key: value for key, value in series.items() if key not in MOTOR_KEYS}
        with self.assertRaises(ValueError):
            feature_schema(cfg, without_motors)
        cfg.pop("motor_state")
        with self.assertRaises(ValueError):
            feature_schema(cfg, series)

    def test_without_motor_option_legacy_state_action_and_schema_stay_seven(self):
        cfg = self.config()
        cfg.pop("motor_state")
        series = {key: value for key, value in self.streams().items() if key not in MOTOR_KEYS}
        features = feature_schema(cfg, series)
        for key in ("observation.state", "action"):
            self.assertEqual(features[key]["shape"], (7,))
            self.assertEqual(features[key]["names"], NAMES)
        selected = {key: np.array([0]) for key in series}
        frame = numeric_frame(series, selected, 0)
        self.assertEqual(frame["observation.state"].shape, (7,))
        self.assertEqual(frame["action"].shape, (7,))

    def test_partial_motor_set_is_rejected_instead_of_padded_or_ignored(self):
        for missing in (MOTOR_KEYS[0], MOTOR_KEYS[-1]):
            series = self.streams()
            del series[missing]
            with self.subTest(missing=missing):
                with self.assertRaises(ValueError):
                    numeric_frame(series, {key: np.array([0]) for key in series}, 0)

    def test_each_motor_uses_its_own_source_timestamp_on_unchanged_grid(self):
        series = self.streams()
        bounds = {"bag_start_ns": 990_000_000, "bag_end_ns": 1_500_000_000}
        grid, selected, summary = build_plan(series, bounds, self.config())
        np.testing.assert_array_equal(grid, [1_080_000_000, 1_180_000_000,
                                             1_280_000_000, 1_380_000_000])
        for index, key in enumerate(MOTOR_KEYS, 1):
            np.testing.assert_array_equal(selected[key], [0, 1, 2, 3])
            self.assertTrue(np.all(series[key].stamps[selected[key]] <= grid))
            self.assertAlmostEqual(summary[key]["max_age_ms"], 80. - index * 7.)

    def test_motor_without_past_feedback_cannot_be_filled_from_the_future(self):
        series = self.streams()
        series["motor_6"] = source_series([1_180_000_000, 1_280_000_000, 1_380_000_000],
                                          [np.array([1., .1], dtype=np.float32)] * 3)
        bounds = {"bag_start_ns": 990_000_000, "bag_end_ns": 1_500_000_000}
        with self.assertRaises(ValueError):
            build_plan(series, bounds, self.config())

    def test_stale_motor_feedback_inside_episode_is_rejected(self):
        series = self.streams()
        series["motor_6"] = source_series([1_042_000_000, 1_142_000_000, 1_442_000_000],
                                          [np.array([1., .1], dtype=np.float32)] * 3)
        bounds = {"bag_start_ns": 990_000_000, "bag_end_ns": 1_500_000_000}
        with self.assertRaisesRegex(ValueError, "motor_6.*refusing to compress a gap"):
            build_plan(series, bounds, self.config())


if __name__ == "__main__":
    unittest.main()
