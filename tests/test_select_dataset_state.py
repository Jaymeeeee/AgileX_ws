"""State selection preserves labels, time, normalization and Arrow metadata."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from piper_control.recording.select_state import (
    BASE_NAMES, SOURCE_NAMES, derive, select_info, select_stats, select_table,
)


class MetadataTests(unittest.TestCase):
    def test_info_changes_only_the_state_contract(self):
        info = {"codebase_version": "v3.0", "total_frames": 22, "features": {
            "observation.state": {"shape": [19], "names": SOURCE_NAMES, "dtype": "float32"},
            "action": {"shape": [7], "names": BASE_NAMES, "dtype": "float32"}}}
        result = select_info(info)
        self.assertEqual(result["features"]["observation.state"]["shape"], [13])
        self.assertEqual(result["features"]["observation.state"]["names"], SOURCE_NAMES[:13])
        self.assertEqual(result["features"]["action"], info["features"]["action"])
        self.assertEqual(info["features"]["observation.state"]["shape"], [19])
        self.assertEqual(result["total_frames"], 22)

    def test_all_state_stats_are_sliced_but_counts_and_other_features_are_retained(self):
        stats = {"observation.state": {"mean": list(range(19)), "std": list(range(19)),
                                        "q99": list(range(19)), "count": [100]},
                 "action": {"mean": list(range(7)), "count": [100]}}
        result = select_stats(stats)
        for key in ("mean", "std", "q99"):
            self.assertEqual(result["observation.state"][key], list(range(13)))
        self.assertEqual(result["observation.state"]["count"], [100])
        self.assertEqual(result["action"], stats["action"])
        self.assertEqual(len(stats["observation.state"]["mean"]), 19)

    def test_bad_source_contract_and_truncated_statistics_are_rejected(self):
        with self.assertRaises(ValueError):
            select_info({"codebase_version": "v3.0", "features": {
                "observation.state": {"shape": [19], "names": list(reversed(SOURCE_NAMES)), "dtype": "float32"}}})
        with self.assertRaises(ValueError):
            select_stats({"observation.state": {"mean": [0.] * 13}})


@unittest.skipUnless(importlib.util.find_spec("pyarrow"), "LeRobot environment provides PyArrow")
class ParquetTests(unittest.TestCase):
    def test_fixed_state_list_and_huggingface_schema_change_together(self):
        import pyarrow as pa
        table = pa.table({"observation.state": pa.array([list(range(19)), list(range(100, 119))],
                                                       type=pa.list_(pa.float32(), 19)),
                          "action": pa.array([[5.] * 7, [6.] * 7], type=pa.list_(pa.float32(), 7)),
                          "timestamp": [0., .1], "episode_index": [0, 0]})
        metadata = {"info": {"features": {"observation.state": {"length": 19, "_type": "List"}}},
                    "fingerprint": "old"}
        table = table.replace_schema_metadata({b"huggingface": json.dumps(metadata).encode()})
        result = select_table(table)
        self.assertEqual(result["observation.state"].to_pylist(), [list(range(13)), list(range(100, 113))])
        self.assertEqual(result.schema.field("observation.state").type.list_size, 13)
        hf = json.loads(result.schema.metadata[b"huggingface"])
        self.assertEqual(hf["info"]["features"]["observation.state"]["length"], 13)
        self.assertNotEqual(hf["fingerprint"], "old")
        for name in ("action", "timestamp", "episode_index"):
            self.assertTrue(result[name].equals(table[name]))

    def test_episode_stats_slice_each_row_and_keep_count_and_video_offsets(self):
        import pyarrow as pa
        table = pa.table({"stats/observation.state/mean": [list(range(19)), list(range(30, 49))],
                          "stats/observation.state/count": [[10], [20]],
                          "videos/cam/from_timestamp": [0., 1.], "episode_index": [0, 1]})
        result = select_table(table, episode_metadata=True)
        self.assertEqual(result["stats/observation.state/mean"].to_pylist(),
                         [list(range(13)), list(range(30, 43))])
        for name in ("stats/observation.state/count", "videos/cam/from_timestamp", "episode_index"):
            self.assertTrue(result[name].equals(table[name]))

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "existing"
            output.mkdir()
            marker = output / "keep.txt"
            marker.write_text("original")
            with self.assertRaisesRegex(ValueError, "refusing to overwrite"):
                derive(root / "missing_source", output)
            self.assertEqual(marker.read_text(), "original")


if __name__ == "__main__":
    unittest.main()
