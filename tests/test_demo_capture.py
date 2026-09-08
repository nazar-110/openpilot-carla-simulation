import json

import numpy as np

from opencarla_eval.demo_capture import DemoCapture


def test_capture_pairs_exact_model_input_and_rejects_missing_frame(tmp_path):
  recorder = DemoCapture(tmp_path)
  pixels = np.zeros((8, 12, 3), dtype=np.uint8)
  recorder.camera(10, 100, 1.0, pixels, pixels)
  recorder.state({"model": {"frame_id": 9, "valid": True}, "runtime": {"phase": "evaluating"}})
  recorder.state({"model": {"frame_id": 10, "valid": True}, "runtime": {"phase": "evaluating"}})
  recorder.state({"model": {"frame_id": 10, "valid": True}, "runtime": {"phase": "evaluating"}})
  recorder.close()
  rows = [json.loads(line) for line in (tmp_path / "frames.jsonl").read_text().splitlines()]
  assert len(rows) == 1
  assert rows[0]["frame_id"] == 10
  assert rows[0]["carla_frame"] == 100
  assert (tmp_path / "000010_road.jpg").is_file()
  assert json.loads((tmp_path / "capture.json").read_text())["error"] is None


def test_warmup_is_not_presented_as_evaluation(tmp_path):
  recorder = DemoCapture(tmp_path)
  recorder.camera(10, 100, 0.0, np.zeros((8, 12, 3), dtype=np.uint8), None)
  recorder.state({"model": {"frame_id": 10, "valid": True}, "runtime": {"phase": "warmup"}})
  recorder.close()
  assert not (tmp_path / "frames.jsonl").read_text()
