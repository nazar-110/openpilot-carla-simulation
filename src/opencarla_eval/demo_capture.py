"""Optional read-only recording of real camera frames and their model messages."""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path

from PIL import Image


class DemoCapture:
  def __init__(self, directory: Path):
    self.directory = directory
    directory.mkdir(parents=True, exist_ok=True)
    self.frames = {}
    self.last_model_frame = -1
    self.dropped = 0
    self.error = None
    self.queue = queue.Queue(maxsize=32)
    self.worker = threading.Thread(target=self._write, daemon=True)
    self.worker.start()

  def camera(self, frame_id, carla_frame, sim_time, rgb, chase):
    self.frames[frame_id] = (carla_frame, sim_time, rgb[::2, ::2].copy(), chase)
    for old in sorted(self.frames)[:-32]:
      del self.frames[old]

  def state(self, state):
    frame_id = state["model"]["frame_id"]
    if frame_id <= self.last_model_frame or frame_id not in self.frames:
      return
    if state["runtime"]["phase"] != "evaluating" or not state["model"]["valid"]:
      return
    self.last_model_frame = frame_id
    try:
      self.queue.put_nowait((frame_id, self.frames[frame_id], state))
    except queue.Full:
      self.dropped += 1

  def _write(self):
    try:
      with (self.directory / "frames.jsonl").open("w", encoding="utf-8") as manifest:
        while True:
          item = self.queue.get()
          if item is None:
            break
          frame_id, (carla_frame, sim_time, road, chase), state = item
          Image.fromarray(road).save(self.directory / f"{frame_id:06d}_road.jpg", quality=90)
          if chase is not None:
            Image.fromarray(chase).save(self.directory / f"{frame_id:06d}_chase.jpg", quality=90)
          manifest.write(
            json.dumps(
              dict(
                frame_id=frame_id,
                carla_frame=carla_frame,
                sim_time_s=sim_time,
                chase_available=chase is not None,
                state=state,
              )
            )
            + "\n"
          )
    except Exception as exc:
      self.error = str(exc)

  def close(self):
    while self.worker.is_alive():
      try:
        self.queue.put(None, timeout=0.2)
        break
      except queue.Full:
        continue
    self.worker.join(timeout=10)
    (self.directory / "capture.json").write_text(
      json.dumps(
        dict(
          source="real CARLA sensor images, paired to exact modelV2.frameId",
          dropped=self.dropped,
          error=self.error,
          worker_finished=not self.worker.is_alive(),
        ),
        indent=2,
      )
    )
