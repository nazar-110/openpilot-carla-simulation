"""Render recorded CARLA pixels and exact paired model paths into a real demo MP4."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("capture", type=Path)
  parser.add_argument("--output", type=Path, default=Path("media/openpilot-carla-demo.mp4"))
  args = parser.parse_args()
  summary = json.loads((args.capture.parent / "summary.json").read_text())
  rows = [json.loads(line) for line in (args.capture / "frames.jsonl").read_text().splitlines()]
  if len(rows) < 2:
    raise RuntimeError("No real model-paired recording to render")
  args.output.parent.mkdir(parents=True, exist_ok=True)
  rendered = args.capture / "rendered"
  rendered.mkdir(exist_ok=True)
  font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 24)
  title = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 36)
  small = ImageFont.truetype("C:/Windows/Fonts/consola.ttf", 19)
  playlist = []
  for index, row in enumerate(rows):
    state = row["state"]
    model, runtime, control = state["model"], state["runtime"], state["control"]
    frame_id = row["frame_id"]
    canvas = Image.new("RGB", (1600, 1000), "#0b1320")
    draw = ImageDraw.Draw(canvas)
    draw.text((30, 22), "OPENPILOT x CARLA", font=title, fill="#f1f5fa")
    draw.text(
      (30, 70),
      "Real simulator recording  /  Calibrated vehicle actuation",
      font=font,
      fill="#aabed0",
    )
    road = Image.open(args.capture / f"{frame_id:06d}_road.jpg").convert("RGBA").resize((960, 602))
    overlay = Image.new("RGBA", road.size)
    polygon = model["trajectory_overlay"]["polygon_px"]
    if polygon:
      points = [(x * 960 / 1928, y * 602 / 1208) for x, y in polygon]
      ImageDraw.Draw(overlay).polygon(points, fill=(40, 225, 115, 105))
    road = Image.alpha_composite(road, overlay)
    canvas.paste(road.convert("RGB"), (30, 130))
    draw.text(
      (45, 142),
      "ROAD CAMERA + MODEL PREDICTED PATH",
      font=small,
      fill="white",
      stroke_width=1,
      stroke_fill="black",
    )
    if row["chase_available"]:
      chase = Image.open(args.capture / f"{frame_id:06d}_chase.jpg").resize((520, 293))
      canvas.paste(chase, (1040, 130))
      draw.text(
        (1050, 142),
        "EGO CHASE CAMERA",
        font=small,
        fill="white",
        stroke_width=1,
        stroke_fill="black",
      )
    draw.text((1040, 455), "LIVE DECISIONS", font=title, fill="#63e5a4")

    def number(value, unit):
      return f"{value:.2f} {unit}" if isinstance(value, (int, float)) else "unavailable"

    fields = [
      ("Speed", number(runtime.get("speed_mps"), "m/s")),
      ("Acceleration request", number(control.get("command_acceleration_mps2"), "m/s2")),
      ("Steering request", number(control.get("command_steering_angle_deg"), "deg")),
      ("Planner", state["planning"]["source"]),
      ("Control", state["selfdrive"]["state"]),
    ]
    actuation = runtime.get("actuation") or {}
    fields += [
      ("Throttle / brake", f"{actuation.get('throttle', 0):.3f} / {actuation.get('brake', 0):.3f}"),
      ("Measured acceleration", number(actuation.get("measured_acceleration_mps2"), "m/s2")),
    ]
    for n, (label, value) in enumerate(fields):
      draw.text((1040, 515 + n * 51), f"{label}: {value}", font=small, fill="#d4e3ef")
    draw.text(
      (30, 760),
      f"Simulation {row['sim_time_s']:05.2f} s  |  CARLA frame {row['carla_frame']}  |  Model/input frame {frame_id}",
      font=font,
      fill="#d4e3ef",
    )
    draw.text(
      (30, 806),
      "Green ribbon: OpenPilot model prediction, not a ground-truth route or safety guarantee.",
      font=font,
      fill="#86cfa8",
    )
    draw.text(
      (30, 861),
      "Dry-road lane-following demonstration. OpenPilot is driver assistance, not autonomous urban driving.",
      font=font,
      fill="#aabed0",
    )
    draw.text(
      (30, 913),
      f"Quality: {'PASS' if summary['metrics']['quality_pass'] else 'FAIL'}  |  "
      f"Collisions: {summary['metrics']['safety']['collision_count']}  |  "
      f"Lane invasions: {summary['metrics']['safety']['lane_invasion_count']}  |  "
      f"Off-road: {summary['metrics']['safety']['offroad_duration_s']:.2f} s  |  Prototype trial",
      font=font,
      fill="#aabed0",
    )
    target = rendered / f"{index:06d}.jpg"
    canvas.save(target, quality=92)
    if index == min(20, len(rows) - 1):
      canvas.save(args.output.with_suffix(".jpg"), quality=92)
    duration = (
      max(0.001, rows[index + 1]["sim_time_s"] - row["sim_time_s"])
      if index + 1 < len(rows)
      else 0.1
    )
    playlist.extend([f"file '{target.name}'", f"duration {duration:.6f}"])
  playlist.append(f"file '{len(rows) - 1:06d}.jpg'")
  (rendered / "frames.txt").write_text("\n".join(playlist) + "\n")
  subprocess.run(
    [
      "ffmpeg",
      "-y",
      "-hide_banner",
      "-loglevel",
      "warning",
      "-f",
      "concat",
      "-safe",
      "0",
      "-i",
      str(rendered / "frames.txt"),
      "-vf",
      "fps=20",
      "-c:v",
      "libx264",
      "-crf",
      "21",
      "-pix_fmt",
      "yuv420p",
      "-movflags",
      "+faststart",
      str(args.output),
    ],
    check=True,
  )
  print(f"Rendered {len(rows)} real model-paired frames to {args.output}")


if __name__ == "__main__":
  main()
