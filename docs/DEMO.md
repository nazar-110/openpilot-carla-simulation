# Real video demo

The README includes a GitHub-hosted inline video player. Its 1280 × 800 copy is
compressed for quick playback; the original 1600 × 1000 MP4 remains in `media/`
and is linked below the player. Both contain the same complete recorded trial.

The committed MP4 contains actual CARLA 0.9.16 road and chase camera pixels
captured during an OpenPilot-driven trial, not a Traffic Manager replay.
The road camera is paired with the exact `modelV2.frameId` before the green
predicted-path ribbon is rendered. The chase image uses the same CARLA frame.
Model predictions are not ground-truth routes or navigation guarantees.

The recording covers a dry-noon lane-following pilot, seed 41000. It reaches
100% route completion, but is explicitly a failed driving-quality trial:
3 lane invasions, 1.75 seconds off-road, no collisions. Its modified OpenPilot
checkout makes it ineligible as a certified research result. These caveats are
shown in the video and preserved in `media/demo-summary.json`.

## Record another trial

From the project directory in Windows PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visible_openpilot.ps1 -RecordDemo
```

The normal live dashboard must remain enabled for model-message capture.
Recording is optional and has a bounded writer queue. A capture status file
reports any writer errors or dropped records. Camera/model frame pairing is
tested independently. Warmup frames are not presented as evaluated driving.

Then render the actual saved frames (requires FFmpeg on PATH and Windows fonts):

```powershell
python scripts/render_demo.py results/urban_suite/lane_following__clear_noon__openpilot__r01__s41000/demo_capture
```

The renderer preserves recorded simulation-time spacing at 20 fps. Frames may
be held between recorded model updates; it does not synthesize intermediate
driving frames. Rendering is offline and does not feed anything back into
OpenPilot. The original raw camera capture remains in the ignored results
directory, while the compact MP4, poster, summary and telemetry are published.
