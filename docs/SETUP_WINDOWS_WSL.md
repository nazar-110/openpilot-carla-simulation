# Windows + WSL2 setup

This is the recommended layout for a Windows workstation with an NVIDIA GPU:

```text
Windows 11                         Ubuntu 24.04 in WSL2
-------------------------------    ----------------------------------
CARLA 0.9.16 server :2000   <----  CARLA Python client
GPU rendering                       OpenPilot v0.11.1 manager/modeld
                                    opencarla-eval bridge and analysis
```

Keeping OpenPilot in the WSL Linux filesystem is important for build and I/O
performance. The project may remain under `/mnt/c`, though copying it into WSL
will also improve repeated JSONL writes.

## 1. Install WSL2

From an elevated PowerShell terminal:

```powershell
wsl --install -d Ubuntu-24.04
wsl --update
```

Reboot if Windows requests it. Open Ubuntu once and complete the user setup.
OpenPilot's official tools documentation recommends Ubuntu 24.04 and identifies
WSL2 as the Windows path: [tools/README.md](https://github.com/commaai/openpilot/blob/v0.11.1/tools/README.md).

## 2. Install and launch CARLA 0.9.16 on Windows

Download the Windows 0.9.16 package from the official
[CARLA releases](https://github.com/carla-simulator/carla/releases/tag/0.9.16)
and extract it, for example to `C:\CARLA_0.9.16`.

Launch with rendering enabled and Epic quality:

```powershell
Set-Location "C:\path\to\Evaluation of OpenPilot in CARLA Using Realistic Urban Driving Scenarios"
.\scripts\start_carla.ps1 -CarlaRoot C:\CARLA_0.9.16 -ShowWindow
```

The `-ShowWindow` switch lets you watch the simulation. Without it, this
low-level launcher uses
`-RenderOffScreen`; off-screen rendering still produces camera frames, while
CARLA's `no_rendering_mode` does not. The combined launcher introduced below
always starts a visible window. Epic quality preserves postprocessing, shadows,
and draw distance needed for perception/weather comparisons.

CARLA's client API exposes `no_rendering_mode` (which the harness rejects) but
does not report the command-line quality preset. Epic quality therefore remains
a host launch/preflight requirement and the exact launch command belongs in the
operator environment manifest.

Allow CARLA's TCP RPC and streaming ports 2000-2002 through Windows Firewall
if prompted. Traffic Manager port 8000 is created on the WSL client in this
layout and normally does not need a Windows inbound rule.

## 3. Build the pinned OpenPilot checkout in WSL

From Ubuntu:

```bash
sudo apt update
sudo apt install -y git git-lfs
git lfs install

git clone https://github.com/commaai/openpilot.git ~/openpilot
cd ~/openpilot
git checkout --detach 4df40d2c1946a57242230186edd073c4073060a6
git submodule update --init --recursive
git lfs pull

tools/op.sh setup
source .venv/bin/activate
scons -u -j"$(nproc)"
```

The repository also provides an equivalent helper:

```bash
bash /mnt/c/path/to/project/scripts/setup_openpilot_wsl.sh "$HOME/openpilot"
```

The build and Git LFS steps are mandatory. Missing generated cereal modules or
model blobs commonly appear later as misleading messaging/model startup errors.
The helper is idempotent for an existing checkout at the tested commit. It also
applies this project's pinned discrete-GPU modeld compatibility patch and writes
the CUDA device configuration required by this WSL installation.

## 4. Install the matching CARLA client and this project

With OpenPilot's `.venv` active:

```bash
uv pip install --python "$VIRTUAL_ENV/bin/python" carla==0.9.16
uv pip install --python "$VIRTUAL_ENV/bin/python" -e "/mnt/c/path/to/project[analysis]"
```

Confirm Python and imports:

```bash
python --version
python -c 'from importlib.metadata import version; print(version("carla"))'
python -c 'from openpilot.tools.sim.bridge.common import SimulatorBridge; print("openpilot sim OK")'
```

OpenPilot v0.11.1 requires Python `>=3.12.3,<3.13`. The custom runtime avoids
ScenarioRunner's older NumPy/Python constraints; do not install ScenarioRunner
into this environment.

Before confirmatory collection, put this harness in a clean Git commit:

```bash
cd /mnt/c/path/to/project
git status --short  # must print nothing after committing the frozen harness
```

Research-valid real artifacts require both the OpenPilot and harness checkouts
to be clean and versioned. The currently required compatibility patch makes the
upstream OpenPilot checkout dirty, so pilot artifacts intentionally fail this
gate. Freeze that patch in a versioned derived checkout and update the tested
provenance policy before confirmatory collection. Development and pilot work can
continue with the explicit exploratory override.

## 5. Configure networking

Recent WSL installations with mirrored networking can usually reach the Windows
CARLA server at `127.0.0.1`. Test:

```bash
export OPENPILOT_ROOT="$HOME/openpilot"
export CARLA_HOST=127.0.0.1
export CARLA_PORT=2000
opencarla-eval doctor --openpilot-root "$OPENPILOT_ROOT" --connect
```

If localhost forwarding is unavailable, find the Windows WSL adapter address
from PowerShell:

```powershell
Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object InterfaceAlias -Like "*WSL*" |
  Select-Object InterfaceAlias, IPAddress
```

Set `CARLA_HOST` to the gateway address, then rerun the doctor. Environment
variables override `config/carla.yaml` without modifying the frozen file.
The visible launcher below performs this detection and connectivity probe
automatically, so the `ip` command is not required inside WSL.

A healthy doctor output has:

```json
{
  "python_supported": true,
  "carla_import": true,
  "carla_client_version": "0.9.16",
  "carla_server_version": "0.9.16",
  "carla_version_match": true,
  "openpilot_commit": "4df40d2c1946a57242230186edd073c4073060a6",
  "openpilot_sim_import": true,
  "toolkit_worktree_dirty": false
}
```

## 6. Watch the verified visible pilot

The current machine is configured for a one-command visible run. In a normal
Windows PowerShell terminal, from the project directory, use:

```powershell
.\scripts\run_visible_openpilot.ps1

# Optional: choose the preferred dashboard port.
.\scripts\run_visible_openpilot.ps1 -DashboardPort 8765

# Optional: serve the dashboard without opening a browser automatically.
.\scripts\run_visible_openpilot.ps1 -NoDashboardWindow
```

The script reuses a running CARLA server or starts one visibly at Epic quality,
checks WSL connectivity and the OpenPilot compatibility patch, runs the known
40-second lane-following pilot, and leaves CARLA open. It uses `--overwrite` for
the fixed pilot run ID so a prior attempt is archived and the car actually
drives again. Its explicit untested-checkout override means the summary remains
`valid_for_research: false` until the compatibility work is frozen as described
above.

The CARLA window uses the server-owned spectator as a smoothed third-person
chase camera. Its default position is 7 m behind and 3 m above the ego vehicle
with -15 degrees of pitch; change `carla.spectator` in `config/carla.yaml` to
tune it. Moving this observer neither ticks the world nor changes ego state or
any input received by OpenPilot.

A separate local browser window displays read-only OpenPilot diagnostics: the
synchronized road/wide camera buffers, an OpenPilot-green learned path projected
onto the road image, the separate bird's-eye path and lane geometry, leads,
longitudinal plan, engagement and alerts, final control decisions, and timing.
The JPEG previews are downsampled and frame-decimated only for display; OpenPilot
still receives the full-resolution camera stream at the configured 20 Hz. This
harness does not feed OpenPilot a destination/map navigation route, so both path
views are explicitly camera-derived predictions rather than a navigation route.

For a manual WSL launch, enable the same views with:

```bash
export OPENCARLA_SPECTATOR=1
export OPENCARLA_DASHBOARD=1
export OPENCARLA_DASHBOARD_PORT=8765
```

## 7. Run evaluator-owned OpenPilot trials

Keep the CARLA server running, but do **not** launch OpenPilot's manager
manually. For every OpenPilot trial, the evaluator invokes the pinned
`tools/sim/launch_openpilot.sh` in a fresh process group, captures its output in
that run's `openpilot_manager.log`, and stops it after the bridge exits. Restarting
manager/modeld for every logical run prevents model/control state and the
one-connection VisionIPC client from carrying over to the next run. A
pre-existing manager from the same OpenPilot checkout is rejected.

In one WSL terminal:

```bash
cd /mnt/c/path/to/project
source ~/openpilot/.venv/bin/activate
export OPENPILOT_ROOT="$HOME/openpilot"

opencarla-eval validate --experiment experiments/urban_suite.yaml

# Two-trial lifecycle smoke check: use empty pilot output paths.
opencarla-eval run \
  --experiment experiments/urban_suite.yaml \
  --backend carla \
  --controller openpilot \
  --limit 2 \
  --keep-going
```

Both executed run directories must contain a distinct `openpilot_manager.log`.
Inspect those logs for model startup errors, then exclude these pilot trials
before confirmatory collection. If either run already has a reusable completed
artifact, the command will keep it instead of exercising a new manager lifecycle;
use clean pilot outputs, not `--overwrite` on confirmatory data.

The bridge automatically engages after OpenPilot reports that it is engageable.
Evaluation starts only after five continuous active seconds, followed by the
common continuous two-tick neutral transition gate described in
[Architecture](ARCHITECTURE.md).

## 8. Baseline and full collection

The Traffic Manager reference does not require the OpenPilot manager, but it may
use the same Python environment:

```bash
opencarla-eval run \
  --experiment experiments/urban_suite.yaml \
  --backend carla \
  --controller traffic_manager \
  --limit 1
```

After pilot acceptance and configuration freeze:

The runner and `doctor` record/check the toolkit commit. Capture the host
environment and model blobs separately before starting the suite:

```bash
mkdir -p reports/environment
git -C "$OPENPILOT_ROOT" lfs ls-files -l > reports/environment/openpilot_lfs_objects.txt
python --version > reports/environment/host_manifest.txt
uname -a >> reports/environment/host_manifest.txt
nvidia-smi >> reports/environment/host_manifest.txt
```

From Windows, append `wsl --version`, the NVIDIA driver version, and the exact
CARLA launch command to that manifest. The run command also writes
`collection_manifest.json` with the ordered run IDs, order seed, Python/platform
string, experiment hash, and toolkit state.

```bash
opencarla-eval run \
  --experiment experiments/urban_suite.yaml \
  --backend carla \
  --randomize-order \
  --order-seed 20260714 \
  --keep-going
```

The command is resumable, but mere `summary.json` existence is not sufficient.
The runner reuses a completed artifact only when its logical identity, raw
artifact set, scenario/condition/experiment hashes, effective runtime
configuration, recorded software state, and valid stopped-recorder audit match
the requested trial; OpenPilot reuse also requires its
`openpilot_manager.log`. The summary must not be marked invalid or
`rerun_same_seed`. A mismatch or invalid attempt fails closed. Correct the
fault, then recollect through an explicit `--overwrite` so the old attempt is
archived instead of silently replaced.

Rerun one invalid artifact explicitly by its existing ID. `--overwrite`
archives the prior attempt before writing the replacement:

```bash
opencarla-openpilot \
  --experiment experiments/urban_suite.yaml \
  --run-id lane_following__clear_noon__openpilot__r01__s41000 \
  --overwrite

# Or use opencarla-baseline for a Traffic Manager run ID.
```

### CARLA recorder retention

Every real trial starts a server-side CARLA recorder and stops it before summary
finalization. Its collision records are an integrity source even when replay
retention is disabled:

- `carla.recording.carla_recorder: false` uses the fixed scratch filename
  `opencarla_eval_integrity.rec`; the next trial overwrites it.
- `carla.recording.carla_recorder: true` uses the unique filename
  `opencarla_eval_<run-id>.rec` so the replay remains available.

These files are written in CARLA's **server-side** default saved directory, not
under the WSL result directory. For the Windows package, CARLA 0.9.16 documents
`C:\Users\<Windows-user>\AppData\Local\CarlaUE4\Saved`; a source build uses
`<CARLA_ROOT>\Unreal\CarlaUE4\Saved`. The server filename and collision-audit
status are recorded in `summary.json`. Archive retained files separately when
replays are part of the study record.

## Troubleshooting

### CARLA version mismatch

The runner intentionally refuses mixed client/server versions. Install the
matching 0.9.16 wheel; do not suppress this check.

### Camera timeout

- Confirm CARLA was not launched with `no_rendering_mode`.
- Use Epic quality for the preregistered study.
- Confirm only the evaluation bridge ticks the world.
- Check GPU memory and Windows Task Manager.
- Do not run ScenarioRunner with `--sync` beside this bridge.

### OpenPilot model or cereal import fails

Rerun `git lfs pull`, `git submodule update --init --recursive`, activate the
OpenPilot venv, then run `scons -u -j"$(nproc)"` from its root.

### OpenPilot commit rejected

Check out the pinned v0.11.1 commit. The environment override for an untested
commit is intentionally explicit and should be used only for a separately
versioned exploratory study.

### Pre-existing OpenPilot manager rejected

Stop any manually launched `tools/sim/launch_openpilot.sh` or `manager.py`
process for the selected checkout, then rerun the evaluator. This refusal is
intentional: sharing manager/modeld across trials would retain controller state
and leave modeld attached to the preceding trial's VisionIPC server.

### CARLA remains frozen after a crash

The harness normally restores asynchronous mode. If its process was killed by
the OS, restart the CARLA server before rerunning; this also guarantees a clean
server state.

### Dashboard window does not appear

Open the `Live OpenPilot decisions` URL printed by the visible launcher. The
selected port may be higher than 8765 if another local process already uses that
port. `-NoDashboardWindow` suppresses only automatic browser opening; it does not
disable the local dashboard server.
