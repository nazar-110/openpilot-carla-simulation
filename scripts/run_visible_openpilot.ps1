[CmdletBinding()]
param(
  [string]$CarlaRoot = "D:\CARLA\0.9.16",
  [string]$Distro = "Ubuntu-24.04-ROS2",
  [string]$OpenPilotRoot = "",
  [ValidateRange(1, 65533)]
  [int]$Port = 2000,
  [string]$Experiment = "experiments/urban_suite.yaml",
  [string]$RunId = "lane_following__clear_noon__openpilot__r01__s41000",
  [ValidateRange(1, 3600)]
  [int]$CarlaStartupTimeoutSeconds = 120,
  [ValidateRange(1, 65501)]
  [int]$DashboardPort = 8765,
  [switch]$NoDashboardWindow,
  [switch]$RecordDemo
)

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
if (-not (Test-Path -LiteralPath $CarlaRoot -PathType Container)) {
  throw "CARLA 0.9.16 root directory not found: $CarlaRoot"
}
$ResolvedCarlaRoot = (Resolve-Path -LiteralPath $CarlaRoot).Path
$CarlaExecutable = Join-Path $ResolvedCarlaRoot "CarlaUE4.exe"
if (-not (Test-Path -LiteralPath $CarlaExecutable -PathType Leaf)) {
  throw "CARLA 0.9.16 executable not found: $CarlaExecutable"
}

$DistroNames = @(& wsl.exe --list --quiet) |
  ForEach-Object { ($_ -replace "`0", "").Trim() } |
  Where-Object { $_ }
if ($DistroNames -notcontains $Distro) {
  throw "WSL distribution '$Distro' was not found. Available: $($DistroNames -join ', ')"
}
if (-not $OpenPilotRoot) {
  $WslHome = (& wsl.exe -d $Distro -- sh -c 'printenv HOME' | Select-Object -First 1).Trim()
  if (-not $WslHome.StartsWith('/')) { throw 'Could not find the WSL home directory.' }
  $OpenPilotRoot = "$WslHome/openpilot"
}

$Listener = @(Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue)
if (-not $Listener) {
  Write-Host "Starting visible CARLA 0.9.16 at Epic quality..."
  Start-Process `
    -FilePath $CarlaExecutable `
    -WorkingDirectory (Split-Path -Parent $CarlaExecutable) `
    -ArgumentList @("-nosound", "-quality-level=Epic", "-carla-rpc-port=$Port")

  $Deadline = (Get-Date).AddSeconds($CarlaStartupTimeoutSeconds)
  do {
    Start-Sleep -Seconds 1
    $Listener = @(Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue)
  } until ($Listener -or (Get-Date) -ge $Deadline)
  if (-not $Listener) {
    throw "CARLA did not start listening on TCP port $Port within $CarlaStartupTimeoutSeconds seconds."
  }
} else {
  Write-Host "Using the CARLA server already listening on TCP port $Port."
}

$ListenerOwners = @($Listener | Select-Object -ExpandProperty OwningProcess -Unique)
$CarlaServerProcesses = @(
  foreach ($OwnerPid in $ListenerOwners) {
    Get-CimInstance Win32_Process -Filter "ProcessId=$OwnerPid" -ErrorAction SilentlyContinue |
      Where-Object {
        $_.Name -like "CarlaUE4*" -and
        $_.ExecutablePath -and
        $_.ExecutablePath.StartsWith($ResolvedCarlaRoot, [System.StringComparison]::OrdinalIgnoreCase)
      }
  }
)
if (-not $CarlaServerProcesses) {
  throw "TCP port $Port is not owned by CARLA under '$ResolvedCarlaRoot'. Stop the conflicting listener or choose the correct -Port/-CarlaRoot."
}
$UnsupportedServer = $CarlaServerProcesses | Where-Object {
  $_.CommandLine -match "(?i)-RenderOffScreen" -or $_.CommandLine -match "(?i)-quality-level=Low"
} | Select-Object -First 1
if ($UnsupportedServer) {
  throw "The CARLA server on port $Port is off-screen or Low quality. Restart it visibly at Epic quality."
}

$WslProjectRootRaw = (& wsl.exe -d $Distro -- wslpath -a -u $ProjectRoot) | Select-Object -First 1
if ($LASTEXITCODE -ne 0 -or -not $WslProjectRootRaw) {
  throw "Could not translate the project path into the '$Distro' filesystem."
}
$WslProjectRoot = $WslProjectRootRaw.Trim()

$Evaluator = "$OpenPilotRoot/.venv/bin/opencarla-openpilot"
$DashboardPython = "$OpenPilotRoot/.venv/bin/python"
$RuntimeProbe = "test -x '$Evaluator' && test -x '$DashboardPython'"
& wsl.exe -d $Distro -- bash -lc $RuntimeProbe
if ($LASTEXITCODE -ne 0) {
  throw "The OpenPilot evaluator or venv Python is missing. Reinstall this project into $OpenPilotRoot/.venv, then rerun."
}

$EvaluatorProbe = "pgrep -af '[o]pencarla-openpilot' || true"
$RunningEvaluators = @(& wsl.exe -d $Distro -- bash -lc $EvaluatorProbe) |
  Where-Object { $_ }
if ($RunningEvaluators) {
  throw "An OpenCarla evaluator is already running in WSL: $($RunningEvaluators -join '; ')"
}

& wsl.exe -d $Distro -- bash `
  "$WslProjectRoot/scripts/apply_openpilot_wsl_compat.sh" `
  $OpenPilotRoot `
  --check
if ($LASTEXITCODE -ne 0) {
  throw "OpenPilot compatibility preflight failed. Follow the command printed above, then rerun this launcher."
}

function Test-CARLAFromWSL {
  param([string]$Candidate)

  $Probe = "timeout 3 bash -c '</dev/tcp/$Candidate/$Port'"
  $PreviousErrorActionPreference = $ErrorActionPreference
  try {
    # A refused candidate is expected while falling back from mirrored
    # localhost networking to the WSL virtual adapter.
    $ErrorActionPreference = "Continue"
    & wsl.exe -d $Distro -- bash -lc $Probe *> $null
    $ProbeExitCode = $LASTEXITCODE
  } finally {
    $ErrorActionPreference = $PreviousErrorActionPreference
  }
  return $ProbeExitCode -eq 0
}

$HostCandidates = @("127.0.0.1")
$HostCandidates += @(
  Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.InterfaceAlias -like "*WSL*" } |
    Select-Object -ExpandProperty IPAddress
)
$CarlaHost = $null
foreach ($Candidate in $HostCandidates | Select-Object -Unique) {
  if (Test-CARLAFromWSL -Candidate $Candidate) {
    $CarlaHost = $Candidate
    break
  }
}
if (-not $CarlaHost) {
  throw "WSL cannot reach CARLA on port $Port. Reapply scripts\configure_carla_firewall.ps1 -BasePort $Port from elevated PowerShell."
}

$DashboardPortProbe = "import socket,sys; s=socket.socket(); s.bind(('127.0.0.1', int(sys.argv[1]))); s.close()"
$SelectedDashboardPort = $null
foreach ($CandidatePort in $DashboardPort..([Math]::Min(65535, $DashboardPort + 34))) {
  $WindowsPortBusy = @(Get-NetTCPConnection -State Listen -LocalPort $CandidatePort -ErrorAction SilentlyContinue)
  if ($WindowsPortBusy) {
    continue
  }
  $PreviousErrorActionPreference = $ErrorActionPreference
  try {
    $ErrorActionPreference = "Continue"
    & wsl.exe -d $Distro -- $DashboardPython -c $DashboardPortProbe $CandidatePort *> $null
    $WslPortAvailable = $LASTEXITCODE -eq 0
  } finally {
    $ErrorActionPreference = $PreviousErrorActionPreference
  }
  if ($WslPortAvailable) {
    $SelectedDashboardPort = $CandidatePort
    break
  }
}
if ($null -eq $SelectedDashboardPort) {
  throw "No free local dashboard port was found in the range $DashboardPort-$($DashboardPort + 34)."
}
$DashboardUrl = "http://127.0.0.1:$SelectedDashboardPort"

Write-Host "CARLA is reachable from WSL at ${CarlaHost}:$Port."
Write-Host "Starting the visible OpenPilot trial. Keep the CARLA window open."
Write-Host "Live OpenPilot decisions: $DashboardUrl"

$WslArguments = @(
  "-d", $Distro,
  "--cd", $WslProjectRoot,
  "--",
  "env",
  "OPENPILOT_ROOT=$OpenPilotRoot",
  "CARLA_HOST=$CarlaHost",
  "CARLA_PORT=$Port",
  "DEV=CUDA",
  "WARP_DEV=CUDA",
  "OPENCARLA_ALLOW_UNTESTED_OPENPILOT=1",
  "OPENCARLA_SPECTATOR=1",
  "OPENCARLA_DASHBOARD=1",
  "OPENCARLA_DASHBOARD_PORT=$SelectedDashboardPort",
  "OPENCARLA_RECORD_DEMO=$([int][bool]$RecordDemo)",
  $Evaluator,
  "--experiment", $Experiment,
  "--run-id", $RunId,
  "--overwrite"
)
$DashboardOpenerJob = $null
$EvaluationExitCode = $null
if (-not $NoDashboardWindow) {
  $DashboardOpenerJob = Start-Job -ScriptBlock {
    param([string]$Url)

    $Deadline = (Get-Date).AddMinutes(3)
    while ((Get-Date) -lt $Deadline) {
      try {
        $Response = Invoke-WebRequest -UseBasicParsing -Uri "$Url/health" -TimeoutSec 1
        if ($Response.StatusCode -eq 200) {
          $Edge = @(
            "C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            "C:\Program Files\Microsoft\Edge\Application\msedge.exe"
          ) | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
          if ($Edge) {
            Start-Process -FilePath $Edge -ArgumentList @("--new-window", $Url)
          } else {
            Start-Process $Url
          }
          return
        }
      } catch {
        # The bridge is still starting. Keep the helper silent and retry.
      }
      Start-Sleep -Milliseconds 500
    }
  } -ArgumentList $DashboardUrl
}

try {
  & wsl.exe @WslArguments
  $EvaluationExitCode = $LASTEXITCODE
} finally {
  if ($null -ne $DashboardOpenerJob) {
    if ($DashboardOpenerJob.State -eq "Running") {
      Stop-Job -Job $DashboardOpenerJob
    }
    Receive-Job -Job $DashboardOpenerJob -ErrorAction SilentlyContinue | Out-Null
    Remove-Job -Job $DashboardOpenerJob
  }
}
if ($EvaluationExitCode -ne 0) {
  throw "The OpenPilot trial exited with code $EvaluationExitCode."
}

$SummaryProbe = "from opencarla_eval.config import load_experiment; import sys; experiment=load_experiment(sys.argv[1]); print((experiment.output_dir / sys.argv[2] / 'summary.json').resolve())"
$SummaryWslPathRaw = (& wsl.exe -d $Distro --cd $WslProjectRoot -- $DashboardPython -c $SummaryProbe $Experiment $RunId) | Select-Object -First 1
if ($LASTEXITCODE -ne 0 -or -not $SummaryWslPathRaw) {
  throw "Could not resolve the result path from experiment '$Experiment'."
}
$SummaryWslPath = $SummaryWslPathRaw.Trim()
$SummaryPathRaw = (& wsl.exe -d $Distro -- wslpath -w $SummaryWslPath) | Select-Object -First 1
if ($LASTEXITCODE -ne 0 -or -not $SummaryPathRaw) {
  throw "Could not translate the result path into Windows: $SummaryWslPath"
}
$SummaryPath = $SummaryPathRaw.Trim()
if (-not (Test-Path -LiteralPath $SummaryPath -PathType Leaf)) {
  throw "The evaluator exited without writing its summary: $SummaryPath"
}
$Summary = Get-Content -Raw -LiteralPath $SummaryPath | ConvertFrom-Json
$RuntimeCompleted = $Summary.termination_reason -in @("timeout", "route_completed")
$SampleCount = [int]$Summary.metrics.system.sample_count
if (-not $RuntimeCompleted -or $SampleCount -le 0) {
  throw "The trial did not reach evaluation: termination=$($Summary.termination_reason), samples=$SampleCount"
}

Write-Host "Trial runtime complete with $SampleCount evaluated samples. CARLA remains open."
Write-Host "Driving-quality pass: $($Summary.metrics.quality_pass)"
Write-Host "Summary: $SummaryPath"
