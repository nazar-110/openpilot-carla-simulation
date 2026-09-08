param(
  [Parameter(Mandatory = $true)]
  [string]$CarlaRoot,

  [int]$Port = 2000,

  [ValidateSet("Epic")]
  [string]$Quality = "Epic",

  [switch]$ShowWindow
)

$ErrorActionPreference = "Stop"
$resolvedRoot = (Resolve-Path -LiteralPath $CarlaRoot).Path
$executable = Join-Path $resolvedRoot "CarlaUE4.exe"
if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
  throw "CARLA 0.9.16 executable not found: $executable"
}

$arguments = @(
  "-nosound"
  "-quality-level=$Quality"
  "-carla-rpc-port=$Port"
)
if (-not $ShowWindow) {
  $arguments += "-RenderOffScreen"
}

Write-Host "Starting CARLA from $executable on port $Port at $Quality quality."
& $executable @arguments
