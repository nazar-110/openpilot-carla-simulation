param(
  [switch]$SkipSmoke
)

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Push-Location $ProjectRoot
try {
  $env:PYTHONPATH = (Join-Path $ProjectRoot "src")
  python -m ruff check src tests
  python -m ruff format --check src tests
  python -m pytest
  python -m compileall -q src
  python -m opencarla_eval validate --experiment experiments/urban_suite.yaml
  if (-not $SkipSmoke) {
    python -m opencarla_eval run `
      --experiment experiments/smoke.yaml `
      --backend synthetic `
      --overwrite
    python -m opencarla_eval analyze `
      results/smoke `
      --output reports/generated/smoke
  }
}
finally {
  Pop-Location
}
