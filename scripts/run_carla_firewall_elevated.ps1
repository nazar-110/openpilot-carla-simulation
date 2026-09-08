$ErrorActionPreference = "Stop"
$configScript = Join-Path $PSScriptRoot "configure_carla_firewall.ps1"
$logPath = "D:\CARLA\firewall-elevated.log"
$statusPath = "D:\CARLA\firewall-elevated-status.json"

try {
  & $configScript `
    -Mode Apply `
    -ResultPath "D:\CARLA\firewall-result.json" `
    -StatePath "D:\CARLA\wsl-firewall-state.json" *>&1 |
    Out-File -LiteralPath $logPath -Encoding utf8
  $status = [ordered]@{
    success = $true
    completed_at = (Get-Date).ToString("o")
    log_path = $logPath
  }
} catch {
  $_ | Format-List * -Force | Out-File -LiteralPath $logPath -Encoding utf8
  $status = [ordered]@{
    success = $false
    completed_at = (Get-Date).ToString("o")
    error = $_.Exception.Message
    log_path = $logPath
  }
}

$status | ConvertTo-Json -Depth 5 |
  Set-Content -LiteralPath $statusPath -Encoding utf8
if (-not $status.success) {
  exit 1
}
