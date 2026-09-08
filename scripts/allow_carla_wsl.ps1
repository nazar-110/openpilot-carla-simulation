param([string]$CarlaRoot = 'D:\CARLA\0.9.16', [string]$WslAddress, [string]$Distro = 'Ubuntu-24.04-ROS2')
$ErrorActionPreference = 'Stop'
if (-not $WslAddress) {
  $WslAddress = ((& wsl.exe -d $Distro -- hostname -I).Trim() -split '\s+')[0]
}
Start-Transcript -Path "$env:TEMP\opencarla-firewall-approval.log" -Force
try {
$exe = (Get-ChildItem -LiteralPath $CarlaRoot -Recurse -Filter CarlaUE4-Win64-Shipping.exe | Select-Object -First 1).FullName
if (-not $exe) { throw 'CARLA server executable was not found.' }
$name = 'OpenCarlaEval-WSL-RPC'
if (Get-NetFirewallRule -Name $name -ErrorAction SilentlyContinue) {
  Set-NetFirewallRule -Name $name -Enabled True -Action Allow -RemoteAddress $WslAddress
} else {
  New-NetFirewallRule -Name $name -DisplayName 'CARLA evaluation from local WSL' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 2000-2002 -Program $exe -RemoteAddress $WslAddress -Profile Any | Out-Null
}
Write-Output 'CARLA firewall access configured for the current WSL address only.'
$octets = $WslAddress.Split('.')
if ($octets.Count -ne 4 -or [int]$octets[3] -le 0 -or [int]$octets[3] -ge 255) { throw 'Expected a host IPv4 address.' }
$prefix = ($octets[0..2] -join '.')
$before = "$prefix.$([int]$octets[3]-1)"
$after = "$prefix.$([int]$octets[3]+1)"
$blocks = @(Get-NetFirewallRule -Enabled True -Action Block | Get-NetFirewallApplicationFilter |
  Where-Object { $_.Program -ieq $exe } | Get-NetFirewallRule)
foreach ($block in $blocks) {
  $addresses = @($block | Get-NetFirewallAddressFilter | Select-Object -ExpandProperty RemoteAddress)
  if ($addresses.Count -eq 1 -and $addresses[0] -eq 'Any') {
    $backup = Join-Path $env:TEMP ('opencarla-block-' + $block.Name + '.json')
    if (-not (Test-Path -LiteralPath $backup)) {
      @{Name=$block.Name; RemoteAddress=$addresses} | ConvertTo-Json | Set-Content -LiteralPath $backup
    }
    Set-NetFirewallRule -Name $block.Name -RemoteAddress @("0.0.0.1-$before", "$after-255.255.255.254", '::2-feff:ffff:ffff:ffff:ffff:ffff:ffff:ffff')
    Write-Output "Excluded only $WslAddress from CARLA block rule $($block.Name); original saved to $backup"
  }
}
} finally {
  Stop-Transcript
}
