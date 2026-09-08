param(
  [ValidateSet("Apply", "Restore")]
  [string]$Mode = "Apply",
  [ValidateRange(1, 65533)]
  [int]$BasePort = 2000,
  [string]$WslSubnet = "Any",
  [string]$InterfaceAlias = "vEthernet (WSL)",
  [string]$ResultPath = "$env:TEMP\opencarla-firewall-result.json",
  [string]$StatePath = "$env:ProgramData\OpenCarlaEval\wsl-firewall-state.json"
)

$ErrorActionPreference = "Stop"
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
  throw "Run this script from an elevated PowerShell session."
}

$displayName = "CARLA 0.9.16 from WSL2"

if ($Mode -eq "Apply") {
  $adapter = Get-NetAdapter -IncludeHidden -Name $InterfaceAlias -ErrorAction Stop
  if ($adapter.Status -ne "Up") {
    throw "The '$InterfaceAlias' adapter is not up. Start the WSL distribution and try again."
  }
  if ($adapter.InterfaceDescription -notlike "Hyper-V Virtual Ethernet Adapter*") {
    throw "Refusing to exempt non-Hyper-V adapter '$InterfaceAlias'."
  }

  # Windows 10 can filter WSL traffic before an ordinary inbound port rule is
  # evaluated. Preserve every existing exclusion, then exempt only the host-local
  # WSL virtual adapter. The saved state makes the change reversible.
  $firewallProfiles = @(Get-NetFirewallProfile -PolicyStore PersistentStore)
  if (-not (Test-Path -LiteralPath $StatePath)) {
    $stateDirectory = Split-Path -Parent $StatePath
    New-Item -ItemType Directory -Path $stateDirectory -Force | Out-Null
    $previousProfiles = $firewallProfiles | ForEach-Object {
      [ordered]@{
        name = [string]$_.Name
        disabled_interface_aliases = @($_.DisabledInterfaceAliases | ForEach-Object { [string]$_ })
      }
    }
    [ordered]@{
      created_at = (Get-Date).ToString("o")
      interface_alias = $InterfaceAlias
      profiles = @($previousProfiles)
    } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $StatePath -Encoding UTF8
  }

  $rollbackProfiles = @($firewallProfiles | ForEach-Object {
    [pscustomobject]@{
      name = [string]$_.Name
      aliases = @($_.DisabledInterfaceAliases | ForEach-Object { [string]$_ })
    }
  })
  try {
    foreach ($profile in $firewallProfiles) {
      $aliases = @(
        $profile.DisabledInterfaceAliases |
          ForEach-Object { [string]$_ } |
          Where-Object { $_ -and $_ -ne "NotConfigured" }
      )
      if ($aliases -notcontains $InterfaceAlias) {
        $aliases += $InterfaceAlias
      }
      Set-NetFirewallProfile -PolicyStore PersistentStore -Name $profile.Name `
        -DisabledInterfaceAliases $aliases
    }
  } catch {
    foreach ($profile in $rollbackProfiles) {
      Set-NetFirewallProfile -PolicyStore PersistentStore -Name $profile.name `
        -DisabledInterfaceAliases $profile.aliases
    }
    throw
  }

  Get-NetFirewallRule -DisplayName $displayName -ErrorAction SilentlyContinue |
    Remove-NetFirewallRule
  $rule = New-NetFirewallRule `
    -DisplayName $displayName `
    -Description "Allow the local WSL2 subnet to reach the CARLA RPC and streaming ports." `
    -Direction Inbound `
    -Action Allow `
    -Protocol TCP `
    -LocalPort "$BasePort-$($BasePort + 2)" `
    -RemoteAddress $WslSubnet `
    -InterfaceAlias $InterfaceAlias `
    -EdgeTraversalPolicy Allow `
    -Profile Any
} else {
  if (-not (Test-Path -LiteralPath $StatePath)) {
    throw "Cannot restore because rollback state was not found at '$StatePath'."
  }
  $state = Get-Content -Raw -LiteralPath $StatePath | ConvertFrom-Json
  foreach ($profile in $state.profiles) {
    Set-NetFirewallProfile -PolicyStore PersistentStore -Name $profile.name `
      -DisabledInterfaceAliases @($profile.disabled_interface_aliases)
  }
  Get-NetFirewallRule -DisplayName $displayName -ErrorAction SilentlyContinue |
    Remove-NetFirewallRule
  $rule = $null
  $adapter = Get-NetAdapter -IncludeHidden -Name $InterfaceAlias -ErrorAction SilentlyContinue
}

$portFilter = if ($rule) { $rule | Get-NetFirewallPortFilter }
$addressFilter = if ($rule) { $rule | Get-NetFirewallAddressFilter }
$interfaceFilter = if ($rule) { $rule | Get-NetFirewallInterfaceFilter }
$activeRule = Get-NetFirewallRule -PolicyStore ActiveStore -DisplayName $displayName -ErrorAction SilentlyContinue
$profiles = Get-NetFirewallProfile -PolicyStore PersistentStore | ForEach-Object {
  [ordered]@{
    name = [string]$_.Name
    enabled = [string]$_.Enabled
    disabled_interface_aliases = @($_.DisabledInterfaceAliases | ForEach-Object { [string]$_ })
    default_inbound_action = [string]$_.DefaultInboundAction
    allow_inbound_rules = [string]$_.AllowInboundRules
    allow_local_firewall_rules = [string]$_.AllowLocalFirewallRules
  }
}
$result = [ordered]@{
  mode = $Mode
  display_name = $displayName
  enabled = [string]$rule.Enabled
  direction = [string]$rule.Direction
  action = [string]$rule.Action
  protocol = [string]$portFilter.Protocol
  local_port = [string]$portFilter.LocalPort
  remote_address = @($addressFilter.RemoteAddress)
  interface_alias = @($interfaceFilter.InterfaceAlias)
  adapter_status = [string]$adapter.Status
  rollback_state_path = $StatePath
  active_enforcement_status = [string]$activeRule.EnforcementStatus
  active_policy_store_source_type = [string]$activeRule.PolicyStoreSourceType
  profiles = @($profiles)
}
$result | ConvertTo-Json | Set-Content -LiteralPath $ResultPath -Encoding UTF8
$result | ConvertTo-Json
