[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$planningRoot = Join-Path $repoRoot '.planning'
$wrapper = Join-Path $planningRoot 'gsd-claude-executor.ps1'
$providerFile = Join-Path $planningRoot 'cc-provider.local'
$providersFile = Join-Path $planningRoot 'cc-providers.psd1'
$tierFile = Join-Path $planningRoot 'cc-tier.local'
$lockRoot = Join-Path $planningRoot (Join-Path 'runtime' 'locks')
$eventsRoot = Join-Path $planningRoot (Join-Path 'runtime' 'events')
$powerShell = if ($IsWindows) { 'powershell.exe' } else { 'pwsh' }
[IO.Directory]::CreateDirectory($lockRoot) | Out-Null
[IO.Directory]::CreateDirectory($eventsRoot) | Out-Null

function Assert-That([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw "ASSERTION FAILED: $Message" }
}

function Get-TestEnv([string]$Name) {
    return [Environment]::GetEnvironmentVariable($Name, 'Process')
}
function Set-TestEnv([string]$Name, [string]$Value) {
    [Environment]::SetEnvironmentVariable($Name, $Value, 'Process')
}

function Invoke-Wrapper([string]$Prompt) {
    $prev = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $out = $Prompt | & $powerShell -NoProfile -File $wrapper 2>&1
        return @{ ExitCode = $LASTEXITCODE; Text = ([string]($out | Out-String)) }
    }
    finally {
        $ErrorActionPreference = $prev
    }
}

# These tests require the tier selector to be "auto" so PLAN routing is what is
# being observed, and the active provider to be deepseek-current.
$tierValue = (Get-Content $tierFile -Raw).Trim()
Assert-That ($tierValue -eq 'auto') "T2/T3 require cc-tier.local=auto (found '$tierValue')"

# Snapshot the two files the mutating tests (T4/T5) overwrite, for restore.
$origProvider = (Get-Content $providerFile -Raw).Trim()
$origProviders = [IO.File]::ReadAllText($providersFile)

function Reset-LocalState {
    [IO.File]::WriteAllText($providerFile, $origProvider + "`n", (New-Object System.Text.UTF8Encoding($false)))
    [IO.File]::WriteAllText($providersFile, $origProviders, (New-Object System.Text.UTF8Encoding($false)))
}

# Env snapshot + test setup (offline, no watchdog auto-start).
$prevDryRun = Get-TestEnv 'CROSS_AI_DRY_RUN'
$prevWatchdog = Get-TestEnv 'GSD_WATCHDOG_DISABLE_AUTOSTART'
$prevBaseUrl = Get-TestEnv 'ANTHROPIC_BASE_URL'
$prevAuthToken = Get-TestEnv 'ANTHROPIC_AUTH_TOKEN'

Set-TestEnv 'GSD_WATCHDOG_DISABLE_AUTOSTART' '1'
Set-TestEnv 'CROSS_AI_DRY_RUN' '1'

try {
    # --- T1: active provider resolves (base_url + model + token presence) ---
    $r = Invoke-Wrapper 'Execute .planning/phases/97-provider-test/97-01-PLAN.md.'
    Assert-That ($r.ExitCode -eq 0) "T1 exit 0, got $($r.ExitCode)"
    Assert-That ($r.Text -match 'provider=deepseek-current') 'T1 provider name'
    Assert-That ($r.Text -match 'base_url=https://api\.deepseek\.com/anthropic') 'T1 base_url'
    Assert-That ($r.Text -match 'model=deepseek-v4-flash') 'T1 flash model'
    Assert-That ($r.Text -match 'auth_token_present=True') 'T1 token presence flag'
    Assert-That ($r.Text -notmatch 'AuthToken=') 'T1 must not log the token value'

    # --- T2: cc-tier.local=auto + PLAN flash-high ---
    $r = Invoke-Wrapper "Execute .planning/phases/97-provider-test/97-02-PLAN.md.`nEXECUTION_TIER: flash-high"
    Assert-That ($r.ExitCode -eq 0) "T2 exit 0, got $($r.ExitCode)"
    Assert-That ($r.Text -match 'tier=flash-high\b') 'T2 tier'
    Assert-That ($r.Text -match 'model=deepseek-v4-flash') 'T2 model'
    Assert-That ($r.Text -match 'effort=high\b') 'T2 effort'
    Assert-That ($r.Text -match 'max_turns=32\b') 'T2 max_turns'

    # --- T3: PLAN flash-max ---
    $r = Invoke-Wrapper "Execute .planning/phases/97-provider-test/97-03-PLAN.md.`nEXECUTION_TIER: flash-max"
    Assert-That ($r.ExitCode -eq 0) "T3 exit 0, got $($r.ExitCode)"
    Assert-That ($r.Text -match 'tier=flash-max\b') 'T3 tier'
    Assert-That ($r.Text -match 'model=deepseek-v4-flash') 'T3 model'
    Assert-That ($r.Text -match 'effort=max\b') 'T3 effort'
    Assert-That ($r.Text -match 'max_turns=64\b') 'T3 max_turns'

    # --- T4: unknown provider must fail-fast ---
    [IO.File]::WriteAllText($providerFile, "does-not-exist`n", (New-Object System.Text.UTF8Encoding($false)))
    $r = Invoke-Wrapper "Execute .planning/phases/97-provider-test/97-04-PLAN.md.`nEXECUTION_TIER: flash-high"
    Assert-That ($r.ExitCode -eq 4) "T4 exit 4, got $($r.ExitCode)"
    Assert-That ($r.Text -match "unknown provider 'does-not-exist'") 'T4 unknown-provider message'
    Reset-LocalState

    # --- T5: provider missing Models[flash-max] must fail-fast ---
    $testRegistry = @'
@{
    "test-missing" = @{
        BaseUrl   = "https://example.invalid/anthropic"
        AuthToken = "sk-test-missing-token"

        Models = @{
            "flash-high" = "model-x"
        }
    }
}
'@
    [IO.File]::WriteAllText($providersFile, $testRegistry, (New-Object System.Text.UTF8Encoding($false)))
    [IO.File]::WriteAllText($providerFile, "test-missing`n", (New-Object System.Text.UTF8Encoding($false)))
    $r = Invoke-Wrapper "Execute .planning/phases/97-provider-test/97-05-PLAN.md.`nEXECUTION_TIER: flash-max"
    Assert-That ($r.ExitCode -eq 4) "T5 exit 4, got $($r.ExitCode)"
    Assert-That ($r.Text -match "has no model mapping for tier 'flash-max'") 'T5 missing-mapping message'
    Reset-LocalState

    # --- T6: decoupled from global env + User/Machine untouched ---
    Set-TestEnv 'ANTHROPIC_BASE_URL' 'https://global-ignored.example/anthropic'
    Set-TestEnv 'ANTHROPIC_AUTH_TOKEN' 'sk-global-ignored'
    $userBaseUrlBefore = [Environment]::GetEnvironmentVariable('ANTHROPIC_BASE_URL', 'User')
    $machineBaseUrlBefore = [Environment]::GetEnvironmentVariable('ANTHROPIC_BASE_URL', 'Machine')

    $r = Invoke-Wrapper "Execute .planning/phases/97-provider-test/97-06-PLAN.md.`nEXECUTION_TIER: flash-high"
    Assert-That ($r.ExitCode -eq 0) "T6 exit 0, got $($r.ExitCode)"
    Assert-That ($r.Text -match 'base_url=https://api\.deepseek\.com/anthropic') 'T6 uses project base_url'
    Assert-That ($r.Text -notmatch 'global-ignored') 'T6 must ignore global ANTHROPIC_BASE_URL'

    $userBaseUrlAfter = [Environment]::GetEnvironmentVariable('ANTHROPIC_BASE_URL', 'User')
    $machineBaseUrlAfter = [Environment]::GetEnvironmentVariable('ANTHROPIC_BASE_URL', 'Machine')
    Assert-That ($userBaseUrlAfter -eq $userBaseUrlBefore) 'T6 User env unchanged'
    Assert-That ($machineBaseUrlAfter -eq $machineBaseUrlBefore) 'T6 Machine env unchanged'
}
finally {
    Reset-LocalState
    Set-TestEnv 'CROSS_AI_DRY_RUN' $prevDryRun
    Set-TestEnv 'GSD_WATCHDOG_DISABLE_AUTOSTART' $prevWatchdog
    Set-TestEnv 'ANTHROPIC_BASE_URL' $prevBaseUrl
    Set-TestEnv 'ANTHROPIC_AUTH_TOKEN' $prevAuthToken

    Get-ChildItem $lockRoot -Filter '97-*.lock' -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue
    Get-ChildItem $eventsRoot -Filter '97-*.json' -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue
}

Write-Host 'PASS: gsd-claude-executor provider tests (T1-T6)'
