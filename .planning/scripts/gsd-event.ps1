[CmdletBinding()]
param(
    [Parameter(Mandatory)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]*$')]
    [string]$Plan,

    [Parameter(Mandatory)]
    [ValidatePattern('^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$')]
    [string]$ExecutionId,

    [Parameter(Mandatory)]
    [ValidateSet('STARTED', 'PROGRESS', 'LONG_OPERATION', 'COMPLETED', 'FAILED')]
    [string]$Event,

    [string]$Executor,
    [string]$BaseCommit,
    [string]$Branch,
    [string]$ExpectedSummary,
    [string]$Stage,
    [string]$Task,
    [string]$Commit,
    [string]$Operation,
    [int]$DeadlineMinutes,
    [string]$Reason,
    [string]$Summary,
    [int]$MutexTimeoutSeconds = 30,
    [switch]$SkipWatchdogEnsure
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Write-EventWarning {
    param([string]$Code, [string]$Message)
    Write-Warning ("{0}: {1}" -f $Code, $Message)
}

function Add-OptionalProperty {
    param($Object, [string]$Name, [string]$Value)
    if (-not [string]::IsNullOrWhiteSpace($Value)) {
        $Object[$Name] = $Value
    }
}

function Assert-RequiredValue {
    param([string]$Name, [string]$Value)
    if ([string]::IsNullOrWhiteSpace($Value)) {
        throw "$Name is required for STARTED."
    }
}

function Get-WatchdogMutexName {
    param([string]$RepoRoot)
    $normalized = [IO.Path]::GetFullPath($RepoRoot).TrimEnd([IO.Path]::DirectorySeparatorChar, [IO.Path]::AltDirectorySeparatorChar).ToUpperInvariant()
    $hasher = [Security.Cryptography.SHA256]::Create()
    try {
        $bytes = $hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($normalized))
    }
    finally { $hasher.Dispose() }
    $key = ([BitConverter]::ToString($bytes) -replace '-', '').Substring(0, 32)
    $prefix = if ($IsWindows) { 'Local\' } else { '' }
    return ("{0}GSD_WATCHDOG_{1}" -f $prefix, $key)
}

function Ensure-WatchdogRunning {
    param([string]$PlanningRoot, [string]$RuntimeRoot)
    if ($SkipWatchdogEnsure -or [Environment]::GetEnvironmentVariable('GSD_WATCHDOG_DISABLE_AUTOSTART') -eq '1') {
        return
    }

    $mutexName = Get-WatchdogMutexName $RuntimeRoot
    $probe = [Threading.Mutex]::new($false, $mutexName)
    $probeHeld = $false
    try {
        try { $probeHeld = $probe.WaitOne(0) }
        catch [Threading.AbandonedMutexException] { $probeHeld = $true }

        if (-not $probeHeld) { return }
        $probe.ReleaseMutex()
        $probeHeld = $false

        $watchdogScript = Join-Path $PlanningRoot (Join-Path 'scripts' 'gsd-watchdog.ps1')
        if (-not [IO.File]::Exists($watchdogScript)) {
            Write-EventWarning 'WATCHDOG_MISSING' "Cannot auto-start missing watchdog '$watchdogScript'."
            return
        }
        $powerShell = if ($IsWindows) { 'powershell.exe' } else { 'pwsh' }
        $arguments = @('-NoProfile')
        if ($IsWindows) { $arguments += @('-ExecutionPolicy', 'Bypass') }
        $arguments += @('-File', $watchdogScript)
        $startParams = @{ FilePath = $powerShell; ArgumentList = $arguments }
        if ($IsWindows) { $startParams.WindowStyle = 'Hidden' }
        Start-Process @startParams | Out-Null
    }
    finally {
        if ($probeHeld) { $probe.ReleaseMutex() }
        $probe.Dispose()
    }
}

if ($MutexTimeoutSeconds -le 0) {
    throw 'MutexTimeoutSeconds must be greater than zero.'
}
if ($Event -eq 'LONG_OPERATION' -and $DeadlineMinutes -le 0) {
    throw 'DeadlineMinutes must be greater than zero for LONG_OPERATION.'
}
if ($Event -eq 'STARTED') {
    Assert-RequiredValue 'Executor' $Executor
    Assert-RequiredValue 'BaseCommit' $BaseCommit
    Assert-RequiredValue 'Branch' $Branch
    Assert-RequiredValue 'ExpectedSummary' $ExpectedSummary
}

$planningRoot = Split-Path -Parent $PSScriptRoot
$testRuntimeRoot = [Environment]::GetEnvironmentVariable('GSD_WATCHDOG_TEST_RUNTIME_ROOT')
$runtimeRoot = if (
    [Environment]::GetEnvironmentVariable('GSD_WATCHDOG_DISABLE_AUTOSTART') -eq '1' -and
    -not [string]::IsNullOrWhiteSpace($testRuntimeRoot)
) {
    [IO.Path]::GetFullPath($testRuntimeRoot)
}
else {
    Join-Path $planningRoot 'runtime'
}
$eventsRoot = Join-Path $runtimeRoot 'events'
[System.IO.Directory]::CreateDirectory($eventsRoot) | Out-Null
$watchdogMutexScope = if (
    [Environment]::GetEnvironmentVariable('GSD_WATCHDOG_DISABLE_AUTOSTART') -eq '1' -and
    -not [string]::IsNullOrWhiteSpace($testRuntimeRoot)
) { $runtimeRoot } else { Split-Path -Parent $planningRoot }
Ensure-WatchdogRunning $planningRoot $watchdogMutexScope

# The plan is validated for the state-file name. Hashing still gives a stable,
# collision-resistant mutex key if future plan naming rules become broader.
$sha256 = [System.Security.Cryptography.SHA256]::Create()
try {
    $hashBytes = $sha256.ComputeHash([System.Text.Encoding]::UTF8.GetBytes($Plan))
}
finally {
    $sha256.Dispose()
}
$mutexKey = ([System.BitConverter]::ToString($hashBytes) -replace '-', '').Substring(0, 32)
$mutex = [System.Threading.Mutex]::new($false, "GSD_EVENT_$mutexKey")
$mutexHeld = $false
$target = Join-Path $eventsRoot "$Plan.json"

try {
    try {
        $mutexHeld = $mutex.WaitOne([TimeSpan]::FromSeconds($MutexTimeoutSeconds))
    }
    catch [System.Threading.AbandonedMutexException] {
        $mutexHeld = $true
        Write-EventWarning 'ABANDONED_MUTEX' "Recovered mutex for plan '$Plan'."
    }

    if (-not $mutexHeld) {
        Write-EventWarning 'MUTEX_TIMEOUT' "Could not acquire mutex for plan '$Plan' within $MutexTimeoutSeconds seconds."
        exit 2
    }

    $current = $null
    if ([System.IO.File]::Exists($target)) {
        try {
            $current = Get-Content -Raw -LiteralPath $target | ConvertFrom-Json
        }
        catch {
            throw "Cannot read current event state '$target': $($_.Exception.Message)"
        }
    }

    if ($Event -ne 'STARTED') {
        if ($null -eq $current) {
            Write-EventWarning 'NO_ACTIVE_EXECUTION' "Rejected $Event for plan '$Plan': no active lifecycle exists."
            return
        }
        if ($current.execution_id -ne $ExecutionId) {
            Write-EventWarning 'STALE_EXECUTION' "Rejected $Event for plan '$Plan': supplied execution_id does not own the current lifecycle."
            return
        }
    }

    $state = [ordered]@{
        plan         = $Plan
        execution_id = $ExecutionId.ToLowerInvariant()
        event        = $Event
        updated_at   = [DateTime]::UtcNow.ToString('o')
    }

    # Event files are current-state snapshots. Carry lifecycle identity forward
    # so a watchdog that starts or restarts after STARTED can reconstruct the
    # active execution without an event-history service.
    if ($Event -ne 'STARTED') {
        foreach ($name in @('executor', 'base_commit', 'branch', 'expected_summary')) {
            $property = $current.PSObject.Properties[$name]
            if ($null -ne $property) {
                Add-OptionalProperty $state $name ([string]$property.Value)
            }
        }
    }

    switch ($Event) {
        'STARTED' {
            foreach ($field in @(
                @{ Name = 'executor'; Value = $Executor },
                @{ Name = 'base_commit'; Value = $BaseCommit },
                @{ Name = 'branch'; Value = $Branch },
                @{ Name = 'expected_summary'; Value = $ExpectedSummary }
            )) { Add-OptionalProperty $state $field.Name $field.Value }
        }
        'PROGRESS' {
            foreach ($field in @(
                @{ Name = 'stage'; Value = $Stage },
                @{ Name = 'task'; Value = $Task },
                @{ Name = 'commit'; Value = $Commit }
            )) { Add-OptionalProperty $state $field.Name $field.Value }
        }
        'LONG_OPERATION' {
            Add-OptionalProperty $state 'operation' $Operation
            $state['deadline_at'] = [DateTime]::UtcNow.AddMinutes($DeadlineMinutes).ToString('o')
        }
        'COMPLETED' {
            Add-OptionalProperty $state 'commit' $Commit
            Add-OptionalProperty $state 'summary' $Summary
        }
        'FAILED' { Add-OptionalProperty $state 'reason' $Reason }
    }

    # Keep read, ownership validation, construction, and replacement in this
    # same mutex-protected critical section. Readers observe either old JSON or
    # fully-written new JSON, never a partial document.
    $temporary = Join-Path $eventsRoot ('{0}.{1}.{2}.tmp' -f $Plan, $PID, [guid]::NewGuid().ToString('N'))
    $backup = $null
    try {
        $json = $state | ConvertTo-Json -Depth 4
        [System.IO.File]::WriteAllText($temporary, $json, [System.Text.UTF8Encoding]::new($false))

        # Test-only hook: makes the mutex race test deterministic without
        # changing normal emitter semantics.
        $testDelay = [Environment]::GetEnvironmentVariable('GSD_EVENT_TEST_HOLD_MUTEX_MS')
        if ($testDelay -match '^\d+$' -and [int]$testDelay -gt 0) {
            $testSignal = [Environment]::GetEnvironmentVariable('GSD_EVENT_TEST_MUTEX_ENTER_SIGNAL')
            if (-not [string]::IsNullOrWhiteSpace($testSignal)) {
                [System.IO.File]::WriteAllText($testSignal, 'entered')
            }
            Start-Sleep -Milliseconds ([int]$testDelay)
        }

        if ([System.IO.File]::Exists($target)) {
            # Windows PowerShell/.NET Framework requires a backup-file path;
            # remove that private backup immediately after the atomic replace.
            $backup = "$temporary.bak"
            [System.IO.File]::Replace($temporary, $target, $backup)
            if ([System.IO.File]::Exists($backup)) {
                Remove-Item -LiteralPath $backup -Force
            }
        }
        else {
            [System.IO.File]::Move($temporary, $target)
        }
    }
    finally {
        if ([System.IO.File]::Exists($temporary)) {
            Remove-Item -LiteralPath $temporary -Force
        }
        if ($null -ne $backup -and [System.IO.File]::Exists($backup)) {
            Remove-Item -LiteralPath $backup -Force
        }
    }
}
finally {
    if ($mutexHeld) { $mutex.ReleaseMutex() }
    $mutex.Dispose()
}
