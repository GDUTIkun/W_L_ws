[CmdletBinding()]
param(
    [int]$RunForSeconds = 0,
    [int]$DebounceMilliseconds = 150,
    [double]$SilenceTimeoutSeconds = 900,
    [double]$GraceSeconds = 300,
    [double]$ScanIntervalSeconds = 60
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-WatchdogPaths {
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
    $repoRoot = Split-Path -Parent $planningRoot
    [ordered]@{
        RepoRoot = $repoRoot
        RuntimeRoot = $runtimeRoot
        MutexScope = if (
            [Environment]::GetEnvironmentVariable('GSD_WATCHDOG_DISABLE_AUTOSTART') -eq '1' -and
            -not [string]::IsNullOrWhiteSpace($testRuntimeRoot)
        ) { $runtimeRoot } else { $repoRoot }
        Events = Join-Path $runtimeRoot 'events'
        Notifications = Join-Path $runtimeRoot 'notifications'
        Log = Join-Path (Join-Path $runtimeRoot 'watchdog') 'watchdog.log'
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

function Write-WatchdogLog {
    param([System.Collections.IDictionary]$Context, [string]$Message)
    $line = '{0} {1}' -f [DateTime]::UtcNow.ToString('o'), $Message
    Add-Content -LiteralPath $Context.Paths.Log -Value $line -Encoding utf8
}

function Get-WatchdogNow {
    param([System.Collections.IDictionary]$Context)
    return & $Context.NowProvider
}

function ConvertTo-WatchdogInstant {
    param($Value)
    if ($Value -is [DateTimeOffset]) { return [DateTimeOffset]$Value }
    if ($Value -is [DateTime]) { return [DateTimeOffset]([DateTime]$Value) }
    return [DateTimeOffset]::Parse(
        [string]$Value,
        [Globalization.CultureInfo]::InvariantCulture,
        [Globalization.DateTimeStyles]::RoundtripKind
    )
}

function Resolve-ExpectedSummaryPath {
    param([System.Collections.IDictionary]$Context, [string]$ExpectedSummary)
    if ([IO.Path]::IsPathRooted($ExpectedSummary)) { return $ExpectedSummary }
    return Join-Path $Context.Paths.RepoRoot $ExpectedSummary
}

function Get-SummaryMtime {
    param([System.Collections.IDictionary]$Context, [string]$ExpectedSummary)
    $path = Resolve-ExpectedSummaryPath $Context $ExpectedSummary
    if (-not [IO.File]::Exists($path)) { return $null }
    return [IO.File]::GetLastWriteTimeUtc($path)
}

function New-WatchdogContext {
    param(
        [int]$DebounceMilliseconds = 150,
        [double]$SilenceTimeoutSeconds = 900,
        [double]$GraceSeconds = 300,
        [double]$ScanIntervalSeconds = 60,
        [scriptblock]$NowProvider = { [DateTime]::UtcNow },
        [scriptblock]$GitBranchProvider = { (git branch --show-current).Trim() },
        [scriptblock]$GitHeadProvider = { (git rev-parse --verify HEAD 2>$null | Out-String).Trim() }
    )
    if ($DebounceMilliseconds -lt 100 -or $DebounceMilliseconds -gt 500) {
        throw 'DebounceMilliseconds must be between 100 and 500.'
    }
    if ($SilenceTimeoutSeconds -le 0 -or $GraceSeconds -lt 0 -or $ScanIntervalSeconds -le 0) {
        throw 'SilenceTimeoutSeconds and ScanIntervalSeconds must be positive; GraceSeconds cannot be negative.'
    }
    $paths = Get-WatchdogPaths
    foreach ($directory in @($paths.Events, $paths.Notifications, (Split-Path -Parent $paths.Log))) {
        [System.IO.Directory]::CreateDirectory($directory) | Out-Null
    }
    [ordered]@{
        Paths = $paths
        DebounceMilliseconds = $DebounceMilliseconds
        SilenceTimeoutSeconds = $SilenceTimeoutSeconds
        GraceSeconds = $GraceSeconds
        ScanIntervalSeconds = $ScanIntervalSeconds
        NowProvider = $NowProvider
        GitBranchProvider = $GitBranchProvider
        GitHeadProvider = $GitHeadProvider
        States = @{}
        # Current-state files can only yield duplicate delivery of the latest
        # logical event. One last key per PLAN is therefore sufficient; no
        # event-history registry is retained.
        LastLogicalKeys = @{}
        LastFilesystemSignal = @{}
        SnapshotSignatures = @{}
    }
}

function Read-EventJson {
    param([System.Collections.IDictionary]$Context, [string]$Path)
    for ($attempt = 1; $attempt -le 3; $attempt++) {
        try {
            return (Get-Content -Raw -LiteralPath $Path | ConvertFrom-Json)
        }
        catch {
            if ($attempt -eq 3) {
                Write-WatchdogLog $Context "JSON_READ_WARNING path='$Path' error='$($_.Exception.Message)'"
                return $null
            }
            Start-Sleep -Milliseconds 50
        }
    }
}

function Get-EventProperty {
    param($Event, [string]$Name)
    $property = $Event.PSObject.Properties[$Name]
    if ($null -eq $property) { return $null }
    return $property.Value
}

function Write-AtomicJson {
    param([string]$Path, $Value)
    $directory = Split-Path -Parent $Path
    $temporary = Join-Path $directory ('{0}.{1}.{2}.tmp' -f [IO.Path]::GetFileNameWithoutExtension($Path), $PID, [guid]::NewGuid().ToString('N'))
    $backup = $null
    try {
        [IO.File]::WriteAllText($temporary, ($Value | ConvertTo-Json -Depth 5), [Text.UTF8Encoding]::new($false))
        if ([IO.File]::Exists($Path)) {
            $backup = "$temporary.bak"
            [IO.File]::Replace($temporary, $Path, $backup)
        }
        else {
            [IO.File]::Move($temporary, $Path)
        }
    }
    finally {
        if ([IO.File]::Exists($temporary)) { Remove-Item -LiteralPath $temporary -Force }
        if ($null -ne $backup -and [IO.File]::Exists($backup)) { Remove-Item -LiteralPath $backup -Force }
    }
}

function Clear-OldNotification {
    param([System.Collections.IDictionary]$Context, $Event)
    $notificationPath = Join-Path $Context.Paths.Notifications "$($Event.plan).json"
    if (-not [IO.File]::Exists($notificationPath)) { return }
    try { $notification = Get-Content -Raw -LiteralPath $notificationPath | ConvertFrom-Json }
    catch {
        Write-WatchdogLog $Context "JSON_READ_WARNING path='$notificationPath' error='$($_.Exception.Message)'"
        return
    }
    if ($notification.execution_id -ne $Event.execution_id) {
        Remove-Item -LiteralPath $notificationPath -Force
        Write-WatchdogLog $Context "OLD_NOTIFICATION_CLEARED plan='$($Event.plan)' execution_id='$($notification.execution_id)'"
    }
}

function Write-TerminalNotification {
    param([System.Collections.IDictionary]$Context, $Event, [string]$Type, [string]$Executor)
    $notification = [ordered]@{
        type = $Type
        plan = $Event.plan
        execution_id = $Event.execution_id
        executor = $Executor
        updated_at = $Event.updated_at
    }
    if ($Type -eq 'PLAN_COMPLETED') {
        $commit = Get-EventProperty $Event 'commit'
        $summary = Get-EventProperty $Event 'summary'
        if ($null -ne $commit) { $notification.commit = $commit }
        if ($null -ne $summary) { $notification.summary = $summary }
    }
    else {
        $reason = Get-EventProperty $Event 'reason'
        if ($null -ne $reason) { $notification.reason = $reason }
    }
    Write-AtomicJson (Join-Path $Context.Paths.Notifications "$($Event.plan).json") $notification
    Write-WatchdogLog $Context "$Type`_WRITTEN plan='$($Event.plan)' execution_id='$($Event.execution_id)'"
}

function Clear-StallNotification {
    param([System.Collections.IDictionary]$Context, $State)
    $path = Join-Path $Context.Paths.Notifications "$($State.plan).json"
    if (-not [IO.File]::Exists($path)) { return }
    try { $notification = Get-Content -Raw -LiteralPath $path | ConvertFrom-Json }
    catch { return }
    if ($notification.type -eq 'SUSPECTED_STALL' -and $notification.execution_id -eq $State.execution_id) {
        Remove-Item -LiteralPath $path -Force
        Write-WatchdogLog $Context "STALL_MAILBOX_CLEARED_AFTER_PROGRESS plan='$($State.plan)' execution_id='$($State.execution_id)'"
    }
}

function Reset-StallObservation {
    param([System.Collections.IDictionary]$Context, $State, [DateTime]$Now)
    $State.last_update = $Now.ToString('o')
    $State.overdue_since = $null
    $State.stall_reported = $false
    $State.duplicate_stall_logged = $false
    Clear-StallNotification $Context $State
}

function Write-SuspectedStallNotification {
    param([System.Collections.IDictionary]$Context, $State, [DateTime]$Now, [bool]$HeadProgress, [bool]$SummaryProgress, [bool]$BranchMismatch, [string]$CurrentBranch)
    $notification = [ordered]@{
        type = 'SUSPECTED_STALL'
        plan = $State.plan
        execution_id = $State.execution_id
        executor = $State.executor
        last_event = $State.last_event
        last_update = $State.last_update
        detected_at = $Now.ToString('o')
        head_progress = $HeadProgress
        summary_progress = $SummaryProgress
        branch_mismatch = $BranchMismatch
    }
    if ($BranchMismatch) {
        $notification.expected_branch = $State.branch
        $notification.current_branch = $CurrentBranch
    }
    Write-AtomicJson (Join-Path $Context.Paths.Notifications "$($State.plan).json") $notification
    $State.stall_reported = $true
    $State.duplicate_stall_logged = $false
    Write-WatchdogLog $Context "SUSPECTED_STALL_WRITTEN plan='$($State.plan)' execution_id='$($State.execution_id)'"
}

function Invoke-FallbackInspection {
    param([System.Collections.IDictionary]$Context, $State, [DateTime]$Now)
    Write-WatchdogLog $Context "FALLBACK_INSPECTION_STARTED plan='$($State.plan)' execution_id='$($State.execution_id)'"
    $currentBranch = $null
    try { $currentBranch = (& $Context.GitBranchProvider).Trim() }
    catch { Write-WatchdogLog $Context "BRANCH_READ_WARNING plan='$($State.plan)' error='$($_.Exception.Message)'" }
    $branchMismatch = $currentBranch -ne $State.branch
    if ($branchMismatch) {
        Write-WatchdogLog $Context "BRANCH_MISMATCH plan='$($State.plan)' expected='$($State.branch)' current='$currentBranch'"
    }

    $headProgress = $false
    if (-not $branchMismatch) {
        try {
            $head = (& $Context.GitHeadProvider).Trim()
            if ($head -ne $State.last_seen_head) {
                $State.last_seen_head = $head
                Reset-StallObservation $Context $State $Now
                Write-WatchdogLog $Context "NEW_HEAD_PROGRESS_OBSERVED plan='$($State.plan)' head='$head'"
                return
            }
        }
        catch { Write-WatchdogLog $Context "HEAD_READ_WARNING plan='$($State.plan)' error='$($_.Exception.Message)'" }
    }

    $summaryProgress = $false
    $summaryMtime = Get-SummaryMtime $Context $State.expected_summary
    if ($null -ne $summaryMtime -and ($null -eq $State.last_seen_summary_mtime -or $summaryMtime -gt $State.last_seen_summary_mtime)) {
        $State.last_seen_summary_mtime = $summaryMtime
        $summaryProgress = $true
        Reset-StallObservation $Context $State $Now
        Write-WatchdogLog $Context "NEW_SUMMARY_PROGRESS_OBSERVED plan='$($State.plan)' path='$($State.expected_summary)'"
        return
    }

    if ($State.stall_reported) {
        if (-not $State.duplicate_stall_logged) {
            $State.duplicate_stall_logged = $true
            Write-WatchdogLog $Context "DUPLICATE_STALL_SUPPRESSED plan='$($State.plan)' execution_id='$($State.execution_id)'"
        }
        return
    }
    Write-SuspectedStallNotification $Context $State $Now $headProgress $summaryProgress $branchMismatch $currentBranch
}

function Invoke-WatchdogScan {
    param([System.Collections.IDictionary]$Context, [DateTime]$Now = (Get-WatchdogNow $Context))
    $nowInstant = [DateTimeOffset]$Now
    foreach ($plan in @($Context.States.Keys)) {
        $state = $Context.States[$plan]
        if ($state.terminal) { continue }
        $fallbackAt = $null
        if ($state.last_event -eq 'LONG_OPERATION') {
            if ($null -eq $state.deadline_at) { continue }
            $deadline = ConvertTo-WatchdogInstant $state.deadline_at
            $fallbackAt = $deadline.AddSeconds($Context.GraceSeconds)
            if ($nowInstant -lt $fallbackAt) {
                if (-not $state.long_deadline_logged) {
                    $state.long_deadline_logged = $true
                    Write-WatchdogLog $Context "LONG_OPERATION_WITHIN_DEADLINE plan='$($state.plan)'"
                }
                continue
            }
        }
        else {
            $silenceAt = (ConvertTo-WatchdogInstant $state.last_update).AddSeconds($Context.SilenceTimeoutSeconds)
            if ($nowInstant -lt $silenceAt) { continue }
            if ($null -eq $state.overdue_since) {
                $state.overdue_since = $silenceAt.ToString('o')
                Write-WatchdogLog $Context "EVENT_OVERDUE plan='$($state.plan)' execution_id='$($state.execution_id)'"
            }
            $fallbackAt = $silenceAt.AddSeconds($Context.GraceSeconds)
            if ($nowInstant -lt $fallbackAt) { continue }
        }
        Invoke-FallbackInspection $Context $state $Now
    }
}

function Process-WatchdogEvent {
    param([System.Collections.IDictionary]$Context, $Event)
    foreach ($field in @('plan', 'execution_id', 'event', 'updated_at')) {
        if ([string]::IsNullOrWhiteSpace([string]$Event.$field)) {
            Write-WatchdogLog $Context "INVALID_EVENT missing='$field'"
            return
        }
    }
    $key = '{0}|{1}|{2}|{3}' -f $Event.plan, $Event.execution_id, $Event.event, $Event.updated_at
    if ($Context.LastLogicalKeys[$Event.plan] -eq $key) {
        Write-WatchdogLog $Context "DUPLICATE_EVENT_IGNORED key='$key'"
        return
    }
    $Context.LastLogicalKeys[$Event.plan] = $key

    $current = $Context.States[$Event.plan]
    if ($Event.event -eq 'STARTED') {
        $summaryBaseline = Get-SummaryMtime $Context $Event.expected_summary
        $Context.States[$Event.plan] = [ordered]@{
            plan = $Event.plan
            execution_id = $Event.execution_id
            executor = $Event.executor
            last_event = 'STARTED'
            last_update = $Event.updated_at
            deadline_at = $null
            base_commit = $Event.base_commit
            last_seen_head = $Event.base_commit
            branch = $Event.branch
            expected_summary = $Event.expected_summary
            last_seen_summary_mtime = $summaryBaseline
            terminal = $false
            overdue_since = $null
            stall_reported = $false
            duplicate_stall_logged = $false
            long_deadline_logged = $false
        }
        Clear-OldNotification $Context $Event
        Write-WatchdogLog $Context "NEW_LIFECYCLE_ACCEPTED plan='$($Event.plan)' execution_id='$($Event.execution_id)'"
        return
    }

    if ($null -eq $current) {
        $terminalEvent = $Event.event -in @('COMPLETED', 'FAILED')
        $executor = [string](Get-EventProperty $Event 'executor')
        $baseCommit = [string](Get-EventProperty $Event 'base_commit')
        $branch = [string](Get-EventProperty $Event 'branch')
        $expectedSummary = [string](Get-EventProperty $Event 'expected_summary')
        if ([string]::IsNullOrWhiteSpace($expectedSummary) -and $terminalEvent) {
            $expectedSummary = [string](Get-EventProperty $Event 'summary')
        }
        $hasLifecycleMetadata = -not [string]::IsNullOrWhiteSpace($executor) -and
            -not [string]::IsNullOrWhiteSpace($baseCommit) -and
            -not [string]::IsNullOrWhiteSpace($branch) -and
            -not [string]::IsNullOrWhiteSpace($expectedSummary)

        if ($terminalEvent -or $hasLifecycleMetadata) {
            $summaryBaseline = if ([string]::IsNullOrWhiteSpace($expectedSummary)) {
                $null
            }
            else { Get-SummaryMtime $Context $expectedSummary }
            $current = [ordered]@{
                plan = $Event.plan
                execution_id = $Event.execution_id
                executor = $(if ([string]::IsNullOrWhiteSpace($executor)) { 'unknown' } else { $executor })
                last_event = 'STARTED'
                last_update = $Event.updated_at
                deadline_at = $null
                base_commit = $baseCommit
                last_seen_head = $baseCommit
                branch = $branch
                expected_summary = $expectedSummary
                last_seen_summary_mtime = $summaryBaseline
                terminal = $false
                overdue_since = $null
                stall_reported = $false
                duplicate_stall_logged = $false
                long_deadline_logged = $false
            }
            $Context.States[$Event.plan] = $current
            Write-WatchdogLog $Context "LIFECYCLE_RECOVERED plan='$($Event.plan)' event='$($Event.event)' execution_id='$($Event.execution_id)'"
        }
        else {
            Write-WatchdogLog $Context "ORPHAN_EVENT_IGNORED plan='$($Event.plan)' execution_id='$($Event.execution_id)'"
            return
        }
    }
    elseif ($current.execution_id -ne $Event.execution_id) {
        Write-WatchdogLog $Context "STALE_EVENT_IGNORED plan='$($Event.plan)' execution_id='$($Event.execution_id)'"
        return
    }
    if ($current.terminal) {
        Write-WatchdogLog $Context "TERMINAL_EVENT_IGNORED plan='$($Event.plan)' execution_id='$($Event.execution_id)'"
        return
    }

    switch ($Event.event) {
        'PROGRESS' {
            $current.last_event = 'PROGRESS'
            $current.last_update = $Event.updated_at
            $current.deadline_at = $null
            $current.overdue_since = $null
            $current.stall_reported = $false
            $current.duplicate_stall_logged = $false
            $current.long_deadline_logged = $false
            Clear-StallNotification $Context $current
        }
        'LONG_OPERATION' {
            $current.last_event = 'LONG_OPERATION'
            $current.last_update = $Event.updated_at
            $current.deadline_at = $Event.deadline_at
            $current.overdue_since = $null
            $current.stall_reported = $false
            $current.duplicate_stall_logged = $false
            $current.long_deadline_logged = $false
            Clear-StallNotification $Context $current
        }
        'COMPLETED' {
            $current.last_event = 'COMPLETED'
            $current.last_update = $Event.updated_at
            $current.terminal = $true
            Write-TerminalNotification $Context $Event 'PLAN_COMPLETED' $current.executor
        }
        'FAILED' {
            $current.last_event = 'FAILED'
            $current.last_update = $Event.updated_at
            $current.terminal = $true
            Write-TerminalNotification $Context $Event 'PLAN_FAILED' $current.executor
        }
        default {
            Write-WatchdogLog $Context "INVALID_EVENT type='$($Event.event)'"
            return
        }
    }
    Write-WatchdogLog $Context "EVENT_ACCEPTED plan='$($Event.plan)' event='$($Event.event)' execution_id='$($Event.execution_id)'"
}

function On-EventFileChanged {
    param([System.Collections.IDictionary]$Context, [string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { return }
    $plan = [IO.Path]::GetFileNameWithoutExtension($Path)
    $now = [DateTime]::UtcNow
    $last = $Context.LastFilesystemSignal[$plan]
    if ($null -ne $last) {
        $elapsed = ($now - $last).TotalMilliseconds
        if ($elapsed -lt $Context.DebounceMilliseconds) {
            Start-Sleep -Milliseconds ([int]($Context.DebounceMilliseconds - $elapsed))
        }
    }
    $Context.LastFilesystemSignal[$plan] = [DateTime]::UtcNow
    $event = Read-EventJson $Context $Path
    if ($null -ne $event) { Process-WatchdogEvent $Context $event }
}

function Start-Watchdog {
    param([System.Collections.IDictionary]$Context, [int]$RunForSeconds = 0)
    Write-WatchdogLog $Context 'WATCHDOG_STARTED'
    Write-WatchdogLog $Context "TIMER_STARTED interval_seconds='$($Context.ScanIntervalSeconds)'"
    $stopAt = if ($RunForSeconds -gt 0) { [DateTime]::UtcNow.AddSeconds($RunForSeconds) } else { $null }
    $nextScan = [DateTime]::UtcNow.AddSeconds($Context.ScanIntervalSeconds)
    while ($null -eq $stopAt -or [DateTime]::UtcNow -lt $stopAt) {
        # The event files are current-state snapshots. A short foreground tick
        # is deterministic across PowerShell hosts and naturally coalesces the
        # Created/Changed/Renamed patterns produced by atomic replacement.
        foreach ($file in @(Get-ChildItem -LiteralPath $Context.Paths.Events -Filter '*.json' -File -ErrorAction SilentlyContinue)) {
            $signature = '{0}|{1}' -f $file.Length, $file.LastWriteTimeUtc.Ticks
            if ($Context.SnapshotSignatures[$file.Name] -ne $signature) {
                $Context.SnapshotSignatures[$file.Name] = $signature
                On-EventFileChanged $Context $file.FullName
            }
        }
        if ([DateTime]::UtcNow -ge $nextScan) {
            Invoke-WatchdogScan -Context $Context -Now ([DateTime]::UtcNow)
            $nextScan = [DateTime]::UtcNow.AddSeconds($Context.ScanIntervalSeconds)
        }
        Start-Sleep -Milliseconds 250
    }
    return $Context
}

if ($MyInvocation.InvocationName -ne '.') {
    $context = New-WatchdogContext -DebounceMilliseconds $DebounceMilliseconds -SilenceTimeoutSeconds $SilenceTimeoutSeconds -GraceSeconds $GraceSeconds -ScanIntervalSeconds $ScanIntervalSeconds
    $watchdogMutex = [Threading.Mutex]::new($false, (Get-WatchdogMutexName $context.Paths.MutexScope))
    $watchdogMutexHeld = $false
    try {
        try { $watchdogMutexHeld = $watchdogMutex.WaitOne(0) }
        catch [Threading.AbandonedMutexException] { $watchdogMutexHeld = $true }
        if (-not $watchdogMutexHeld) { return }
        Start-Watchdog -Context $context -RunForSeconds $RunForSeconds | Out-Null
    }
    finally {
        if ($watchdogMutexHeld) { $watchdogMutex.ReleaseMutex() }
        $watchdogMutex.Dispose()
    }
}
