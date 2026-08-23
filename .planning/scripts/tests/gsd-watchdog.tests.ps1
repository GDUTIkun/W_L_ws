[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$env:GSD_WATCHDOG_DISABLE_AUTOSTART = '1'
$testRuntimeRoot = Join-Path ([IO.Path]::GetTempPath()) ('gsd-watchdog-unit-{0}' -f [guid]::NewGuid().ToString('N'))
$env:GSD_WATCHDOG_TEST_RUNTIME_ROOT = $testRuntimeRoot

. (Join-Path (Split-Path -Parent $PSScriptRoot) 'gsd-watchdog.ps1')
$script:Emitter = Join-Path (Split-Path -Parent $PSScriptRoot) 'gsd-event.ps1'

function Assert-That {
    param([bool]$Condition, [string]$Message)
    if (-not $Condition) { throw "ASSERTION FAILED: $Message" }
}

function Invoke-Emitter {
    param([hashtable]$Parameters)
    & $script:Emitter @Parameters | Out-Null
}

function New-StartedParameters([string]$Plan, [string]$Id) {
    @{
        Plan = $Plan; ExecutionId = $Id; Event = 'STARTED'; Executor = 'codex'
        BaseCommit = 'base123'; Branch = 'main'; ExpectedSummary = 'summary.md'
    }
}

function Get-EventPath([System.Collections.IDictionary]$Context, [string]$Plan) {
    Join-Path $Context.Paths.Events "$Plan.json"
}

function Send-And-Process([System.Collections.IDictionary]$Context, [hashtable]$Parameters) {
    Invoke-Emitter $Parameters
    On-EventFileChanged $Context (Get-EventPath $Context $Parameters.Plan)
}

function Get-Notification([System.Collections.IDictionary]$Context, [string]$Plan) {
    Get-Content -Raw -LiteralPath (Join-Path $Context.Paths.Notifications "$Plan.json") | ConvertFrom-Json
}

$paths = Get-WatchdogPaths
Remove-Item -LiteralPath (Split-Path -Parent $paths.Events) -Recurse -Force -ErrorAction SilentlyContinue
$context = New-WatchdogContext
$idA = 'aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa'
$idB = 'bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb'

# W1: normal lifecycle and completed mailbox.
Send-And-Process $context (New-StartedParameters 'w1' $idA)
Send-And-Process $context @{ Plan='w1'; ExecutionId=$idA; Event='PROGRESS'; Stage='verification'; Task='2/3' }
Send-And-Process $context @{ Plan='w1'; ExecutionId=$idA; Event='COMPLETED'; Commit='done123'; Summary='summary.md' }
$state = $context.States['w1']; $notification = Get-Notification $context 'w1'
Assert-That ($state.execution_id -eq $idA -and $state.terminal -and $state.last_event -eq 'COMPLETED') 'W1 terminal lifecycle'
Assert-That ($notification.type -eq 'PLAN_COMPLETED' -and $notification.execution_id -eq $idA) 'W1 completed notification'

# W2: failed mailbox.
Send-And-Process $context (New-StartedParameters 'w2' $idA)
Send-And-Process $context @{ Plan='w2'; ExecutionId=$idA; Event='FAILED'; Reason='test_failure' }
$state = $context.States['w2']; $notification = Get-Notification $context 'w2'
Assert-That ($state.terminal -and $notification.type -eq 'PLAN_FAILED' -and $notification.reason -eq 'test_failure') 'W2 failed notification'

# W3: long-operation deadline is copied unchanged.
Send-And-Process $context (New-StartedParameters 'w3' $idA)
Invoke-Emitter @{ Plan='w3'; ExecutionId=$idA; Event='LONG_OPERATION'; Operation='test'; DeadlineMinutes=2 }
$emitted = Get-Content -Raw -LiteralPath (Get-EventPath $context 'w3') | ConvertFrom-Json
On-EventFileChanged $context (Get-EventPath $context 'w3')
$state = $context.States['w3']
Assert-That ($state.last_event -eq 'LONG_OPERATION' -and $state.deadline_at -eq $emitted.deadline_at -and -not $state.terminal) 'W3 long operation state'

# W4: progress exits the long-operation state without any timeout behavior.
Send-And-Process $context @{ Plan='w3'; ExecutionId=$idA; Event='PROGRESS'; Task='done' }
$state = $context.States['w3']
Assert-That ($state.last_event -eq 'PROGRESS' -and $null -eq $state.deadline_at -and -not $state.terminal) 'W4 progress clears deadline'

# W5: a new STARTED replaces lifecycle state and clears an older mailbox.
Send-And-Process $context (New-StartedParameters 'w5' $idA)
Send-And-Process $context @{ Plan='w5'; ExecutionId=$idA; Event='FAILED'; Reason='retry' }
Assert-That ((Get-Notification $context 'w5').execution_id -eq $idA) 'W5 old notification exists'
Send-And-Process $context (New-StartedParameters 'w5' $idB)
Assert-That ($context.States['w5'].execution_id -eq $idB -and -not $context.States['w5'].terminal) 'W5 lifecycle takeover'
Assert-That (-not (Test-Path -LiteralPath (Join-Path $context.Paths.Notifications 'w5.json'))) 'W5 old notification cleared'

# W6: inject a stale event directly; the emitter remains untouched.
Send-And-Process $context (New-StartedParameters 'w6' $idB)
$stale = [pscustomobject]@{ plan='w6'; execution_id=$idA; event='FAILED'; updated_at=[DateTime]::UtcNow.ToString('o'); reason='stale' }
Process-WatchdogEvent $context $stale
Assert-That ($context.States['w6'].execution_id -eq $idB -and -not $context.States['w6'].terminal) 'W6 stale event leaves state intact'
Assert-That (-not (Test-Path -LiteralPath (Join-Path $context.Paths.Notifications 'w6.json'))) 'W6 stale event writes no notification'

# W7: Created/Changed/Renamed all reach the same handler. Repeating a terminal
# current-state JSON must process one logical key only.
Send-And-Process $context (New-StartedParameters 'w7' $idA)
Invoke-Emitter @{ Plan='w7'; ExecutionId=$idA; Event='COMPLETED' }
$eventPath = Get-EventPath $context 'w7'
On-EventFileChanged $context $eventPath # Created
On-EventFileChanged $context $eventPath # Changed
On-EventFileChanged $context $eventPath # Renamed
$terminalKey = $context.LastLogicalKeys['w7']
$terminalWriteCount = @(Select-String -LiteralPath $context.Paths.Log -Pattern "PLAN_COMPLETED_WRITTEN plan='w7'" -SimpleMatch).Count
Assert-That ($terminalKey -like "w7|$idA|COMPLETED|*" -and $terminalWriteCount -eq 1 -and $context.States['w7'].terminal) 'W7 duplicate logical event deduplication'

# W8: current-state overwrites may coalesce, but sequential latest events must
# converge on the latest observed progress without rollback or parse failures.
Send-And-Process $context (New-StartedParameters 'w8' $idA)
foreach ($task in @('1', '2', '3')) {
    Send-And-Process $context @{ Plan='w8'; ExecutionId=$idA; Event='PROGRESS'; Task=$task }
}
$state = $context.States['w8']
Assert-That ($state.execution_id -eq $idA -and $state.last_event -eq 'PROGRESS' -and -not $state.terminal) 'W8 rapid progress convergence'

# W9: a watchdog starting after STARTED was overwritten by COMPLETED recovers
# from the current-state snapshot and writes the durable terminal mailbox.
Invoke-Emitter (New-StartedParameters 'w9' $idA)
Invoke-Emitter @{ Plan='w9'; ExecutionId=$idA; Event='COMPLETED'; Commit='cold123'; Summary='summary.md' }
$cold = New-WatchdogContext
On-EventFileChanged $cold (Get-EventPath $cold 'w9')
$notification = Get-Notification $cold 'w9'
Assert-That ($cold.States['w9'].terminal -and $notification.type -eq 'PLAN_COMPLETED' -and $notification.execution_id -eq $idA) 'W9 cold-start terminal recovery'

# W10: legacy terminal snapshots without carried STARTED metadata remain
# recoverable so deployments can upgrade without losing completed mailboxes.
$legacy = New-WatchdogContext
Process-WatchdogEvent $legacy ([pscustomobject]@{ plan='w10'; execution_id=$idB; event='FAILED'; updated_at=[DateTime]::UtcNow.ToString('o'); reason='legacy_failure' })
$notification = Get-Notification $legacy 'w10'
Assert-That ($legacy.States['w10'].terminal -and $notification.type -eq 'PLAN_FAILED' -and $notification.executor -eq 'unknown') 'W10 legacy terminal recovery'

# Phase 2B uses an injected clock and Git evidence providers, so no test
# changes the user's branch, HEAD, worktree, or repository history.
function New-StallContext {
    $context = New-WatchdogContext -SilenceTimeoutSeconds 2 -GraceSeconds 1 -ScanIntervalSeconds 0.2 -GitBranchProvider { 'main' } -GitHeadProvider { 'baseX' }
    [void]$context.Add('Clock', @{ Now = [datetime]'2020-01-01T00:00:00Z' })
    return $context
}
function Start-StallLifecycle([System.Collections.IDictionary]$Context, [string]$Plan, [string]$Id, [string]$Base = 'baseX', [string]$Branch = 'main', [string]$Summary = '.planning/runtime/no-summary.md') {
    Send-And-Process $Context @{ Plan=$Plan; ExecutionId=$Id; Event='STARTED'; Executor='codex'; BaseCommit=$Base; Branch=$Branch; ExpectedSummary=$Summary }
    $Context.States[$Plan].last_update = $Context.Clock.Now.ToString('o')
}
function Advance-Scan([System.Collections.IDictionary]$Context, [double]$Seconds) {
    $Context.Clock.Now = $Context.Clock.Now.AddSeconds($Seconds)
    Invoke-WatchdogScan -Context $Context -Now $Context.Clock.Now
}

# S1: normal timeout plus grace reports a non-terminal suspected stall.
$stall = New-StallContext; Start-StallLifecycle $stall 's1' $idA
Advance-Scan -Context $stall -Seconds 3.1
$state = $stall.States['s1']; $notification = Get-Notification $stall 's1'
Assert-That ($notification.type -eq 'SUSPECTED_STALL' -and $notification.execution_id -eq $idA -and -not $state.terminal) 'S1 normal silence stall'

# S2: a legal long operation suppresses ordinary silence detection until its deadline + grace.
$stall = New-StallContext; Start-StallLifecycle $stall 's2' $idA
$long = [pscustomobject]@{ plan='s2'; execution_id=$idA; event='LONG_OPERATION'; updated_at=$stall.Clock.Now.ToString('o'); deadline_at=$stall.Clock.Now.AddSeconds(10).ToString('o') }
Process-WatchdogEvent $stall $long
Advance-Scan -Context $stall -Seconds 5
Assert-That (-not (Test-Path -LiteralPath (Join-Path $stall.Paths.Notifications 's2.json'))) 'S2 no stall inside legal deadline'
$progress = [pscustomobject]@{ plan='s2'; execution_id=$idA; event='PROGRESS'; updated_at=$stall.Clock.Now.ToString('o') }
Process-WatchdogEvent $stall $progress
Assert-That ($null -eq $stall.States['s2'].deadline_at -and $stall.States['s2'].last_event -eq 'PROGRESS') 'S2 progress restores normal observation'

# S3: HEAD delta is consumed once, never as a permanent healthy condition.
$stall = New-StallContext; Start-StallLifecycle $stall 's3' $idA 'baseX'
$stall.GitHeadProvider = { 'headY' }; Advance-Scan -Context $stall -Seconds 3.1
Assert-That ($stall.States['s3'].last_seen_head -eq 'headY' -and -not (Test-Path -LiteralPath (Join-Path $stall.Paths.Notifications 's3.json'))) 'S3 first HEAD delta is progress'
Advance-Scan -Context $stall -Seconds 3.1
Assert-That ((Get-Notification $stall 's3').type -eq 'SUSPECTED_STALL') 'S3 unchanged HEAD eventually stalls'

# S4: the explicitly supplied summary path also supplies one-time progress evidence.
$summaryRelative = '.planning/runtime/s4-summary.tmp'
$summaryPath = Join-Path $context.Paths.RepoRoot $summaryRelative
Remove-Item -LiteralPath $summaryPath -Force -ErrorAction SilentlyContinue
$stall = New-StallContext; Start-StallLifecycle $stall 's4' $idA 'baseX' 'main' $summaryRelative
[IO.File]::WriteAllText($summaryPath, 'new summary evidence')
Advance-Scan -Context $stall -Seconds 3.1
Assert-That ($null -ne $stall.States['s4'].last_seen_summary_mtime -and -not (Test-Path -LiteralPath (Join-Path $stall.Paths.Notifications 's4.json'))) 'S4 summary delta is progress'
Advance-Scan -Context $stall -Seconds 3.1
Assert-That ((Get-Notification $stall 's4').type -eq 'SUSPECTED_STALL') 'S4 unchanged summary eventually stalls'
Remove-Item -LiteralPath $summaryPath -Force

# S5: terminal lifecycles are permanently immune until a future STARTED takeover.
$stall = New-StallContext; Start-StallLifecycle $stall 's5-completed' $idA
Process-WatchdogEvent $stall ([pscustomobject]@{ plan='s5-completed'; execution_id=$idA; event='COMPLETED'; updated_at=$stall.Clock.Now.ToString('o') })
Start-StallLifecycle $stall 's5-failed' $idB
Process-WatchdogEvent $stall ([pscustomobject]@{ plan='s5-failed'; execution_id=$idB; event='FAILED'; updated_at=$stall.Clock.Now.ToString('o'); reason='test' })
Advance-Scan -Context $stall -Seconds 20
Assert-That ((Get-Notification $stall 's5-completed').type -eq 'PLAN_COMPLETED' -and (Get-Notification $stall 's5-failed').type -eq 'PLAN_FAILED') 'S5 terminal immunity'

# S6: branch mismatch disables HEAD evidence but records the mismatch on stall.
$stall = New-StallContext; Start-StallLifecycle $stall 's6' $idA 'baseX' 'expected'
$stall.GitBranchProvider = { 'other' }; $stall.GitHeadProvider = { 'headY' }
Advance-Scan -Context $stall -Seconds 3.1
$notification = Get-Notification $stall 's6'
Assert-That ($notification.type -eq 'SUSPECTED_STALL' -and $notification.branch_mismatch -and $notification.expected_branch -eq 'expected' -and $notification.current_branch -eq 'other') 'S6 branch mismatch evidence'

# S7: a live event clears a prior stall mailbox; terminal output later replaces it.
$stall = New-StallContext; Start-StallLifecycle $stall 's7' $idA
Advance-Scan -Context $stall -Seconds 3.1
Process-WatchdogEvent $stall ([pscustomobject]@{ plan='s7'; execution_id=$idA; event='PROGRESS'; updated_at=$stall.Clock.Now.ToString('o') })
Assert-That (-not $stall.States['s7'].terminal -and -not (Test-Path -LiteralPath (Join-Path $stall.Paths.Notifications 's7.json'))) 'S7 progress recovers from stall'
Advance-Scan -Context $stall -Seconds 3.1
Process-WatchdogEvent $stall ([pscustomobject]@{ plan='s7'; execution_id=$idA; event='COMPLETED'; updated_at=$stall.Clock.Now.ToString('o') })
Assert-That ((Get-Notification $stall 's7').type -eq 'PLAN_COMPLETED') 'S7 terminal replaces stall mailbox'

# S8: repeated scans after one report neither rewrite nor spam the stall mailbox.
$stall = New-StallContext; Start-StallLifecycle $stall 's8' $idA
Advance-Scan -Context $stall -Seconds 3.1
Advance-Scan -Context $stall -Seconds 1
Advance-Scan -Context $stall -Seconds 1
$writes = @(Select-String -LiteralPath $stall.Paths.Log -Pattern "SUSPECTED_STALL_WRITTEN plan='s8'" -SimpleMatch).Count
Assert-That ($writes -eq 1 -and $stall.States['s8'].stall_reported) 'S8 duplicate stall suppression'

Write-Host 'PASS: gsd-watchdog tests (W1-W10, S1-S8)'
Remove-Item -LiteralPath $testRuntimeRoot -Recurse -Force -ErrorAction SilentlyContinue
$env:GSD_WATCHDOG_TEST_RUNTIME_ROOT = $null
