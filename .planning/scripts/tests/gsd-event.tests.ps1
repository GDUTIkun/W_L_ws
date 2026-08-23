[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$env:GSD_WATCHDOG_DISABLE_AUTOSTART = '1'

$script:Emitter = Join-Path (Split-Path -Parent $PSScriptRoot) 'gsd-event.ps1'
$script:PlanningRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$script:EventsRoot = Join-Path $script:PlanningRoot 'runtime/events'

function Invoke-Emitter {
    param(
        [string[]]$EmitterArgs
    )
    $parameters = @{}
    for ($index = 0; $index -lt $EmitterArgs.Count; $index += 2) {
        $parameters[$EmitterArgs[$index].TrimStart('-')] = $EmitterArgs[$index + 1]
    }
    & $script:Emitter @parameters 3>&1
}

function Assert-That {
    param([bool]$Condition, [string]$Message)
    if (-not $Condition) { throw "ASSERTION FAILED: $Message" }
}

function Get-State([string]$Plan) {
    Get-Content -Raw -LiteralPath (Join-Path $script:EventsRoot "$Plan.json") | ConvertFrom-Json
}

function Wait-ForFile([string]$Path) {
    $deadline = [DateTime]::UtcNow.AddSeconds(5)
    while (-not (Test-Path -LiteralPath $Path)) {
        if ([DateTime]::UtcNow -gt $deadline) { throw "Timed out waiting for mutex test signal '$Path'." }
        Start-Sleep -Milliseconds 25
    }
}

Remove-Item -LiteralPath $script:EventsRoot -Force -Recurse -ErrorAction SilentlyContinue
$idA = '11111111-1111-1111-1111-111111111111'
$idB = '22222222-2222-2222-2222-222222222222'
$idC = '33333333-3333-3333-3333-333333333333'

# E1-E6: lifecycle creation, ordinary updates, deadline, takeover, stale and no-active rejection.
Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idA,'-Event','STARTED','-Executor','codex','-BaseCommit','base123','-Branch','main','-ExpectedSummary','.planning/phases/06-x/06-01-SUMMARY.md') | Out-Null
$state = Get-State '06-01'
Assert-That ($state.execution_id -eq $idA -and $state.event -eq 'STARTED') 'E1 STARTED state'
Assert-That ($state.base_commit -eq 'base123' -and $state.branch -eq 'main') 'E1 frozen metadata'
Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idA,'-Event','PROGRESS','-Stage','verification','-Task','2/3','-Commit','abc123') | Out-Null
$state = Get-State '06-01'
Assert-That ($state.event -eq 'PROGRESS' -and $state.execution_id -eq $idA) 'E2 PROGRESS'
Assert-That ($state.executor -eq 'codex' -and $state.base_commit -eq 'base123' -and $state.branch -eq 'main' -and $state.expected_summary) 'E2 lifecycle metadata carried forward'
$before = [DateTime]::UtcNow
Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idA,'-Event','LONG_OPERATION','-Operation','matlab_simulation','-DeadlineMinutes','2') | Out-Null
$state = Get-State '06-01'; Assert-That (([DateTime]$state.deadline_at) -gt $before.AddMinutes(1)) 'E3 deadline'
Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idB,'-Event','STARTED','-Executor','codex','-BaseCommit','baseB','-Branch','main','-ExpectedSummary','summary-B.md') | Out-Null
$state = Get-State '06-01'; Assert-That ($state.execution_id -eq $idB) 'E4 lifecycle takeover'
$output = Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idA,'-Event','FAILED','-Reason','max_turns')
Assert-That (($output | Out-String) -match 'STALE_EXECUTION') 'E5 stale rejection signal'
Assert-That ((Get-State '06-01').execution_id -eq $idB) 'E5 stale rejection preserves state'
Remove-Item -LiteralPath (Join-Path $script:EventsRoot '06-01.json') -Force
$output = Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idC,'-Event','PROGRESS')
Assert-That (($output | Out-String) -match 'NO_ACTIVE_EXECUTION') 'E6 no-active rejection signal'
Assert-That (-not (Test-Path -LiteralPath (Join-Path $script:EventsRoot '06-01.json'))) 'E6 no state created'

# E7: a delayed old lifecycle holds the plan mutex; takeover runs afterward,
# leaving C current. A subsequent B event must be rejected.
$signal = Join-Path $script:EventsRoot 'e7-entered.signal'
Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idB,'-Event','STARTED','-Executor','codex','-BaseCommit','baseB','-Branch','main','-ExpectedSummary','summary-B.md') | Out-Null
$job = Start-Job -ScriptBlock {
    param($Emitter, $Id, $Signal)
    $env:GSD_EVENT_TEST_HOLD_MUTEX_MS = '700'
    $env:GSD_EVENT_TEST_MUTEX_ENTER_SIGNAL = $Signal
    & $Emitter -Plan '06-01' -ExecutionId $Id -Event PROGRESS -Stage 'held' 3>&1
} -ArgumentList $script:Emitter, $idB, $signal
Wait-ForFile $signal
Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idC,'-Event','STARTED','-Executor','codex','-BaseCommit','baseC','-Branch','main','-ExpectedSummary','summary-C.md') | Out-Null
Receive-Job -Job $job -Wait | Out-Null
Remove-Job -Job $job -Force
$state = Get-State '06-01'; Assert-That ($state.execution_id -eq $idC) 'E7 takeover wins after serialized old progress'
$output = Invoke-Emitter -EmitterArgs @('-Plan','06-01','-ExecutionId',$idB,'-Event','FAILED')
Assert-That (($output | Out-String) -match 'STALE_EXECUTION') 'E7 old lifecycle stays stale'

# E8: separate plan mutexes allow an independent emitter to complete while
# 06-01 is intentionally holding its own mutex.
$signal = Join-Path $script:EventsRoot 'e8-entered.signal'
$job = Start-Job -ScriptBlock {
    param($Emitter, $Id, $Signal)
    $env:GSD_EVENT_TEST_HOLD_MUTEX_MS = '1000'
    $env:GSD_EVENT_TEST_MUTEX_ENTER_SIGNAL = $Signal
    & $Emitter -Plan '06-01' -ExecutionId $Id -Event PROGRESS 3>&1
} -ArgumentList $script:Emitter, $idC, $signal
Wait-ForFile $signal
$stopwatch = [Diagnostics.Stopwatch]::StartNew()
Invoke-Emitter -EmitterArgs @('-Plan','06-02','-ExecutionId',$idA,'-Event','STARTED','-Executor','codex','-BaseCommit','baseA','-Branch','main','-ExpectedSummary','summary-A.md') | Out-Null
$stopwatch.Stop()
Receive-Job -Job $job -Wait | Out-Null
Remove-Job -Job $job -Force
Assert-That ($stopwatch.ElapsedMilliseconds -lt 700) 'E8 different plans must not serialize on one mutex'
Assert-That ((Get-State '06-02').execution_id -eq $idA) 'E8 independent plan state'

Write-Host 'PASS: gsd-event emitter tests (E1-E8)'
