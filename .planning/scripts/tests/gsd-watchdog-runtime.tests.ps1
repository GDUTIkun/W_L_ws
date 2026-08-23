[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$env:GSD_WATCHDOG_DISABLE_AUTOSTART = '1'
$testRuntimeRoot = Join-Path ([IO.Path]::GetTempPath()) ('gsd-watchdog-runtime-{0}' -f [guid]::NewGuid().ToString('N'))
$env:GSD_WATCHDOG_TEST_RUNTIME_ROOT = $testRuntimeRoot

$script:RepoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$script:Watchdog = Join-Path $script:RepoRoot '.planning/scripts/gsd-watchdog.ps1'
$script:Emitter = Join-Path $script:RepoRoot '.planning/scripts/gsd-event.ps1'
$script:Runtime = $testRuntimeRoot
$script:Branch = (git -C $script:RepoRoot branch --show-current).Trim()
$script:Head = (git -C $script:RepoRoot rev-parse --verify HEAD 2>$null | Out-String).Trim()
if ([string]::IsNullOrWhiteSpace($script:Head)) { $script:Head = 'NO_COMMIT' }

function Assert-That([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw "ASSERTION FAILED: $Message" }
}

function Start-TestWatchdog([int]$RunForSeconds) {
    Start-Job -ScriptBlock {
        param($Root, $Seconds)
        Set-Location $Root
        & '.planning/scripts/gsd-watchdog.ps1' -RunForSeconds $Seconds -SilenceTimeoutSeconds 2 -GraceSeconds 1 -ScanIntervalSeconds 1
    } -ArgumentList $script:RepoRoot, $RunForSeconds
}

function Emit-Started([string]$Plan, [string]$Id) {
    & $script:Emitter -Plan $Plan -ExecutionId $Id -Event STARTED -Executor codex -BaseCommit $script:Head -Branch $script:Branch -ExpectedSummary ".planning/runtime/$Plan-missing-summary.md"
}

function Wait-ForPath([string]$Path, [double]$TimeoutSeconds) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        if (Test-Path -LiteralPath $Path) { return $true }
        Start-Sleep -Milliseconds 200
    }
    return $false
}

function Stop-TestWatchdog($Job) {
    if ($null -eq $Job) { return }
    if ($Job.State -eq 'Running') { Stop-Job -Job $Job }
    Receive-Job -Job $Job -ErrorAction SilentlyContinue | Out-Null
    Remove-Job -Job $Job -Force
}

function Remove-Fixture([string]$Plan) {
    foreach ($path in @(
        (Join-Path $script:Runtime "events/$Plan.json"),
        (Join-Path $script:Runtime "notifications/$Plan.json"),
        (Join-Path $script:Runtime "$Plan-missing-summary.md")
    )) { Remove-Item -LiteralPath $path -Force -ErrorAction SilentlyContinue }
}

# R1: independent process performs a wall-clock scan without any new event.
$plan = 'runtime-r1'; $id = '21111111-1111-1111-1111-111111111111'; $job = $null
Remove-Fixture $plan
try {
    $job = Start-TestWatchdog 9
    Start-Sleep -Seconds 1
    Emit-Started $plan $id
    $notificationPath = Join-Path $script:Runtime "notifications/$plan.json"
    Assert-That (Wait-ForPath $notificationPath 7) 'R1 stall mailbox was not produced by runtime scheduler'
    $notification = Get-Content -Raw -LiteralPath $notificationPath | ConvertFrom-Json
    Assert-That ($notification.type -eq 'SUSPECTED_STALL' -and $notification.execution_id -eq $id) 'R1 stall notification identity'
}
finally { Stop-TestWatchdog $job; Remove-Fixture $plan }

# R2: LONG_OPERATION deadline supersedes the ordinary silence window.
$plan = 'runtime-r2'; $id = '22222222-2222-2222-2222-222222222222'; $job = $null
Remove-Fixture $plan
try {
    $job = Start-TestWatchdog 7
    Start-Sleep -Seconds 1
    Emit-Started $plan $id
    Start-Sleep -Milliseconds 300
    & $script:Emitter -Plan $plan -ExecutionId $id -Event LONG_OPERATION -Operation runtime_test -DeadlineMinutes 1
    Start-Sleep -Seconds 4
    Assert-That (-not (Test-Path -LiteralPath (Join-Path $script:Runtime "notifications/$plan.json"))) 'R2 legal long operation stalled early'
}
finally { Stop-TestWatchdog $job; Remove-Fixture $plan }

# R3: PROGRESS resets the real observation window, then later silence stalls.
$plan = 'runtime-r3'; $id = '23333333-3333-3333-3333-333333333333'; $job = $null
Remove-Fixture $plan
try {
    $job = Start-TestWatchdog 11
    Start-Sleep -Seconds 1
    Emit-Started $plan $id
    Start-Sleep -Milliseconds 1500
    & $script:Emitter -Plan $plan -ExecutionId $id -Event PROGRESS -Stage runtime_reset -Task 1/1
    $notificationPath = Join-Path $script:Runtime "notifications/$plan.json"
    Start-Sleep -Seconds 2
    Assert-That (-not (Test-Path -LiteralPath $notificationPath)) 'R3 stalled before renewed timeout + grace elapsed'
    Assert-That (Wait-ForPath $notificationPath 5) 'R3 did not stall after renewed observation window'
    $notification = Get-Content -Raw -LiteralPath $notificationPath | ConvertFrom-Json
    Assert-That ($notification.type -eq 'SUSPECTED_STALL' -and $notification.last_event -eq 'PROGRESS') 'R3 timer reset evidence'
}
finally { Stop-TestWatchdog $job; Remove-Fixture $plan }

# R4: terminal-only current state is recoverable when the independent watchdog
# starts after the executor has already completed.
$plan = 'runtime-r4'; $id = '24444444-4444-4444-4444-444444444444'; $job = $null
Remove-Fixture $plan
try {
    Emit-Started $plan $id
    & $script:Emitter -Plan $plan -ExecutionId $id -Event COMPLETED -Commit $script:Head -Summary ".planning/runtime/$plan-summary.md"
    $job = Start-TestWatchdog 4
    $notificationPath = Join-Path $script:Runtime "notifications/$plan.json"
    Assert-That (Wait-ForPath $notificationPath 3) 'R4 cold-start terminal mailbox was not produced'
    $notification = Get-Content -Raw -LiteralPath $notificationPath | ConvertFrom-Json
    Assert-That ($notification.type -eq 'PLAN_COMPLETED' -and $notification.execution_id -eq $id) 'R4 cold-start terminal identity'
}
finally { Stop-TestWatchdog $job; Remove-Fixture $plan }

Write-Host 'PASS: gsd-watchdog runtime tests (R1-R4)'
Remove-Item -LiteralPath $testRuntimeRoot -Recurse -Force -ErrorAction SilentlyContinue
$env:GSD_WATCHDOG_TEST_RUNTIME_ROOT = $null
