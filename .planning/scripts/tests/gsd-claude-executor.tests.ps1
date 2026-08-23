[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$wrapper = Join-Path $repoRoot '.planning/gsd-claude-executor.ps1'
$lockRoot = Join-Path $repoRoot '.planning/runtime/locks'
[IO.Directory]::CreateDirectory($lockRoot) | Out-Null
$lockPath = Join-Path $lockRoot '98-01.lock'
$powerShell = if ($IsWindows) { 'powershell.exe' } else { 'pwsh' }

function Assert-That([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw "ASSERTION FAILED: $Message" }
}

$held = $null
try {
    $held = [IO.File]::Open($lockPath, [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
    $prompt = 'Execute .planning/phases/98-lock-test/98-01-PLAN.md.'
    $previousErrorAction = $ErrorActionPreference
    try {
        # The wrapper intentionally writes its rejection to stderr. Capture it
        # as assertion evidence instead of promoting the expected signal.
        $ErrorActionPreference = 'Continue'
        $output = $prompt | & $powerShell -NoProfile -File $wrapper 2>&1
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorAction
    }
    Assert-That ($exitCode -eq 3) 'concurrent wrapper must exit with code 3'
    Assert-That (($output | Out-String) -match 'PLAN_ALREADY_RUNNING plan=98-01') 'concurrent wrapper rejection signal'
}
finally {
    if ($null -ne $held) { $held.Dispose() }
}

# Lock files are durable names, not durable ownership. Once the owning process
# releases its OS handle, the same path must be immediately reusable.
$reopened = [IO.File]::Open($lockPath, [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
$reopened.Dispose()
Remove-Item -LiteralPath $lockPath -Force -ErrorAction SilentlyContinue

Write-Host 'PASS: gsd-claude-executor single-flight test (C1)'
