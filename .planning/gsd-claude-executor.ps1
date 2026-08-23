$ErrorActionPreference = "Stop"

# Accept both GSD's process-level stdin pipe and an in-process PowerShell
# pipeline. `[Console]::In.ReadToEnd()` alone blocks forever for the latter
# when its parent owns an open console stdin, so Claude is never launched.
$pipelinePrompt = @($input)
if ($pipelinePrompt.Count -gt 0) {
    $prompt = [string]::Join([Environment]::NewLine, [string[]]$pipelinePrompt)
    $promptSource = "pipeline"
}
else {
    $prompt = [Console]::In.ReadToEnd()
    $promptSource = "stdin"
}

if ([string]::IsNullOrWhiteSpace($prompt)) {
    Write-Error "No GSD execution prompt received."
    exit 2
}

$planMatch = [regex]::Match(
    $prompt,
    '(?im)(?<path>\.planning[/\\]phases[/\\][^\r\n<>"'']*?(?<plan>\d{2,}-\d{2,})-PLAN\.md)'
)
if (-not $planMatch.Success) {
    Write-Error "Could not determine PLAN path/id from GSD execution prompt."
    exit 2
}

$planId = $planMatch.Groups['plan'].Value
$planPath = $planMatch.Groups['path'].Value -replace '[\\/]', [IO.Path]::DirectorySeparatorChar
$expectedSummary = $planPath -replace '-PLAN\.md$', '-SUMMARY.md'
$lockRoot = Join-Path $PSScriptRoot (Join-Path 'runtime' 'locks')
[IO.Directory]::CreateDirectory($lockRoot) | Out-Null
$lockPath = Join-Path $lockRoot "$planId.lock"
try {
    $script:gsdPlanLock = [IO.File]::Open(
        $lockPath,
        [IO.FileMode]::OpenOrCreate,
        [IO.FileAccess]::ReadWrite,
        [IO.FileShare]::None
    )
}
catch [IO.IOException] {
    [Console]::Error.WriteLine("[Cross-AI] ERROR: PLAN_ALREADY_RUNNING plan=$planId")
    exit 3
}

function Exit-CrossAiWrapper {
    param([int]$Code)
    if ($null -ne $script:gsdPlanLock) {
        $script:gsdPlanLock.Dispose()
        $script:gsdPlanLock = $null
    }
    exit $Code
}

$executionId = [guid]::NewGuid().ToString()
$eventScript = Join-Path $PSScriptRoot (Join-Path 'scripts' 'gsd-event.ps1')
$baseCommit = (git rev-parse --verify HEAD 2>$null | Out-String).Trim()
if ([string]::IsNullOrWhiteSpace($baseCommit)) { $baseCommit = 'NO_COMMIT' }
$branch = (git branch --show-current).Trim()

function Publish-GsdEvent {
    param([string]$Event, [hashtable]$Fields = @{})
    & $eventScript -Plan $planId -ExecutionId $executionId -Event $Event @Fields
}

Publish-GsdEvent 'STARTED' @{
    Executor = 'claude'
    BaseCommit = $baseCommit
    Branch = $branch
    ExpectedSummary = $expectedSummary
}

# Local execution tier.
# Execution tier routing.
# cc-tier.local:
#   auto       -> use EXECUTION_TIER from PLAN
#   flash-high -> force Flash High
#   flash-max  -> force Flash Max
#   pro-max    -> force Pro Max

$tierFile = Join-Path $PSScriptRoot "cc-tier.local"
$localTier = "auto"

if (Test-Path $tierFile) {
    $localTier = (Get-Content $tierFile -Raw).Trim().ToLower()
}

# Read execution tier from the GSD PLAN prompt.
$tierMatch = [regex]::Match(
    $prompt,
    '(?im)^\s*EXECUTION_TIER\s*:\s*(flash-high|flash-max|pro-max)\s*$'
)

if ($tierMatch.Success) {
    $planTier = $tierMatch.Groups[1].Value.ToLower()
}
else {
    $planTier = $null
}

# Priority:
# 1. Explicit local override
# 2. PLAN routing tier
# 3. Safe default
if ($localTier -in @("flash-high", "flash-max", "pro-max")) {
    $tier = $localTier
    $tierSource = "local-override"
}
elseif ($planTier) {
    $tier = $planTier
    $tierSource = "plan"
}
else {
    $tier = "flash-high"
    $tierSource = "default"
}

[Console]::Error.WriteLine("[Cross-AI] tier=$tier source=$tierSource prompt_source=$promptSource prompt_chars=$($prompt.Length)")

# Tier policy owns reasoning effort and turn budget only. The actual model
# name is a provider concern (below) — tiers are logical, not model names.
switch ($tier) {
    "flash-high" {
        $effort = "high"
        $maxTurns = 32
    }

    "flash-max" {
        $effort = "max"
        $maxTurns = 64
    }

    "pro-max" {
        $effort = "max"
        $maxTurns = 64
    }

    default {
        Write-Error "Unknown GSD Claude tier: $tier"
        Exit-CrossAiWrapper 3
    }
}

# Project-local provider (decoupled from the global Claude Code config).
# cc-provider.local -> active provider name; cc-providers.psd1 -> registry.
# There is deliberately no fallback to the global ANTHROPIC_* environment.
$providerFile = Join-Path $PSScriptRoot "cc-provider.local"
$providersFile = Join-Path $PSScriptRoot "cc-providers.psd1"

function Fail-CrossAiProvider {
    param([string]$Message)
    [Console]::Error.WriteLine("[Cross-AI] ERROR: $Message")
    Exit-CrossAiWrapper 4
}

if (-not (Test-Path $providerFile)) {
    Fail-CrossAiProvider "cc-provider.local missing: $providerFile"
}
$providerName = (Get-Content $providerFile -Raw).Trim()
if ([string]::IsNullOrWhiteSpace($providerName)) {
    Fail-CrossAiProvider "cc-provider.local empty: $providerFile"
}

if (-not (Test-Path $providersFile)) {
    Fail-CrossAiProvider "cc-providers.psd1 missing: $providersFile"
}
try {
    $providers = Import-PowerShellDataFile -Path $providersFile
}
catch {
    Fail-CrossAiProvider "invalid provider config: cannot parse $providersFile : $($_.Exception.Message)"
}
if ($null -eq $providers -or -not $providers.ContainsKey($providerName)) {
    Fail-CrossAiProvider "unknown provider '$providerName' (not defined in $providersFile)"
}
$provider = $providers[$providerName]

$baseUrl = $provider.BaseUrl
$authToken = $provider.AuthToken
$providerModels = $provider.Models

if ([string]::IsNullOrWhiteSpace($baseUrl)) {
    Fail-CrossAiProvider "invalid provider config: provider '$providerName' has empty BaseUrl"
}
if ([string]::IsNullOrWhiteSpace($authToken)) {
    Fail-CrossAiProvider "invalid provider config: provider '$providerName' has empty AuthToken"
}
if ($null -eq $providerModels) {
    Fail-CrossAiProvider "invalid provider config: provider '$providerName' has no Models table"
}
if (-not $providerModels.ContainsKey($tier)) {
    Fail-CrossAiProvider "Provider '$providerName' has no model mapping for tier '$tier'"
}
$model = $providerModels[$tier]
if ([string]::IsNullOrWhiteSpace($model)) {
    Fail-CrossAiProvider "Provider '$providerName' has empty model mapping for tier '$tier'"
}

# A positive, process-scoped override exists solely for controlled diagnostics.
# Normal GSD execution never sets it and keeps the tier budget above.
if ($env:CROSS_AI_MAX_TURNS_OVERRIDE -and $env:CROSS_AI_MAX_TURNS_OVERRIDE -match '^\d+$' -and [int]$env:CROSS_AI_MAX_TURNS_OVERRIDE -gt 0) {
    $maxTurns = [int]$env:CROSS_AI_MAX_TURNS_OVERRIDE
    $maxTurnsSource = "diagnostic-override"
}
else {
    $maxTurnsSource = "tier-default"
}

# Internal Claude Code subagents follow the selected provider model.
$env:CLAUDE_CODE_SUBAGENT_MODEL = $model

# Do not let claude -p kill background subagents/workflows after
# its default 10-minute wait ceiling.
# GSD's cross_ai_timeout remains the outer hard timeout.
$env:CLAUDE_CODE_PRINT_BG_WAIT_CEILING_MS = "0"

# Explicit reasoning effort.
$env:CLAUDE_CODE_EFFORT_LEVEL = $effort

# Project-local provider, scoped to this wrapper process and its `claude`
# child only. The global ANTHROPIC_* environment is never read as a fallback
# and never modified (User/Machine are untouched).
$env:ANTHROPIC_BASE_URL = $baseUrl
$env:ANTHROPIC_AUTH_TOKEN = $authToken
Remove-Item Env:ANTHROPIC_API_KEY -ErrorAction SilentlyContinue

[Console]::Error.WriteLine("[Cross-AI] provider=$providerName base_url=$baseUrl auth_token_present=$([bool]$authToken) tier=$tier model=$model effort=$effort max_turns=$maxTurns max_turns_source=$maxTurnsSource")

# Test-only hook: stop after resolving tier + provider so mapping tests stay
# offline and never call the API. Never set in normal GSD execution.
if ($env:CROSS_AI_DRY_RUN -eq '1') {
    [Console]::Error.WriteLine("[Cross-AI] DRY_RUN complete")
    Exit-CrossAiWrapper 0
}

$executionRules = @"
You are the execution agent in a GSD workflow.

Follow the supplied PLAN exactly.
Do not broaden scope.
Do not redesign architecture unless the PLAN explicitly requires it.
Do not invent successful results.

If execution, testing, simulation, or verification fails, report the failure clearly.

Do not run destructive git commands such as:
- git reset --hard
- git clean -fd
- git push --force

Do not overwrite experimental raw data unless the PLAN explicitly requires it.
Do not rerun long simulations merely because existing results are inconvenient.
Prefer inspecting existing outputs first when appropriate.

Perform the implementation and verification required by the supplied PLAN.

Return a concise execution result compatible with GSD SUMMARY.md.

Watchdog event protocol for this execution:
- plan: $planId
- execution_id: $executionId
- emitter: $eventScript
- You may publish only PROGRESS and LONG_OPERATION.
- Publish PROGRESS at task/verification/verified-commit boundaries.
- Publish LONG_OPERATION before a bounded long MATLAB/Simulink operation.
- Never publish STARTED, COMPLETED, or FAILED; the wrapper owns terminal events.
- Do not publish periodic heartbeats.
"@

# Use JSON internally so GSD can distinguish normal completion, turn-budget
# exhaustion, and execution errors. Stdout remains only the model result.
[Console]::Error.WriteLine("[Cross-AI] launching claude")
$startedAt = [DateTime]::UtcNow
$jsonResult = $prompt | & claude -p `
    --model $model `
    --effort $effort `
    --output-format json `
    --permission-mode acceptEdits `
    --allowedTools "Read,Edit,Write,Glob,Grep,Bash" `
    --max-turns $maxTurns `
    --append-system-prompt $executionRules

$claudeExit = $LASTEXITCODE
$elapsedSeconds = [Math]::Round(([DateTime]::UtcNow - $startedAt).TotalSeconds, 2)

try {
    $result = $jsonResult | ConvertFrom-Json -ErrorAction Stop
}
catch {
    [Console]::Error.WriteLine("[Cross-AI] claude_exit=$claudeExit elapsed_sec=$elapsedSeconds result_parse=failed")
    [Console]::Error.WriteLine("[Cross-AI] ERROR: Claude returned non-JSON output")
    Publish-GsdEvent 'FAILED' @{ Reason = 'result_parse_failed' }
    if ($jsonResult) { [Console]::Out.Write($jsonResult) }
    Exit-CrossAiWrapper $(if ($claudeExit -eq 0) { 1 } else { $claudeExit })
}

$turns = if ($null -ne $result.num_turns) { $result.num_turns } else { "unknown" }
$terminalReason = if ($result.terminal_reason) { $result.terminal_reason } else { "unknown" }
$subtype = if ($result.subtype) { $result.subtype } else { "unknown" }
$isError = [bool]$result.is_error
$sessionId = if ($result.session_id) { $result.session_id } else { "unknown" }
[Console]::Error.WriteLine("[Cross-AI] claude_exit=$claudeExit elapsed_sec=$elapsedSeconds turns=$turns is_error=$isError terminal_reason=$terminalReason subtype=$subtype session_id=$sessionId")

if ($terminalReason -eq "max_turns" -or $subtype -eq "error_max_turns") {
    [Console]::Error.WriteLine("[Cross-AI] ERROR: max turns reached before task completion")
}

if ($result.result) { [Console]::Out.Write([string]$result.result) }

if ($terminalReason -eq 'max_turns' -or $subtype -eq 'error_max_turns') {
    Publish-GsdEvent 'FAILED' @{ Reason = 'max_turns' }
    Exit-CrossAiWrapper $(if ($claudeExit -eq 0) { 1 } else { $claudeExit })
}
if ($claudeExit -ne 0 -or $isError) {
    $failureReason = if ($terminalReason -and $terminalReason -ne 'unknown') { $terminalReason } else { 'executor_error' }
    Publish-GsdEvent 'FAILED' @{ Reason = $failureReason }
    Exit-CrossAiWrapper $(if ($claudeExit -eq 0) { 1 } else { $claudeExit })
}
$finalCommit = (git rev-parse --verify HEAD 2>$null | Out-String).Trim()
if ([string]::IsNullOrWhiteSpace($finalCommit)) { $finalCommit = 'NO_COMMIT' }
Publish-GsdEvent 'COMPLETED' @{ Commit = $finalCommit; Summary = $expectedSummary }
Exit-CrossAiWrapper 0
