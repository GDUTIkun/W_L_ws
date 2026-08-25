[CmdletBinding()]
param(
    [string]$ModelPath = "simulation/mujoco/model/wheel_leg.xml",
    [string]$ScenePath = "simulation/mujoco/model/scence.xml",
    [string]$OutputPath = "docs/workflow/phases/02-coordinate-interface-contract/evidence/mujoco_frame_manifest.json"
)

$ErrorActionPreference = "Stop"

function Get-Attributes {
    param([System.Xml.XmlElement]$Element)

    $result = [ordered]@{}
    foreach ($attribute in $Element.Attributes) {
        $result[$attribute.Name] = $attribute.Value
    }
    return $result
}

function Get-BodyRecords {
    param(
        [System.Xml.XmlElement]$Body,
        [string]$ParentName,
        [string]$ParentPath
    )

    $name = $Body.GetAttribute("name")
    $path = if ($ParentPath) { "$ParentPath/$name" } else { $name }
    $records = @([ordered]@{
        name = $name
        path = $path
        parent = $ParentName
        pose = [ordered]@{
            pos = if ($Body.HasAttribute("pos")) { $Body.GetAttribute("pos") } else { "0 0 0" }
            orientationAttribute = @("quat", "axisangle", "xyaxes", "zaxis", "euler") |
                Where-Object { $Body.HasAttribute($_) } |
                Select-Object -First 1
            orientationValue = @("quat", "axisangle", "xyaxes", "zaxis", "euler") |
                Where-Object { $Body.HasAttribute($_) } |
                ForEach-Object { $Body.GetAttribute($_) } |
                Select-Object -First 1
        }
        joints = @($Body.SelectNodes("./joint") | ForEach-Object {
            [ordered]@{
                name = $_.GetAttribute("name")
                type = if ($_.HasAttribute("type")) { $_.GetAttribute("type") } else { "hinge" }
                pos = if ($_.HasAttribute("pos")) { $_.GetAttribute("pos") } else { "0 0 0" }
                axis = if ($_.HasAttribute("axis")) { $_.GetAttribute("axis") } else { "0 0 1" }
                axisExpressionFrame = "local body frame"
                range = if ($_.HasAttribute("range")) { $_.GetAttribute("range") } else { $null }
            }
        })
        freeJoint = [bool]$Body.SelectSingleNode("./freejoint")
        sites = @($Body.SelectNodes("./site") | ForEach-Object {
            $site = $_
            [ordered]@{
                name = $site.GetAttribute("name")
                pos = if ($site.HasAttribute("pos")) { $site.GetAttribute("pos") } else { "0 0 0" }
                orientationAttribute = @("quat", "axisangle", "xyaxes", "zaxis", "euler") |
                    Where-Object { $site.HasAttribute($_) } |
                    Select-Object -First 1
                orientationValue = @("quat", "axisangle", "xyaxes", "zaxis", "euler") |
                    Where-Object { $site.HasAttribute($_) } |
                    ForEach-Object { $site.GetAttribute($_) } |
                    Select-Object -First 1
            }
        })
    })

    foreach ($child in $Body.SelectNodes("./body")) {
        $records += Get-BodyRecords -Body $child -ParentName $name -ParentPath $path
    }
    return $records
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$modelAbsolute = (Resolve-Path (Join-Path $repoRoot $ModelPath)).Path
$sceneAbsolute = (Resolve-Path (Join-Path $repoRoot $ScenePath)).Path
$outputAbsolute = Join-Path $repoRoot $OutputPath

[xml]$model = Get-Content -LiteralPath $modelAbsolute -Raw
[xml]$scene = Get-Content -LiteralPath $sceneAbsolute -Raw

$compiler = $model.mujoco.compiler
$bodies = @($model.mujoco.worldbody.body | ForEach-Object {
    Get-BodyRecords -Body $_ -ParentName "world" -ParentPath ""
})
$joints = @($bodies | ForEach-Object { $_.joints } | Where-Object { $_ })
$sites = @($bodies | ForEach-Object { $_.sites } | Where-Object { $_ })
$sensors = @($model.mujoco.sensor.ChildNodes | Where-Object { $_ -is [System.Xml.XmlElement] } | ForEach-Object {
    [ordered]@{
        type = $_.Name
        name = $_.GetAttribute("name")
        attributes = Get-Attributes -Element $_
    }
})
$equalities = @($model.mujoco.equality.ChildNodes | Where-Object { $_ -is [System.Xml.XmlElement] } | ForEach-Object {
    [ordered]@{
        type = $_.Name
        attributes = Get-Attributes -Element $_
    }
})

$allNames = @($bodies.name) + @($joints.name) + @($sites.name) + @($sensors.name)
$duplicates = @($allNames | Where-Object { $_ } | Group-Object | Where-Object { $_.Count -gt 1 } | ForEach-Object {
    [ordered]@{ name = $_.Name; count = $_.Count }
})

$manifest = [ordered]@{
    schemaVersion = 1
    generatedAt = (Get-Date).ToString("o")
    source = [ordered]@{
        modelPath = $ModelPath
        modelSha256 = (Get-FileHash -LiteralPath $modelAbsolute -Algorithm SHA256).Hash
        scenePath = $ScenePath
        sceneSha256 = (Get-FileHash -LiteralPath $sceneAbsolute -Algorithm SHA256).Hash
    }
    compiler = [ordered]@{
        angle = if ($compiler.angle) { [string]$compiler.angle } else { "degree (MJCF default)" }
        eulerSequence = if ($compiler.eulerseq) { [string]$compiler.eulerseq } else { "xyz (MJCF default, intrinsic)" }
        coordinate = "local (only supported MJCF mode)"
    }
    declaredOptions = [ordered]@{
        includedModelGravity = [string]$model.mujoco.option.gravity
        sceneGravity = [string]$scene.mujoco.option.gravity
        sceneTimestep = [string]$scene.mujoco.option.timestep
    }
    include = [ordered]@{
        file = [string]$scene.mujoco.include.file
        matchesAuditedModel = ([IO.Path]::GetFileName($modelAbsolute) -eq [string]$scene.mujoco.include.file)
    }
    bodies = $bodies
    joints = $joints
    sites = $sites
    sensors = $sensors
    equalities = $equalities
    integrity = [ordered]@{
        bodyCount = $bodies.Count
        jointCount = $joints.Count
        siteCount = $sites.Count
        sensorCount = $sensors.Count
        duplicatePublicNames = $duplicates
        baseFreeJointPresent = [bool]($bodies | Where-Object { $_.name -eq "base_body" -and $_.freeJoint })
        baseWeldedToWorld = [bool]($equalities | Where-Object {
            $_.type -eq "weld" -and $_.attributes.body1 -eq "base_body" -and $_.attributes.body2 -eq "world"
        })
        allJointAxesExplicit = (($joints | Where-Object { -not $_.axis }).Count -eq 0)
    }
    auditNotes = @(
        "This manifest records imported MJCF facts only; it does not approve the coordinate convention.",
        "Body positions and orientations are local to their parent body.",
        "Joint axes are expressed in the local body frame containing the joint.",
        "The base_frame site has no orientation attribute, so it inherits base_body orientation.",
        "Dynamic sensor meaning requires loading the compiled model; this static audit does not resolve duplicate option precedence across include boundaries."
    )
}

$outputDirectory = Split-Path -Parent $outputAbsolute
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
$manifest | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $outputAbsolute -Encoding utf8

Write-Host "MuJoCo frame manifest written to $outputAbsolute"
Write-Host "Bodies=$($bodies.Count), joints=$($joints.Count), sites=$($sites.Count), sensors=$($sensors.Count)"
Write-Host "Duplicate public names=$($duplicates.Count), base welded=$($manifest.integrity.baseWeldedToWorld)"
