param(
    [switch]$Build,
    [string]$Image = "",
    [int]$BootTimeoutSeconds = 8,
    [int]$AppTimeoutSeconds = 10
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$depsDir = Join-Path $repoRoot "_deps"
$commissionCli = Join-Path $repoRoot "tools\rp2350_commission\rp2350_commission.py"
$buildScript = Join-Path $repoRoot "tools\rp2350_build.ps1"

function Require-Command([string]$name) {
    $command = Get-Command $name -ErrorAction SilentlyContinue
    if (-not $command) {
        throw "Required command '$name' was not found in PATH."
    }
    return $command.Source
}

function Find-Picotool {
    $candidates = @()

    if ($env:picotool_DIR -and (Test-Path $env:picotool_DIR)) {
        $candidates += Get-ChildItem $env:picotool_DIR -Filter "picotool.exe" -File -ErrorAction SilentlyContinue
    }

    if (Test-Path $depsDir) {
        $candidates += Get-ChildItem $depsDir -Recurse -Filter "picotool.exe" -File -ErrorAction SilentlyContinue
    }

    $candidate = $candidates | Select-Object -First 1
    if (-not $candidate) {
        throw "picotool.exe was not found under '$depsDir' or picotool_DIR."
    }
    return $candidate.FullName
}

function Invoke-NativeProbe {
    param(
        [Parameter(Mandatory = $true)][string]$FilePath,
        [string[]]$ArgumentList = @()
    )

    # Windows PowerShell promotes redirected native stderr to ErrorRecord objects.
    # During USB re-enumeration a missing device is expected, so polling must not
    # abort merely because the probe writes a transient diagnostic to stderr.
    $previousErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $output = @(& $FilePath @ArgumentList 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }

    return [pscustomobject]@{
        ExitCode = $exitCode
        Output   = $output
    }
}

$python = Require-Command "python"
$picotool = Find-Picotool

if ($Build) {
    Write-Host "[1/6] Build firmware"
    & $buildScript
}
else {
    Write-Host "[1/6] Use existing build"
}

if (-not $Image) {
    $Image = Join-Path $repoRoot "build\rp2350a\rip_rp2350a.elf"
}
elseif (-not [System.IO.Path]::IsPathRooted($Image)) {
    $Image = Join-Path $repoRoot $Image
}

if (-not (Test-Path $Image)) {
    throw "Firmware image not found: $Image`nRun .\tools\rp2350_build.ps1 first or use -Build."
}

Write-Host "[2/6] Request safe bootloader handoff over HID"
& $python $commissionCli bootloader
if ($LASTEXITCODE -ne 0) {
    throw "The running firmware did not accept ENTER_USB_BOOTLOADER. Bootstrap this feature once with the BOOTSEL/UF2 recovery path."
}

Write-Host "[3/6] Wait for RP2350 ROM PICOBOOT"
$bootDeadline = [DateTime]::UtcNow.AddSeconds($BootTimeoutSeconds)
$bootReady = $false
while ([DateTime]::UtcNow -lt $bootDeadline) {
    $probe = Invoke-NativeProbe -FilePath $picotool -ArgumentList @("info")
    if ($probe.ExitCode -eq 0) {
        $bootReady = $true
        break
    }
    Start-Sleep -Milliseconds 200
}
if (-not $bootReady) {
    throw "RP2350 PICOBOOT did not appear within $BootTimeoutSeconds seconds."
}

Write-Host "[4/6] Program changed flash sectors and verify"
& $picotool load -u -v -x $Image
if ($LASTEXITCODE -ne 0) {
    throw "picotool load failed with exit code $LASTEXITCODE. The ROM bootloader remains the recovery path; rerun this script or use BOOTSEL if needed."
}

Write-Host "[5/6] Wait for application HID/CDC"
$appDeadline = [DateTime]::UtcNow.AddSeconds($AppTimeoutSeconds)
$appReady = $false
$statusOutput = @()
while ([DateTime]::UtcNow -lt $appDeadline) {
    $probe = Invoke-NativeProbe -FilePath $python -ArgumentList @($commissionCli, "status")
    if ($probe.ExitCode -eq 0) {
        $statusOutput = @($probe.Output)
        $appReady = $true
        break
    }
    Start-Sleep -Milliseconds 250
}
if (-not $appReady) {
    throw "Application did not re-enumerate within $AppTimeoutSeconds seconds after programming."
}

Write-Host "[6/6] Firmware update PASS"
Write-Host "Image: $Image"
$statusOutput | ForEach-Object { Write-Host $_ }
