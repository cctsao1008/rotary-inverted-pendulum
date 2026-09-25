param(
    [string]$Uf2 = ""
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

if (-not $Uf2) {
    $Uf2 = Join-Path $repoRoot "build\rp2350a\rip_rp2350a.uf2"
}
elseif (-not [System.IO.Path]::IsPathRooted($Uf2)) {
    $Uf2 = Join-Path $repoRoot $Uf2
}

if (-not (Test-Path $Uf2)) {
    throw "UF2 not found: $Uf2`nRun .\tools\rp2350_build.ps1 first."
}

$volumes = @(Get-Volume -ErrorAction Stop | Where-Object {
    $_.FileSystemLabel -eq "RPI-RP2" -and $_.DriveLetter
})

if ($volumes.Count -eq 0) {
    throw "No RPI-RP2 drive found. Hold BOOTSEL while connecting/resetting the RP2350 board, then run this script again."
}

if ($volumes.Count -gt 1) {
    $letters = ($volumes | ForEach-Object { "$($_.DriveLetter):" }) -join ", "
    throw "Multiple RPI-RP2 drives found ($letters). Leave only the target board in BOOTSEL mode."
}

$drive = "$($volumes[0].DriveLetter):\"
Write-Host "Flashing $(Split-Path $Uf2 -Leaf) -> $drive"
Copy-Item -Path $Uf2 -Destination $drive -Force

Write-Host "UF2 copied. The board should reboot automatically and the RPI-RP2 drive should disappear."
