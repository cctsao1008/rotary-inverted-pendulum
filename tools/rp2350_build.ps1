param(
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$targetDir = Join-Path $repoRoot "firmware\targets\rp2350a"
$buildDir = Join-Path $repoRoot "build\rp2350a"
$depsDir = Join-Path $repoRoot "_deps"
$defaultSdk = Join-Path $depsDir "pico-sdk"

function Require-Command([string]$name) {
    if (-not (Get-Command $name -ErrorAction SilentlyContinue)) {
        throw "Required command '$name' was not found in PATH."
    }
}

Require-Command "cmake"
Require-Command "ninja"
Require-Command "arm-none-eabi-gcc"

if (-not $env:PICO_SDK_PATH) {
    if (-not (Test-Path $defaultSdk)) {
        throw "PICO_SDK_PATH is not set and '$defaultSdk' does not exist. Clone Pico SDK 2.3.1 there or set PICO_SDK_PATH."
    }
    $env:PICO_SDK_PATH = (Resolve-Path $defaultSdk).Path
}
elseif (-not (Test-Path $env:PICO_SDK_PATH)) {
    throw "PICO_SDK_PATH does not exist: $env:PICO_SDK_PATH"
}

if (-not $env:picotool_DIR -or -not (Test-Path $env:picotool_DIR)) {
    $picotoolConfig = Get-ChildItem $depsDir -Recurse -Filter "picotoolConfig.cmake" -File -ErrorAction SilentlyContinue |
        Select-Object -First 1

    if (-not $picotoolConfig) {
        throw "Prebuilt picotool was not found under '$depsDir'. Extract the Raspberry Pi picotool Windows package there or set picotool_DIR."
    }

    $env:picotool_DIR = Split-Path $picotoolConfig.FullName -Parent
}

if ($Clean -and (Test-Path $buildDir)) {
    Write-Host "Cleaning $buildDir"
    Remove-Item -Recurse -Force $buildDir
}

Write-Host "PICO_SDK_PATH = $env:PICO_SDK_PATH"
Write-Host "picotool_DIR  = $env:picotool_DIR"
Write-Host "Configuring RP2350A firmware..."

& cmake `
    -S $targetDir `
    -B $buildDir `
    -G Ninja `
    -DCMAKE_BUILD_TYPE=Release `
    "-Dpicotool_DIR=$env:picotool_DIR"

if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed with exit code $LASTEXITCODE."
}

Write-Host "Building RP2350A firmware..."
& cmake --build $buildDir --parallel

if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE."
}

$uf2 = Join-Path $buildDir "rip_rp2350a.uf2"
$elf = Join-Path $buildDir "rip_rp2350a.elf"
$bin = Join-Path $buildDir "rip_rp2350a.bin"

foreach ($artifact in @($elf, $bin, $uf2)) {
    if (-not (Test-Path $artifact)) {
        throw "Expected build artifact was not produced: $artifact"
    }
}

Write-Host ""
Write-Host "Build complete:"
Get-Item $elf, $bin, $uf2 | Format-Table Name, Length, LastWriteTime -AutoSize
Write-Host "UF2: $uf2"
