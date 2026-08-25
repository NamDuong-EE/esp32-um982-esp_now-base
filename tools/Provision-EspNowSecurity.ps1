[CmdletBinding()]
param(
    [string]$RoverProjectPath,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

$baseProjectPath = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($RoverProjectPath)) {
    $workspaceParent = Split-Path -Parent $baseProjectPath
    $RoverProjectPath = Join-Path $workspaceParent 'esp32-um982-lora'
}

$baseHeaderPath = Join-Path $baseProjectPath 'include\EspNow_Secrets.h'
$roverHeaderPath = Join-Path $RoverProjectPath 'include\EspNow_Secrets.h'
$targetPaths = @($baseHeaderPath, $roverHeaderPath)

foreach ($targetPath in $targetPaths) {
    $includePath = Split-Path -Parent $targetPath
    if (-not (Test-Path -LiteralPath $includePath -PathType Container)) {
        throw "Project include directory not found: $includePath"
    }
    if ((Test-Path -LiteralPath $targetPath) -and -not $Force) {
        throw "Secret file already exists: $targetPath. Use -Force only when intentionally rotating both devices."
    }
}

function New-RandomKey {
    $key = New-Object byte[] 16
    $generator = [System.Security.Cryptography.RandomNumberGenerator]::Create()
    try {
        $generator.GetBytes($key)
    }
    finally {
        $generator.Dispose()
    }
    return $key
}

function Format-KeyBytes([byte[]]$Key) {
    $formatted = $Key | ForEach-Object { '0x{0:X2}' -f $_ }
    return $formatted -join ', '
}

function Test-KeysEqual([byte[]]$Left, [byte[]]$Right) {
    if ($Left.Length -ne $Right.Length) {
        return $false
    }
    for ($index = 0; $index -lt $Left.Length; ++$index) {
        if ($Left[$index] -ne $Right[$index]) {
            return $false
        }
    }
    return $true
}

$pmk = New-RandomKey
do {
    $lmk = New-RandomKey
} while (Test-KeysEqual $pmk $lmk)

$header = @"
#ifndef ESPNOW_SECRETS_H
#define ESPNOW_SECRETS_H

// Generated locally. Never commit or transmit this file.
#define ESPNOW_SECURITY_ENABLED 1
#define ESPNOW_PMK_BYTES \
    $(Format-KeyBytes $pmk)
#define ESPNOW_LMK_BYTES \
    $(Format-KeyBytes $lmk)

#endif // ESPNOW_SECRETS_H
"@

$utf8WithoutBom = New-Object System.Text.UTF8Encoding($false)
foreach ($targetPath in $targetPaths) {
    [System.IO.File]::WriteAllText($targetPath, $header, $utf8WithoutBom)
}

$fingerprintInput = New-Object byte[] 32
[Array]::Copy($pmk, 0, $fingerprintInput, 0, 16)
[Array]::Copy($lmk, 0, $fingerprintInput, 16, 16)
$sha256 = [System.Security.Cryptography.SHA256]::Create()
try {
    $fingerprintBytes = $sha256.ComputeHash($fingerprintInput)
}
finally {
    $sha256.Dispose()
}
$fingerprint = (($fingerprintBytes[0..5] | ForEach-Object { '{0:X2}' -f $_ }) -join ':')

Write-Host "ESP-NOW security provisioned for Base and Rover."
Write-Host "Key fingerprint: $fingerprint"
Write-Host "Flash every Base/Rover/Relay with this same provisioned key set."
