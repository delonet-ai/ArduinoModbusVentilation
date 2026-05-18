param(
    [string]$HostName = "192.168.0.222",
    [string]$UserName = "root",
    [string]$Password = "wirenboard",
    [string]$HostKey = "SHA256:d6UF/LVkuvaMXz7xUKxZcRxZR/dX1WVp+q0S0oqu5gU"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$template = Join-Path $repoRoot "wirenboard\templates\config-arduino-modbus-servo.json"
$remoteTemplate = "/usr/share/wb-mqtt-serial/templates/config-arduino-modbus-servo.json"
$plink = "C:\Program Files\PuTTY\plink.exe"
$pscp = "C:\Program Files\PuTTY\pscp.exe"

if (-not (Test-Path $template)) {
    throw "Template not found: $template"
}

& $plink -batch -hostkey $HostKey -pw $Password "$UserName@$HostName" "mkdir -p /usr/share/wb-mqtt-serial/templates"
& $pscp -batch -hostkey $HostKey -pw $Password $template "$UserName@$HostName`:$remoteTemplate"
& $plink -batch -hostkey $HostKey -pw $Password "$UserName@$HostName" "systemctl restart wb-mqtt-serial; sleep 3; systemctl is-active wb-mqtt-serial; python3 -m json.tool $remoteTemplate >/dev/null && echo template-json-ok"
