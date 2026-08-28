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
$remoteLegacyRule = "/etc/wb-rules/ventilation.js"
$plink = "C:\Program Files\PuTTY\plink.exe"
$pscp = "C:\Program Files\PuTTY\pscp.exe"
$legacyCleanup = @"
import subprocess
result = subprocess.run(
    ['mosquitto_sub', '-W', '2', '-v', '-t', '/devices/ventilation/#'],
    stdout=subprocess.PIPE,
    text=True,
)
topics = sorted({
    line.split(' ', 1)[0]
    for line in result.stdout.splitlines()
    if line.startswith('/devices/ventilation/')
})
for topic in topics:
    subprocess.run(['mosquitto_pub', '-r', '-n', '-t', topic], check=True)
print('legacy-topics-removed', len(topics))
"@
$legacyCleanupEncoded = [Convert]::ToBase64String(
    [Text.Encoding]::UTF8.GetBytes($legacyCleanup)
)

if (-not (Test-Path $template)) {
    throw "Template not found: $template"
}
& $plink -batch -hostkey $HostKey -pw $Password "$UserName@$HostName" "mkdir -p /usr/share/wb-mqtt-serial/templates"
& $pscp -batch -hostkey $HostKey -pw $Password $template "$UserName@$HostName`:$remoteTemplate"
& $plink -batch -hostkey $HostKey -pw $Password "$UserName@$HostName" "rm -f $remoteLegacyRule; python3 -m json.tool $remoteTemplate >/dev/null && echo template-json-ok; systemctl restart wb-mqtt-serial wb-rules; sleep 3; systemctl is-active wb-mqtt-serial; systemctl is-active wb-rules"
& $plink -batch -hostkey $HostKey -pw $Password "$UserName@$HostName" "python3 - <<'PY'
import base64
exec(base64.b64decode('$legacyCleanupEncoded').decode())
PY"
