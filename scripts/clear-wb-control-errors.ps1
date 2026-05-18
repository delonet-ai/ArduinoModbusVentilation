param(
    [string]$HostName = "192.168.0.222",
    [string]$UserName = "root",
    [string]$Password = "wirenboard",
    [string]$HostKey = "SHA256:d6UF/LVkuvaMXz7xUKxZcRxZR/dX1WVp+q0S0oqu5gU",
    [string]$DeviceId = "arduino_modbus_slave_128"
)

$ErrorActionPreference = "Stop"

$plink = "C:\Program Files\PuTTY\plink.exe"
$remoteScript = @"
import subprocess
controls = ['Kitchen_out','Cabinet_out','Bedroom_out','Living_out','Kid_out','Kitchen_in','Cabinet_in','Bedroom_in','Living_in','Kid_in','Test Mode']
for control in controls:
    subprocess.run(['mosquitto_pub', '-r', '-n', '-t', f'/devices/$DeviceId/controls/{control}/meta/error'], check=True)
print('control errors cleared')
"@

$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($remoteScript))
& $plink -batch -hostkey $HostKey -pw $Password "$UserName@$HostName" "python3 - <<'PY'
import base64
exec(base64.b64decode('$encoded').decode())
PY"
