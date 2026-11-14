# Long-L Corridor Bag

The v0.1.0 GitHub Release carries the frozen rich-texture seed 7 sensor bag as two assets because each GitHub release asset must stay below 2 GiB.

- `hospital-l-corridor-rich-seed7-v2-sensors.bag.part-001`
- `hospital-l-corridor-rich-seed7-v2-sensors.bag.part-002`

Download both parts into the same directory. On Windows PowerShell, join them with the built-in binary copy command:

```powershell
cmd /c copy /b hospital-l-corridor-rich-seed7-v2-sensors.bag.part-001+hospital-l-corridor-rich-seed7-v2-sensors.bag.part-002 sensors.bag
Get-FileHash .\sensors.bag -Algorithm SHA256
```

On Linux or WSL:

```bash
cat hospital-l-corridor-rich-seed7-v2-sensors.bag.part-001 \
    hospital-l-corridor-rich-seed7-v2-sensors.bag.part-002 > sensors.bag
sha256sum sensors.bag
```

Expected file size: `3002838721` bytes. Expected SHA-256: `09d5d6d3d346e83a13e3687c4f17a6ffe550c53ca4b38044c93d1bb789217e0e`.
