# ble_uart origin

Copied from ESP-IDF **v6.0.2** so this project does not depend on
`EXTRA_COMPONENT_DIRS` pointing at `$IDF_PATH/examples`.

| Item | Value |
| --- | --- |
| IDF version | 6.0.2 |
| Source path | `$IDF_PATH/examples/bluetooth/common/ble_uart/` |
| Commit | `7101770dc6db2667b3c477cc31365dd1acd6db4e` (`change(version): Update version to 6.0.2`) |
| License | `Unlicense OR CC0-1.0` (see SPDX headers on each source file) |

## Files copied (unchanged)

- `ble_uart.h`
- `ble_uart_nimble.c`
- `ble_uart_bluedroid.c` (listed in upstream `CMakeLists.txt`; body is compiled out when NimBLE is the host)
- `CMakeLists.txt`
- `Kconfig`
- `PORTING.md`

No line edits in those files. GATT UUIDs stay Nordic UART Service:

- Service `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- RX `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- TX `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

## Encryption off (`encrypted=false`)

Upstream `ble_uart_service` example calls `ble_uart_install()` with
`.encrypted = true`, which starts LE Secure Connections + a 6-digit
passkey. Windows `bleak` often stalls on that pairing flow.

This project does **not** patch `ble_uart_nimble.c`. Encryption is
disabled at the application call site in `firmware/main/ble_app.c`:

```c
ble_uart_install(&(ble_uart_config_t){
    .encrypted      = (APP_BLE_ENCRYPTED != 0),  /* APP_BLE_ENCRYPTED=0 */
    .device_name    = APP_BLE_DEVICE_NAME,       /* "IMU-AHRS" */
    .ble_uart_on_rx = ble_on_rx,                 /* MVP: log and drop */
});
```

With `encrypted=false`, upstream NimBLE backend already:

- clears GATT `_ENC | _AUTHEN` flags
- sets `sm_sc = sm_bonding = sm_mitm = 0`
- does **not** call `ble_gap_security_initiate()` on connect

`sdkconfig.defaults` still copies the example NimBLE knobs
(`CONFIG_BT_NIMBLE_SM_SC`, `CONFIG_BT_NIMBLE_NVS_PERSIST`) so the host
can be built; they are not used while `APP_BLE_ENCRYPTED` is 0.
