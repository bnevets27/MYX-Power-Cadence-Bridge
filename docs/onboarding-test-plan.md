# Browser onboarding acceptance checks

Run on both supported boards before release. These checks require hardware;
passing a firmware build does not establish the results below. Use disposable
test credentials and a test network. Do not record passwords in test logs.

| Scenario | Expected result |
| --- | --- |
| Fresh installation, bike asleep | `MYX-Bridge-Setup` appears without a password. Opening `http://192.168.4.1/` shows setup, with no serial-console step or HTTP login prompt. |
| Upgrade from generated-password firmware | Browser setup becomes available without knowing the old generated password; calibration and MQTT settings survive. |
| First setup with Wi-Fi | Choose and confirm an administrator password, supply Wi-Fi details, and save. After restart, the LAN UI requires `admin` and the chosen password. |
| First setup without Wi-Fi | Save the administrator password. Normal AP access uses the chosen password and the normal sensor-dependent AP behavior resumes. |
| Invalid administrator password | Empty, fewer than 8 characters, more than 63 characters, non-ASCII, control characters, or mismatched confirmation is rejected without changing settings. |
| Password with HTML punctuation | A permitted password containing quotes or angle brackets works after restart; submitted text never becomes executable HTML. |
| Open setup attack surface | `/api/status`, normal settings, reset, tune, and `/update` cannot be used through the open AP, even with previously valid administrator credentials. |
| Setup from LAN | Setup page and submission are rejected when addressed through the LAN interface, including when the recovery AP and LAN are both active. |
| Forged or replayed submission | Missing/wrong setup token, foreign Origin, and a token from an expired/reopened window are rejected without saving settings. |
| Setup expires | After 10 minutes without saving, the open AP closes. An already configured device resumes its previous settings. |
| Physical recovery | After normal boot, hold BOOT for 5 seconds. Setup reopens without erasing existing credentials. A brief press does not open it; a held button does not repeatedly extend the window. |
| Cancel recovery | Open recovery, then let it expire. Existing administrator/Wi-Fi/MQTT/calibration settings remain usable. |
| Save recovery | Choose a new administrator password and update Wi-Fi. The new password works after restart; the old administrator password does not. |
| Wrong Wi-Fi password | Setup save completes; if LAN cannot connect, the protected fallback AP permits correcting Wi-Fi with the chosen administrator password. |
| Bike disappears during setup | Setup remains reachable for the rest of its window. Saving works without waking the bike. |
| Existing BLE ride during recovery | Already established BLE streaming continues; recovery work resumes after setup closes. |
| Reboot after completed setup | No open setup hotspot is automatically offered; authentication remains required. |
| Storage write failure | Setup reports an error and stays open for retry. It does not report success or schedule a restart when persistence fails. |

Press BOOT only after the board has booted normally. Holding GPIO0 low during
reset enters the ROM flashing bootloader instead of this firmware's setup mode.
See [Espressif boot-mode documentation](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html).
