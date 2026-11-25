# Sensor tuning protocol

The bridge writes to the sensor Cycling Power Control Point. It uses standard
Start Offset Compensation (`0x0C`) by default, including when the feature
characteristic is unavailable. Enhanced Start Offset Compensation (`0x10`) is
selected only when feature bit 19 is set; feature value 128 (bit 7, accumulated
energy) does not enable it. An enhanced unsupported response triggers a
standard retry. The write acknowledgment is not a tune result; the bridge
waits up to 30 seconds for a matching response.

A response must match the opcode of the active tune. Standard Bluetooth response
header `0x20` is recognized; the MYX header `0x10` is accepted only for the two
exact observed frames below. Standard success must contain exactly two offset
bytes; enhanced success must contain its offset, company ID, and a correctly
sized length-prefixed manufacturer payload. Offsets are signed little-endian.
The observed MYX standard success has no offset. The overview shows `Not reported`
and JSON exposes `tuneOffsetReported: false` and `tuneReportedOffset: null`.
These fields reset when a new tune starts, so a previous value cannot be mistaken
for a new result.

## Protocol regression fixtures

All rows assume the corresponding opcode is currently pending unless otherwise
specified. Byte strings are hexadecimal. These fixtures describe expected results
for replay against the control-point callback or on-device diagnostics.

| Pending | Frame | Expected result |
| --- | --- | --- |
| `0C` | `10 0C 01` | Success; offset not reported |
| `10` | `10 10 02` | Unsupported; retry standard |
| `0C` | `20 0C 01 00 00` | Success; offset 0 reported |
| `0C` | `20 0C 01 FE FF` | Success; offset -2 reported |
| `0C` | `20 0C 01` | Malformed success; failed |
| `0C` | `20 0C 01 00 00 00` | Malformed success; failed |
| `10` | `20 10 01 00 00 34 12 00` | Success; offset 0 reported |
| `10` | `20 10 01 00 00 34 12 01` | Malformed success; failed |
| `0C` | `10 0C 01 00 00` | Ignored: unobserved MYX frame |
| `10` | `10 0C 01` | Ignored: wrong pending opcode |
| `0C` | `10 10 02` | Ignored: wrong pending opcode |
| none | `10 0C 01` | Ignored: no tune pending |
