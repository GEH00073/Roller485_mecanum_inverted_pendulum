# ESP-NOW Packet

The controller sends a fixed 25-byte ESP-NOW broadcast packet on channel 3.

```text
Packet size: 25 bytes
byte 0-2: destination lower 3 bytes or FF FF FF for broadcast
byte 3-6: rudder / Psi, float
byte 7-10: throttle, float
byte 11-14: aileron / Phi, float
byte 15-18: elevator / Theta, float
byte 19: arm button
byte 20: flip button
byte 21: mode button
byte 22: option button
byte 23: proactive flag
byte 24: checksum, sum of bytes 0-23
```

The current public firmware uses `FF FF FF` in bytes 0-2 for broadcast operation. The receiver updates its last valid receive timestamp only when the packet length, destination, and checksum are valid.
