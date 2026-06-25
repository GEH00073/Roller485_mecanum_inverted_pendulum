# Roller485 Mecanum Inverted Pendulum

Roller485 x 4 and 80 mm mecanum wheels are used to build a side-by-side inverted pendulum robot that can move laterally. This repository contains both the robot firmware and the Atom JoyStick controller firmware for the Transistor Technology article project.

## System

- Robot: AtomS3 + Roller485 x 4
- Controller: Atom JoyStick
- Communication: ESP-NOW ch 3 broadcast
- Drive power: 7.2 V battery to PWR485
- Wheels: 80 mm mecanum wheels

## Directory Structure

- `firmware/pendulum`: Robot firmware for AtomS3 + Roller485 x 4
- `firmware/controller`: Atom JoyStick controller firmware
- `docs`: Packet, wiring, and safety notes
- `images`: Photos and figures for documentation

## Safety Notes

- Always test with the wheels lifted off the floor during first bring-up.
- The robot motors do not start until a valid ESP-NOW packet is received.
- Local button-only startup on the robot is disabled for safety.
- Motors stop on fall detection and ESP-NOW receive timeout.
- The inverted pendulum can move suddenly; keep enough free space around it.

## Roller485 Layout

- LEFT = left outer, addr 0x64
- LEFT2 = left inner, addr 0x65
- RIGHT2 = right inner, addr 0x66
- RIGHT = right outer, addr 0x67

## Build

Install PlatformIO, then build each firmware from its own directory.

Robot firmware:

```bash
cd firmware/pendulum
pio run
```

Controller firmware:

```bash
cd firmware/controller
pio run
```

The checked-in `platformio.ini` files leave Wi-Fi credentials empty for public release. Set `WIFI_SSID`, `WIFI_PASSWORD`, and optional `OTA_PASSWORD` locally if OTA upload is needed.

## License

MIT License.

Original code by Kouhei Ito. Modified by Atsushi Kataoka.
