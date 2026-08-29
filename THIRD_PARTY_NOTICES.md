# Third-party notices

This file records the licenses of third-party material included in this source
tree.  Retain this file and the referenced license texts when redistributing.

## MadgwickAHRS

`firmware/pendulum/src/MadgwickAHRS.cpp` and `.h` are derived from the Arduino
MadgwickAHRS library, itself an implementation of Sebastian Madgwick's AHRS
algorithm.  The upstream repository states GNU General Public License version
3: <https://github.com/arduino-libraries/MadgwickAHRS>.  This repository uses
the precise designation `GPL-3.0-only`; its text is
`LICENSES/GPL-3.0-only.txt`.  Atsushi Kataoka's modifications are identified in
the files.  The completed `firmware/pendulum` firmware must be distributed
under GPL-3.0-only.

## M5Stack Unit Roller I2C driver

`firmware/pendulum/src/unit_rolleri2c.cpp` and `.hpp` retain M5Stack Technology
CO LTD's `MIT` SPDX notices.  Upstream: <https://github.com/m5stack/M5Unit-Roller>.
The files include a performance-oriented I2C modification from the Kouhei Ito
derived project; do not remove M5Stack attribution.

`firmware/controller/src/buzzer.cpp` and `.h` also retain their M5Stack
Technology CO LTD MIT SPDX notices.

## Controller dependencies resolved by PlatformIO

The controller no longer vendors M5AtomS3.  Its `platformio.ini` resolves
M5Unified 0.2.21 and the transitive M5GFX 0.2.28 from the official M5Stack
PlatformIO packages.  The completed controller firmware therefore is not
described as “MIT only”: it contains the MIT-licensed project code together
with these dependencies and their applicable notices.

| Dependency actually linked in the clean build | Conditions | Upstream |
| --- | --- | --- |
| M5Unified 0.2.21 | MIT | <https://github.com/m5stack/M5Unified> |
| M5GFX 0.2.28 | M5GFX is MIT; the linked library also includes its documented LovyanGFX (FreeBSD), font, and image-code notices | <https://github.com/m5stack/M5GFX> |

The linker maps for both controller environments contain M5Unified and M5GFX,
and contain no M5AtomS3 or FastLED object.  M5GFX's upstream license inventory
lists the relevant component notices, including LovyanGFX (FreeBSD), Adafruit
GFX/GLCD fonts (2-clause BSD), Bodmer fonts (FreeBSD), IPA fonts (IPA Font
License), efont (3-clause BSD), TomThumb (3-clause BSD), and MIT-licensed image
utilities.  A distributor of a completed controller binary must retain the
notices from the exact M5Unified/M5GFX package versions used to build it.

The removed vendored M5AtomS3 library was the only repository material carrying
the former LGPL-2.1-or-later and CC BY-SA 3.0 notices.  Those licenses no
longer apply to this repository's controller source or its completed firmware.

## Documentation, images, and 3D models

`docs/`, `images/`, and `3d_models/` are Copyright (c) 2026 Atsushi Kataoka and
are licensed under Creative Commons Attribution 4.0 International (CC BY 4.0).
Commercial use, redistribution, and adaptation are permitted subject to the
attribution requirements.  The official license text is
`LICENSES/CC-BY-4.0.txt`; each directory contains a short notice.

## Project-origin code

For the MIT-licensed Kouhei Ito / Atsushi Kataoka code, retain:

```
Copyright (c) 2024 Kouhei Ito
Modifications Copyright (c) 2026 Atsushi Kataoka

Original code by Kouhei Ito. Modified by Atsushi Kataoka.
```

No license decision is made here for the repository's documentation, images,
or 3D models because their authorship and intended publication license are not
recorded in this tree.
