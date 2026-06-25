# Wiring

## AtomS3 to Roller485

Connect the AtomS3 and four Roller485 units in parallel on PORT.A using Grove 4-wire cables.

AtomS3 PORT.A signals:

- G7
- G8
- 5V
- GND

The four Roller485 units share the same I2C bus and are distinguished by address.

## Roller485 Power

Supply 7.2 V and GND from the 7.2 V battery to each Roller485 PWR485 input.

## Roller485 Address Layout

- LEFT = left outer, addr 0x64
- LEFT2 = left inner, addr 0x65
- RIGHT2 = right inner, addr 0x66
- RIGHT = right outer, addr 0x67

## Controller Link

The Atom JoyStick controller connects wirelessly to the AtomS3 receiver using ESP-NOW on channel 3.
