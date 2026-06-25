# Safety

An inverted pendulum can move suddenly. Keep enough free space around the robot and keep hands, cables, and tools away from the wheels while powered.

- Always test with the wheels lifted off the floor during first bring-up.
- Make sure the area around the robot is clear before enabling control.
- To prevent motion immediately after power-on, the receiver requires a valid ESP-NOW packet before motor startup.
- Motors stop when ESP-NOW receive timeout is detected.
- Motors stop when the tilt angle exceeds the fall-detection threshold.
- If Roller485 initialization fails, the control task is not started.
