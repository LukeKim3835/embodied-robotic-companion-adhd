# Raspberry Pi (robot side)

Runs on the Raspberry Pi 5. Receives commands from the laptop over TCP (port 5000) and
drives the robot.

## Expected files (add here)

- `main_v3.py` — TCP server. Parses G-code arm commands and gimbal commands, drives the
  Freenove board's stepper motors via GPIO and the gimbal servos via hardware PWM.
- `gimbal_motion_test.py` — calibrates gimbal servo positions (rotate centre/look,
  tilt nod up/down).
- `hwpwm_servo_test.py` — minimal hardware-PWM test for GPIO 18 and GPIO 19.

<!-- TODO: copy these three files from the Raspberry Pi into this folder. -->
