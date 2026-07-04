# Raspberry Pi (robot side)

Runs on the Raspberry Pi 5. Receives commands from the laptop over TCP (port 5000) and
drives the robot.

- `main_v3.py` — TCP server. Parses G-code arm commands and gimbal commands, drives the
  Freenove board's stepper motors via GPIO and the gimbal servos via hardware PWM
  (sysfs `/sys/class/pwm`, GPIO 18 = rotate, GPIO 19 = tilt). Handles `GIMBAL_SHAKE_NO`
  and `GIMBAL_NOD_YES` gesture commands.
- `gimbal_motion_test.py` — interactive calibration of gimbal servo positions
  (rotate centre/look, tilt nod/shake amplitudes).
- `hwpwm_servo_test.py` — minimal hardware-PWM test for GPIO 18 / GPIO 19
  (`sweep`, `center`, `jitter`, `rotate <deg>`, `wrist <deg>`, `cleanup`).

## Setup

Enable 2-channel hardware PWM on the Pi — add to `/boot/firmware/config.txt` and reboot:

```
dtoverlay=pwm-2chan
```

`main_v3.py` also depends on the Freenove Robot Arm Kit server modules (`arm`, `servo`,
`buzzer`, `command`, `messageThread`, etc.), which ship with the kit and must be present
on the Pi's `PYTHONPATH`.

```bash
python main_v3.py      # starts the TCP server on port 5000
```
