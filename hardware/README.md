# Hardware

## Bill of materials (BOM)

| # | Part | Qty | Notes |
|---|------|-----|-------|
| 1 | Raspberry Pi 5 (2 GB) | 1 | Raspberry Pi OS 64-bit |
| 2 | Freenove Robot Arm Kit | 1 | Onboard A4988 stepper drivers |
| 3 | Micro servo (gimbal rotate) | 1 | Signal on **GPIO 18** (hardware PWM) |
| 4 | Micro servo (gimbal tilt) | 1 | Signal on **GPIO 19** (hardware PWM) |
| 5 | Custom 3D-printed lamp head | 1 | See [`cad/`](cad/) |
| 6 | 2-axis SG90 gimbal (printed) | 1 | STL: https://grabcad.com/library/gimbal-2-axis-with-sg90-mini-servo-3d-printing-1 |
| 7 | Laptop with webcam | 1 | Runs the vision pipeline |
| 8 | Power supply for the arm | 1 | Per Freenove kit spec |

<!-- TODO: refine quantities / add exact part numbers and the power supply rating. -->

## Wiring

- Gimbal **rotate** servo signal → **GPIO 18**
- Gimbal **tilt** servo signal → **GPIO 19**
- Servo power/ground per the Freenove board's servo rail (not the Pi 3V3 rail)
- Arm steppers driven by the Freenove board's onboard A4988 drivers

<!-- TODO: add a wiring diagram at hardware/wiring.png -->
<!-- ![Wiring](wiring.png) -->

## Assembly notes

<!-- TODO: short build notes — mounting the gimbal on the arm end effector,
     lamp-head attachment, servo calibration (see gimbal_motion_test.py on the Pi). -->

## Raspberry Pi software setup

```bash
# On the Pi
sudo apt install python3-pip
pip install rpi-hardware-pwm gpiozero    # adjust to what the Pi code imports
python src/raspberry_pi/main_v3.py       # starts the TCP server on port 5000
```

The laptop connects to the Pi's IP on port `5000` (set `ROBOT_IP` in the laptop script).
