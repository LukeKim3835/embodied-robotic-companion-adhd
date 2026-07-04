#!/usr/bin/env python3
# gimbal_motion_test.py
import os
import time
import math

PERIOD_NS    = 20_000_000
MIN_PULSE_NS =    500_000
MAX_PULSE_NS =  2_500_000

ROTATE_CENTER    = 50
ROTATE_LOOK      = 125
TILT_CENTER      = 80
ROTATE_SHAKE_AMP = 20
TILT_NOD_AMP     = 20
GIMBAL_SMOOTH_STEPS = 40

def _angle_to_ns(angle):
    clamped = max(0.0, min(180.0, float(angle)))
    return int(MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * clamped / 180.0)

class PioServo:
    def __init__(self, chip_path):
        self.path = chip_path + "/pwm0"
        self.current = 90.0

        
        if not os.path.exists(self.path):
            with open(chip_path + "/export", "w") as f:
                f.write("0")
            time.sleep(0.2)

       
        with open(f"{self.path}/enable", "w") as f:
            f.write("0")
        with open(f"{self.path}/period", "w") as f:
            f.write(str(PERIOD_NS))
        with open(f"{self.path}/duty_cycle", "w") as f:
            f.write(str(_angle_to_ns(90)))
        with open(f"{self.path}/enable", "w") as f:
            f.write("1")

       
        self._duty_fd = open(f"{self.path}/duty_cycle", "w", buffering=1)

    def set_angle(self, angle):
        self.current = max(0.0, min(180.0, float(angle)))
        self.servo.value = (self.current - 90.0) / 90.0

    def detach(self):
        self.servo.value = None  

    def reattach(self):
        with open(f"{self.path}/enable", "w") as f:
            f.write("1")

    def close(self):
        try:
            self._duty_fd.close()
        except:
            pass
        self.detach()


def smooth_move(servo, start, end, duration=0.3):
    steps = GIMBAL_SMOOTH_STEPS
    dt = duration / steps
    for i in range(steps + 1):
        eased = (1.0 - math.cos(math.pi * i / steps)) / 2.0
        servo.set_angle(start + (end - start) * eased)
        time.sleep(dt)
    servo.detach()   



chips = sorted([
    f"/sys/class/pwm/{d}"
    for d in os.listdir("/sys/class/pwm")
    if d.startswith("pwmchip")
])
r = PioServo(chips[1])
t = PioServo(chips[0])


def do_shake_no():
    smooth_move(r, r.current, ROTATE_LOOK, duration=0.5)
    cur = TILT_CENTER
    for target in [TILT_CENTER - TILT_NOD_AMP,
                   TILT_CENTER + TILT_NOD_AMP,
                   TILT_CENTER - TILT_NOD_AMP,
                   TILT_CENTER + TILT_NOD_AMP,
                   TILT_CENTER - TILT_NOD_AMP,
                   TILT_CENTER]:
        smooth_move(t, cur, target, duration=duration)
        cur = target
    smooth_move(r, ROTATE_LOOK, ROTATE_CENTER, duration=0.5)

def do_nod_yes():
    smooth_move(r, r.current, ROTATE_CENTER, duration=0.3)
    cur = TILT_CENTER
    for target in [TILT_CENTER - TILT_NOD_AMP,
                   TILT_CENTER + TILT_NOD_AMP,
                   TILT_CENTER - TILT_NOD_AMP,
                   TILT_CENTER + TILT_NOD_AMP,
                   TILT_CENTER - TILT_NOD_AMP,
                   TILT_CENTER]:
        smooth_move(t, cur, target, duration=duration)
        cur = target



smooth_move(r, 90, ROTATE_CENTER, duration=0.5)
smooth_move(t, 90, TILT_CENTER,   duration=0.5)
time.sleep(0.5)



duration = 0.21
amp = 20

while True:
    cmd = input(f"[duration={duration} amp={amp}] > ").strip()
    if cmd == 'q':
        break
    elif cmd == '1':
        ROTATE_SHAKE_AMP = amp
        TILT_NOD_AMP = amp
        do_shake_no()
    elif cmd == '2':
        ROTATE_SHAKE_AMP = amp
        TILT_NOD_AMP = amp
        do_nod_yes()
    elif cmd == 's':
        smooth_move(r, r.current, ROTATE_CENTER, duration=0.5)
        smooth_move(t, t.current, TILT_CENTER,   duration=0.5)
    elif cmd.startswith('d'):
        try:
            duration = float(cmd[1:])
            print(f"duration → {duration}")
        except:
            print("(d0.15)")
    elif cmd.startswith('a'):
        try:
            amp = int(cmd[1:])
            print(f"amp → {amp}")
        except:
            print("(a25)")

r.close()
t.close()
print("quit")