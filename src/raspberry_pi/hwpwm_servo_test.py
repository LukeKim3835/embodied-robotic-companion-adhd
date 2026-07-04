#!/usr/bin/env python3
"""
Hardware PWM Servo Test v2
===========================
GPIO 18 (PWM channel 2) → rotate servo
GPIO 19 (PWM channel 3) → wrist (tilt) servo

Uses raw sysfs (not rpi-hardware-pwm) so we have full control over
export/unexport lifecycle — avoids the "already exported" trap.
"""

import os
import sys
import time
import atexit


CHIP = 0
PWM_FREQ_HZ = 50
PERIOD_US    = 1_000_000 // PWM_FREQ_HZ   # 20000 μs
MIN_PULSE_US = 500    # 0°
MAX_PULSE_US = 2500   # 180°

ROTATE_CHANNEL = 2   # GPIO 18
WRIST_CHANNEL  = 3   # GPIO 19

PWMCHIP_PATH = f"/sys/class/pwm/pwmchip{CHIP}"


def _write(path, value):
    """Write to sysfs file. Returns True on success."""
    try:
        with open(path, "w") as f:
            f.write(str(value))
        return True
    except Exception as e:
        print(f"[warn] write {path}={value} failed: {e}")
        return False

def force_unexport(channel):
    """Force-unexport a channel. Safe if not exported."""
    ch_path = f"{PWMCHIP_PATH}/pwm{channel}"
    if os.path.exists(ch_path):
        _write(f"{ch_path}/enable", 0)
        time.sleep(0.05)
        try:
            _write(f"{PWMCHIP_PATH}/unexport", channel)
        except Exception:
            pass
        time.sleep(0.05)

def cleanup_all():
    """Cleanup both channels on exit."""
    for ch in (ROTATE_CHANNEL, WRIST_CHANNEL):
        force_unexport(ch)


class SysfsServo:
    def __init__(self, channel, name):
        self.channel = channel
        self.name    = name
        self.path    = f"{PWMCHIP_PATH}/pwm{channel}"
        
        # Clean up any stuck state from previous runs
        force_unexport(channel)
        
        # Export fresh
        if not os.path.exists(self.path):
            if not _write(f"{PWMCHIP_PATH}/export", channel):
                raise RuntimeError(f"Failed to export channel {channel}")
            time.sleep(0.1)
        
        # Set period
        period_ns = PERIOD_US * 1000
        if not _write(f"{self.path}/period", period_ns):
            raise RuntimeError(f"Failed to set period on channel {channel}")
        
        # Set initial duty (90°) before enabling
        self.set_angle(90)
        
        # Enable output
        if not _write(f"{self.path}/enable", 1):
            raise RuntimeError(f"Failed to enable channel {channel}")
        
        print(f"[OK] {name}: channel={channel}, chip={CHIP}")
    
    def set_angle(self, angle):
        angle = max(0, min(180, angle))
        pulse_us = MIN_PULSE_US + (angle / 180.0) * (MAX_PULSE_US - MIN_PULSE_US)
        duty_ns = int(pulse_us * 1000)
        _write(f"{self.path}/duty_cycle", duty_ns)


def main():
    if not os.path.exists(PWMCHIP_PATH):
        print(f"[FAIL] {PWMCHIP_PATH} not found.")
        print("       Check /boot/firmware/config.txt has: dtoverlay=pwm-2chan")
        print("       Then reboot.")
        sys.exit(1)
    
    cmd = sys.argv[1].lower() if len(sys.argv) > 1 else "sweep"
    
    # Cleanup-only mode
    if cmd == "cleanup":
        cleanup_all()
        print("Channels released.")
        return
    
    # Register cleanup handler
    atexit.register(cleanup_all)
    
    # Initialize both servos
    try:
        rotate = SysfsServo(ROTATE_CHANNEL, "rotate (GPIO18)")
        wrist  = SysfsServo(WRIST_CHANNEL,  "wrist  (GPIO19)")
    except Exception as e:
        print(f"[FAIL] {e}")
        print("       Try: sudo python3 hwpwm_servo_test.py cleanup")
        sys.exit(1)
    
    time.sleep(0.5)
    print("Both servos centered at 90°\n")
    
    try:
        
        if cmd == "rotate" and len(sys.argv) == 3:
            angle = float(sys.argv[2])
            rotate.set_angle(angle)
            print(f"rotate → {angle}°")
            time.sleep(1.0)
        
        elif cmd == "wrist" and len(sys.argv) == 3:
            angle = float(sys.argv[2])
            wrist.set_angle(angle)
            print(f"wrist → {angle}°")
            time.sleep(1.0)
        
        elif cmd == "center":
            rotate.set_angle(50)
            wrist.set_angle(90)
            print("Both → 90°")
            time.sleep(1.0)
        
        
        elif cmd == "jitter":
            print("Holding at 90° for 10 seconds — watch and listen.")
            print("If servos are silent and still, hardware PWM is working.\n")
            rotate.set_angle(90)
            wrist.set_angle(90)
            for i in range(10):
                print(f"  holding... {i+1}/10")
                time.sleep(1.0)
        
        
        else:
            print("=== Sweep test ===\n")
            
            print("[3/4] shake_no (rotate back-and-forth)")
            rotate.set_angle(125)
            for _ in range(3):
                wrist.set_angle(70)
                time.sleep(0.25)
                wrist.set_angle(110)
                time.sleep(0.25)
            rotate.set_angle(90)
            time.sleep(0.5)
            
            print("[4/4] nod_yes (wrist up-and-down)")
            rotate.set_angle(50)
            for _ in range(3):
                wrist.set_angle(70)
                time.sleep(0.25)
                wrist.set_angle(110)
                time.sleep(0.25)
            wrist.set_angle(90)
            time.sleep(0.5)
            
            print("\n=== Sweep complete ===")
    
    except KeyboardInterrupt:
        print("\n[Ctrl+C] stopping...")
    
    # Return to center before cleanup
    try:
        rotate.set_angle(90)
        wrist.set_angle(90)
        time.sleep(0.5)
    except Exception:
        pass

if __name__ == "__main__":
    main()
