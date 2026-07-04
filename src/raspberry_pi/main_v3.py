

import os
import time
import socket
import fcntl
import struct
import threading
import atexit
import messageThread
import messageQueue
import messageParser
import messageRecord
import command
import arm
import buzzer
import servo
# import ledPixel   ← DISABLED: GPIO18 is now used by the rotate servo


# Wiring:
#   rotate servo → Freenove LED port  → GPIO 18 → PWM0_CHAN2 → channel 2
#   wrist  servo → Freenove servo ch2 → GPIO 19 → PWM0_CHAN3 → channel 3
GIMBAL_CHIP      = 0
ROTATE_CHANNEL   = 2     # GPIO 18
TILT_CHANNEL     = 3     # GPIO 19

PWMCHIP_PATH     = f"/sys/class/pwm/pwmchip{GIMBAL_CHIP}"
PWM_PERIOD_NS    = 20_000_000   # 50 Hz
PWM_MIN_PULSE_NS =    500_000   # 0°
PWM_MAX_PULSE_NS =  2_500_000   # 180°

_GIMBAL_AVAILABLE = os.path.exists(PWMCHIP_PATH)
if not _GIMBAL_AVAILABLE:
    print(f"[GIMBAL] {PWMCHIP_PATH} not found — check /boot/firmware/config.txt "
          "has 'dtoverlay=pwm-2chan' and reboot.")



ROTATE_CENTER    = 50    # lamp looking away (standby)
ROTATE_LOOK      = 125   # lamp facing user
TILT_CENTER      = 90    # wrist neutral (verified in hwpwm_servo_test.py)

SHAKE_AMP        = 20    # wrist oscillation during shake_no  (→ 70/110)
NOD_AMP          = 20    # wrist oscillation during nod_yes   (→ 70/110)
SHAKE_REPS       = 3
NOD_REPS         = 3
STEP_PAUSE       = 0.25  # pause between wrist positions

GIMBAL_SHAKE_WAIT = 2.0
GIMBAL_NOD_WAIT   = 3.5


─
def _pwm_write(path, value):
    try:
        with open(path, "w") as f:
            f.write(str(value))
        return True
    except Exception as e:
        print(f"[GIMBAL] sysfs write {path}={value} failed: {e}")
        return False

def _pwm_force_unexport(channel):
    ch_path = f"{PWMCHIP_PATH}/pwm{channel}"
    if os.path.exists(ch_path):
        _pwm_write(f"{ch_path}/enable", 0)
        time.sleep(0.05)
        _pwm_write(f"{PWMCHIP_PATH}/unexport", channel)
        time.sleep(0.05)

class SysfsServo:
    """Hardware-PWM servo via /sys/class/pwm. Mimics gpiozero.Servo API
    just enough to be a drop-in replacement for this codebase."""

    def __init__(self, channel, name=""):
        self.channel = channel
        self.name    = name
        self.path    = f"{PWMCHIP_PATH}/pwm{channel}"

        _pwm_force_unexport(channel)
        if not os.path.exists(self.path):
            if not _pwm_write(f"{PWMCHIP_PATH}/export", channel):
                raise RuntimeError(f"export ch={channel} failed")
            time.sleep(0.1)

        _pwm_write(f"{self.path}/period", PWM_PERIOD_NS)
        self.set_angle(90)
        _pwm_write(f"{self.path}/enable", 1)

    def set_angle(self, angle):
        a = max(0.0, min(180.0, float(angle)))
        pulse_ns = int(PWM_MIN_PULSE_NS + (a / 180.0) * (PWM_MAX_PULSE_NS - PWM_MIN_PULSE_NS))
        _pwm_write(f"{self.path}/duty_cycle", pulse_ns)

    def stop(self):
        try:
            _pwm_write(f"{self.path}/enable", 0)
            time.sleep(0.05)
            _pwm_write(f"{PWMCHIP_PATH}/unexport", self.channel)
        except Exception:
            pass

class ArmServer:
    def __init__(self):
        self.robotAction = arm.Arm()
        self.robotServo = servo.Servo()
        self.robotLed = None   # LED disabled — GPIO 18 is now used by rotate servo
        self.robotBuzzer = buzzer.Buzzer()
        self.robotFile = messageRecord.MessageRecord()
        self.cmd = command.Command()
        self.queueParser = messageParser.MessageParser()
        self.queueActionParser = messageParser.MessageParser()
        self.queueBuzzerParser = messageParser.MessageParser()
        self.queueLedParser = messageParser.MessageParser()
        self.queueAction = messageQueue.MessageQueue()
        self.queueLed = messageQueue.MessageQueue()
        self.queueBuzzer = messageQueue.MessageQueue()
        self.last_pozition = [0,0,0]
        self.current_pozition = [0,0,0]
        self.plane_axis_state = [0,0,0,0,0,0]
        self.calibration_height = 20
        self.thread_led_parameter = [0,0,0,0]
        self.threadingReceive = None
        self.threadingArm = None
        self.threadingLed = None
        self.threadingBuzzer = None
        self.threadingActionFeedback = None
        self.threadings_state = 0
        self.robotAction.setClampLength(self.robotFile.readJsonObject("Clamp Length"))
        self.robotAction.setOriginHeight(self.robotFile.readJsonObject("Original Height"))
        self.robotAction.setGroundHeight(self.robotFile.readJsonObject("Ground Height"))
        self.robotAction.setPenHeight(self.robotFile.readJsonObject("Pen Height"))
        self.robotAction.setMsxMode(self.robotFile.readJsonObject("A4988 MSx"))
        self.robotAction.setFrequency(self.robotFile.readJsonObject("A4988 CLK"))
        self.robotAction.setArmOffseAngle(self.robotFile.readJsonObject("Home Angle Offset"))
        self.homePoint = self.robotFile.readJsonObject("Home point")
        self.armState = self.robotFile.readJsonObject("Arm State")
        self.plane_axis_original_1 = [-100, 200, self.last_pozition[2]]
        self.plane_axis_original_2 = [100, 200, self.last_pozition[2]]
        self.plane_axis_original_3 = [0, 150, self.last_pozition[2]]
        self.plane_axis_original_4 = [0, 250, self.last_pozition[2]]
        self.plane_axis_offset_1 = self.robotFile.readJsonObject("Point 1")
        self.plane_axis_offset_2 = self.robotFile.readJsonObject("Point 2")
        self.plane_axis_offset_3 = self.robotFile.readJsonObject("Point 3")
        self.plane_axis_offset_4 = self.robotFile.readJsonObject("Point 4")
        plane_x_z_value = self.robotFile.readJsonObject("Plane X-Z")
        self.robotAction.setPlaneXZ(plane_x_z_value[0], plane_x_z_value[1], plane_x_z_value[2], plane_x_z_value[3])
        plane_y_z_value = self.robotFile.readJsonObject("Plane Y-Z")
        self.robotAction.setPlaneYZ(plane_y_z_value[0], plane_y_z_value[1], plane_y_z_value[2], plane_y_z_value[3])
        self.robotActionCheck = 0
        self.threadCheckServer = messageThread.create_thread(self.threadingCheckServer)
        self.threadCheckServer.start()

        self._gimbal_lock = threading.Lock()
        self._gimbal_busy = False
        self._rotate_servo = None
        self._tilt_servo   = None
        self._init_gimbal()
    
    def _init_gimbal(self):
        if not _GIMBAL_AVAILABLE:
            return
        try:
            self._rotate_servo = SysfsServo(ROTATE_CHANNEL, "rotate")
            self._tilt_servo   = SysfsServo(TILT_CHANNEL,   "tilt")
            self._rotate_current = ROTATE_CENTER
            self._tilt_current   = TILT_CENTER
            self._set_angle(self._rotate_servo, ROTATE_CENTER, "rotate")
            self._set_angle(self._tilt_servo,   TILT_CENTER,   "tilt")
            time.sleep(0.5)
            atexit.register(self._cleanup_gimbal)
            print("[GIMBAL] Initialized (hardware PWM, chip=%d, ch=%d/%d)."
                  % (GIMBAL_CHIP, ROTATE_CHANNEL, TILT_CHANNEL))
        except Exception as e:
            print(f"[GIMBAL] Init failed: {e}")
            self._rotate_servo = None
            self._tilt_servo   = None

    def _cleanup_gimbal(self):
        # Return to neutral before releasing
        try:
            if self._rotate_servo is not None:
                self._set_angle(self._rotate_servo, ROTATE_CENTER, "rotate")
            if self._tilt_servo is not None:
                self._set_angle(self._tilt_servo,   TILT_CENTER,   "tilt")
            time.sleep(0.3)
        except Exception:
            pass
        for s in (self._rotate_servo, self._tilt_servo):
            if s is not None:
                try:
                    s.stop()
                except Exception:
                    pass
        self._rotate_servo = None
        self._tilt_servo   = None
        print("[GIMBAL] Cleaned up.")

    def _gimbal_available(self):
        return self._rotate_servo is not None and self._tilt_servo is not None

    def _set_angle(self, servo_obj, angle, which):
        clamped = max(0.0, min(180.0, float(angle)))
        servo_obj.set_angle(clamped)
        if which == "rotate":
            self._rotate_current = clamped
        else:
            self._tilt_current = clamped

    def _do_shake_no(self):
        # Rotate to LOOK position
        self._set_angle(self._rotate_servo, ROTATE_LOOK, "rotate")
        time.sleep(0.4)
        for _ in range(SHAKE_REPS):
            self._set_angle(self._tilt_servo, TILT_CENTER - SHAKE_AMP, "tilt")
            time.sleep(STEP_PAUSE)
            self._set_angle(self._tilt_servo, TILT_CENTER + SHAKE_AMP, "tilt")
            time.sleep(STEP_PAUSE)
        self._set_angle(self._tilt_servo, TILT_CENTER, "tilt")
        time.sleep(0.2)
        # Return to standby
        self._set_angle(self._rotate_servo, ROTATE_CENTER, "rotate")
        time.sleep(0.5)

    def _do_nod_yes(self):
        # Ensure rotate is at CENTER, then wrist nods up-down
        self._set_angle(self._rotate_servo, ROTATE_CENTER, "rotate")
        time.sleep(0.2)
        for _ in range(NOD_REPS):
            self._set_angle(self._tilt_servo, TILT_CENTER - NOD_AMP, "tilt")
            time.sleep(STEP_PAUSE)
            self._set_angle(self._tilt_servo, TILT_CENTER + NOD_AMP, "tilt")
            time.sleep(STEP_PAUSE)
        self._set_angle(self._tilt_servo, TILT_CENTER, "tilt")
        time.sleep(0.5)

    def _run_gimbal_shake_no(self):
        with self._gimbal_lock:
            if self._gimbal_busy:
                return
            self._gimbal_busy = True
        try:
            time.sleep(GIMBAL_SHAKE_WAIT)
            print("[GIMBAL] shake_no start")
            self._do_shake_no()
            print("[GIMBAL] shake_no done")
        except Exception as e:
            print(f"[GIMBAL] shake_no error: {e}")
        finally:
            with self._gimbal_lock:
                self._gimbal_busy = False

    def _run_gimbal_nod_yes(self):
        timeout = 30.0
        start = time.monotonic()
        while True:
            with self._gimbal_lock:
                if not self._gimbal_busy:
                    self._gimbal_busy = True
                    break
            if time.monotonic() - start > timeout:
                print("[GIMBAL] nod_yes timeout, skipping.")
                return
            time.sleep(0.1)
        try:
            time.sleep(GIMBAL_NOD_WAIT)
            print("[GIMBAL] nod_yes start")
            self._do_nod_yes()
            print("[GIMBAL] nod_yes done")
        except Exception as e:
            print(f"[GIMBAL] nod_yes error: {e}")
        finally:
            with self._gimbal_lock:
                self._gimbal_busy = False

    def setThreadingReceiveState(self, state):
        try:
            buf_state = self.threadingReceive.is_alive()
            if state != buf_state:
                if state == True:
                    self.threadingReceive = messageThread.create_thread(self.threadingReceiveInstruction)
                    self.threadingReceive.start()
                elif state == False:
                    messageThread.stop_thread(self.threadingReceive)
        except:
            pass

    def setThreadingArmState(self, state):
        try:
            buf_state = self.threadingArm.is_alive()
            if state != buf_state:
                if state == True:
                    self.threadingArm = messageThread.create_thread(self.threadingRobotAction)
                    self.threadingArm.start()
                elif state == False:
                    messageThread.stop_thread(self.threadingArm)
        except:
            print("setThreadingArmState error.")

    def setThreadingLedState(self, state):
        try:
            buf_state = self.threadingLed.is_alive()
            if state != buf_state:
                if state == True:
                    self.threadingLed = messageThread.create_thread(self.threadingRobotLed)
                    self.threadingLed.start()
                elif state == False:
                    messageThread.stop_thread(self.threadingLed)
        except:
            print("setThreadingLedState error.")

    def setThreadingBuzzerState(self, state):
        try:
            buf_state = self.threadingBuzzer.is_alive()
            if state != buf_state:
                if state == True:
                    self.threadingBuzzer = messageThread.create_thread(self.threadingRobotBuzzer)
                    self.threadingBuzzer.start()
                elif state == False:
                    messageThread.stop_thread(self.threadingBuzzer)
        except:
            print("setThreadingBuzzerState error.")

    def setThreadingFeedbackState(self, state):
        try:
            buf_state = self.threadingActionFeedback.is_alive()
            if state != buf_state:
                if state == True:
                    self.threadingActionFeedback = messageThread.create_thread(self.threadingRobotActionFeedback)
                    self.threadingActionFeedback.start()
                elif state == False:
                    messageThread.stop_thread(self.threadingActionFeedback)
        except:
            print("setThreadingFeedbackState error.")

    def setRobotBuzzer(self, frequency, delayms, times):
        cmd = self.cmd.CUSTOM_ACTION + str("2") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.BUZZER_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.BUZZER_ACTION + str(frequency) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.BUZZER_ACTION + str(delayms) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.BUZZER_ACTION + str(times)
        self.queueBuzzer.put(cmd)

    def setRobotLED(self, mode, r, g, b):
        cmd = self.cmd.CUSTOM_ACTION + str("1") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_MODE + str(mode) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_RED + str(r) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_GREEN + str(g) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.WS2812_BLUE + str(b)
        self.queueLed.put(cmd)

    def setRobotAction(self, axis):
        cmd = self.cmd.MOVE_ACTION + str("0") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.AXIS_X_ACTION + str(axis[0]) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.AXIS_Y_ACTION + str(axis[1]) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.AXIS_Z_ACTION + str(axis[2])
        self.queueAction.put(cmd)

    def sendRobotPaintingRadius(self, radius):
        cmd = self.cmd.CUSTOM_ACTION + str("14") + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.PAINT_MIN_RADIUS + str(radius[0]) + self.cmd.DECOLLATOR_CHAR \
            + self.cmd.PAINT_MAX_RADIUS + str(radius[1]) + "\r\n"
        self.serverSend(cmd)

    def get_interface_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s',b'wlan0'[:15]))[20:24])

    def serverSend(self, data):
        self.connection.send(data.encode('utf-8'))

    def turn_on_server(self):
        SOCKET_IP = self.get_interface_ip()
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server_socket.bind((SOCKET_IP, 5000))
        self.server_socket.listen(1)
        print("main.py,", 'Server address: ' + SOCKET_IP)
        self.threadingReceive = messageThread.create_thread(self.threadingReceiveInstruction)
        self.threadingArm = messageThread.create_thread(self.threadingRobotAction)
        self.threadingLed = messageThread.create_thread(self.threadingRobotLed)
        self.threadingBuzzer = messageThread.create_thread(self.threadingRobotBuzzer)
        self.threadingActionFeedback = messageThread.create_thread(self.threadingRobotActionFeedback)
        self.robotAction.setArmEnable(0)   # 추가
        self.robotActionCheck = 12         # 추가
        print("main.py, Motor enabled.")
        self.setThreadingReceiveState(True)
        self.setThreadingArmState(True)
        self.setThreadingLedState(True)
        self.setThreadingBuzzerState(True)

    def turn_off_server(self):
        try:
            self.thread_led_parameter = [0,0,0,0]
            self.setThreadingReceiveState(False)
            self.setThreadingArmState(False)
            self.setThreadingBuzzerState(False)
            self.setThreadingFeedbackState(False)
            self.setThreadingLedState(False)
            self.connection.close()
        except:
            print("Turn off server failed.")
        self._cleanup_gimbal()

    def safetyOperationInspection(self):
        cmd = None
        if self.robotActionCheck == 0:
            cmd = self.cmd.CUSTOM_ACTION + str("8") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_ENABLE + str("0") + str('\r\n')
            self.serverSend(cmd)
            print("Please enable the motor.")
        elif self.robotActionCheck == 1:
            cmd = self.cmd.CUSTOM_ACTION + str("10") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_SENSOR_POINT + str("0") + str('\r\n')
            self.serverSend(cmd)
            print("Please calibrate the sensor point.")
        elif self.robotActionCheck == 2:
            cmd = self.cmd.CUSTOM_ACTION + str("10") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_SENSOR_POINT + str("1") + str('\r\n')
            self.serverSend(cmd)
            print("Please goto the sensor point.")
        elif self.robotActionCheck == 3:
            cmd = self.cmd.CUSTOM_ACTION + str("3") + self.cmd.DECOLLATOR_CHAR + self.cmd.GROUND_HEIGHT + str("?") + str('\r\n')
            self.serverSend(cmd)
            print("Please set the height of the bottom of the robot arm to the ground.")
        elif self.robotActionCheck == 4:
            cmd = self.cmd.CUSTOM_ACTION + str("4") + self.cmd.DECOLLATOR_CHAR + self.cmd.CLAMP_LENGTH + str("?") + str('\r\n')
            self.serverSend(cmd)
            print("Please set the length of the clamp.")
        elif self.robotActionCheck == 5:
            cmd = self.cmd.CUSTOM_ACTION + str("5") + self.cmd.DECOLLATOR_CHAR \
                + self.cmd.AXIS_X_ACTION + str("?") + self.cmd.DECOLLATOR_CHAR \
                + self.cmd.AXIS_Y_ACTION + str("?") + self.cmd.DECOLLATOR_CHAR \
                + self.cmd.AXIS_Z_ACTION + str("?") + str('\r\n')
            self.serverSend(cmd)
            print("Please set the original coordinates of the home point.")
        elif self.robotActionCheck == 6:
            cmd = self.cmd.CUSTOM_ACTION + str("11") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_CALIBRATION_START + str("?") + str('\r\n')
            self.serverSend(cmd)
            print("Please select the point you want to calibrate first.")
        elif self.robotActionCheck == 7:
            cmd = self.cmd.CUSTOM_ACTION + str("11") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_CALIBRATION_END + str("0") + str('\r\n')
            self.serverSend(cmd)
            print("Failed to calibrate the home point.")
        elif self.robotActionCheck == 8:
            cmd = self.cmd.CUSTOM_ACTION + str("11") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_CALIBRATION_END + str("1") + str('\r\n')
            self.serverSend(cmd)
            print("Failed to calibrate the point 1.")
        elif self.robotActionCheck == 9:
            cmd = self.cmd.CUSTOM_ACTION + str("11") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_CALIBRATION_END + str("2") + str('\r\n')
            self.serverSend(cmd)
            print("Failed to calibrate the point 2.")
        elif self.robotActionCheck == 10:
            cmd = self.cmd.CUSTOM_ACTION + str("11") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_CALIBRATION_END + str("3") + str('\r\n')
            self.serverSend(cmd)
            print("Failed to calibrate the point 3.")
        elif self.robotActionCheck == 11:
            cmd = self.cmd.CUSTOM_ACTION + str("11") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_CALIBRATION_END + str("4") + str('\r\n')
            self.serverSend(cmd)
            print("Failed to calibrate the point 4.")
        if self.robotActionCheck == 12:
            return 1
        elif self.robotActionCheck != 12:
            self.setRobotBuzzer(1000, 100, 1)
            if 6 <= self.robotActionCheck <= 11:
                self.robotActionCheck = 12
            return 0

    def threadingReceiveInstruction(self):
        try:
            self.connection, self.client_address = self.server_socket.accept()
            print("main.py, Client connection successful!")
        except:
            print("main.py, Client connect failed")
        self.server_socket.close()
        self.receiveData = None
        try:
            while True:
                try:
                    self.receiveData = self.connection.recv(1024).decode('utf-8')
                except:
                    self.threadings_state = 1
                    print("main.py, The socket was disconnected.")
                    break
                if self.receiveData == "":
                    self.threadings_state = 1
                else:
                    cmdArray = self.receiveData.split('\r\n')
                    print("main.py,", cmdArray)
                    if cmdArray[-1] != " ":
                        cmdArray = cmdArray[:-1]
                        for i in range(len(cmdArray)):
                            raw = cmdArray[i].strip()
                            if raw == "GIMBAL_SHAKE_NO":
                                if self._gimbal_available():
                                    threading.Thread(target=self._run_gimbal_shake_no, daemon=True).start()
                                else:
                                    print("[GIMBAL] shake_no requested but gimbal not available.")
                                continue
                            elif raw == "GIMBAL_NOD_YES":
                                if self._gimbal_available():
                                    threading.Thread(target=self._run_gimbal_nod_yes, daemon=True).start()
                                else:
                                    print("[GIMBAL] nod_yes requested but gimbal not available.")
                                continue
                            try:
                                self.queueParser.parser(cmdArray[i])
                            except:
                                print("main.py,", cmdArray[i])
                                self.queueParser.clearParameters()
                                continue
                            if self.queueParser.commandArray[0] == self.cmd.MOVE_ACTION:
                                if self.queueParser.intParameter[0] == 0 or self.queueParser.intParameter[0] == 1 or self.queueParser.intParameter[0] == 4:
                                    result = self.safetyOperationInspection()
                                    if result == 1:
                                        self.queueAction.put(cmdArray[i])
                                else:
                                    print("main.py, G{0} is error.".format(self.queueParser.intParameter[0]))
                            elif self.queueParser.commandArray[0] == self.cmd.CUSTOM_ACTION:
                                if self.queueParser.commandArray[1] == self.cmd.WS2812_MODE:
                                    self.queueLed.put(cmdArray[i])
                                elif self.queueParser.commandArray[1] == self.cmd.BUZZER_ACTION:
                                    self.queueBuzzer.put(cmdArray[i])
                                elif self.queueParser.commandArray[1] == self.cmd.GROUND_HEIGHT:
                                    self.robotAction.setGroundHeight(self.queueParser.intParameter[1])
                                    self.robotFile.writeJsonObject("Ground Height", self.queueParser.intParameter[1])
                                    self.robotActionCheck = 4
                                    self.armState[1] = 1
                                    self.robotFile.writeJsonObject("Arm State", self.armState)
                                elif self.queueParser.commandArray[1] == self.cmd.CLAMP_LENGTH:
                                    self.robotAction.setClampLength(self.queueParser.intParameter[1])
                                    self.robotFile.writeJsonObject("Clamp Length", self.queueParser.intParameter[1])
                                    self.robotActionCheck = 5
                                    self.armState[2] = 1
                                    self.robotFile.writeJsonObject("Arm State", self.armState)
                                elif self.queueParser.commandArray[1] == self.cmd.AXIS_X_ACTION:
                                    self.homePoint = [self.queueParser.floatParameter[i] for i in range(1,4)]
                                    y_value = self.robotAction.calculate_y_value(self.homePoint[2])
                                    self.sendRobotPaintingRadius(y_value)
                                    self.robotFile.writeJsonObject("Home point", self.homePoint)
                                    self.setRobotAction(self.homePoint)
                                    self.robotActionCheck = 12
                                    self.armState[3] = 1
                                    self.robotFile.writeJsonObject("Arm State", self.armState)
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_FREQUENCY:
                                    self.robotAction.setFrequency(self.queueParser.intParameter[1])
                                    self.robotFile.writeJsonObject("A4988 CLK", self.queueParser.intParameter[1])
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_MSX:
                                    self.robotAction.writeA4988Clk(self.queueParser.intParameter[1])
                                    self.robotFile.writeJsonObject("A4988 MSx", self.queueParser.intParameter[1])
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_ENABLE:
                                    self.robotAction.setArmEnable(self.queueParser.intParameter[1])
                                    if self.queueParser.intParameter[1] == 0:
                                        if self.armState[0] == 0:
                                            self.robotActionCheck = 1
                                        else:
                                            self.robotActionCheck = 2
                                    else:
                                        self.robotActionCheck = 0
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_SERVO_INDEX:
                                    self.queueAction.put(cmdArray[i])
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_SENSOR_POINT:
                                    if self.robotActionCheck in (1, 2, 3, 12):
                                        self.queueAction.put(cmdArray[i])
                                        if self.armState[0] == 1 and self.armState[1] == 1 and self.armState[2] == 1 and self.armState[3] == 1:
                                            self.robotActionCheck = 12
                                        else:
                                            self.robotActionCheck = 3
                                    else:
                                        self.safetyOperationInspection()
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_CALIBRATION_START:
                                    self.queueAction.put(cmdArray[i])
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_CALIBRATION_POINT:
                                    self.queueAction.put(cmdArray[i])
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_CALIBRATION_END:
                                    self.queueAction.put(cmdArray[i])
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_QUERY:
                                    if self.queueParser.intParameter[1] == 1:
                                        self.setThreadingFeedbackState(True)
                                    elif self.queueParser.intParameter[1] == 0:
                                        self.setThreadingFeedbackState(False)
                                elif self.queueParser.commandArray[1] == self.cmd.ARM_STOP:
                                    self.threadings_state = 2
                                    self.receiveData = ""
                                    self.robotAction.setArmEnable(1)
                            else:
                                print("main.py, The received command was incomplete.")
                            self.queueParser.clearParameters()
                    else:
                        print("main.py, The received data is incomplete.")
        except SystemExit:
            pass

    def threadingRobotAction(self):
        while True:
            if self.queueAction.len() > 0:
                data = self.queueAction.get()
                self.queueActionParser.parser(data)
                if self.queueActionParser.commandArray[0] == self.cmd.MOVE_ACTION:
                    if self.queueActionParser.intParameter[0] == 0 or self.queueActionParser.intParameter[0] == 1:
                        x_index = None
                        y_index = None
                        z_index = None
                        if self.cmd.AXIS_X_ACTION in self.queueActionParser.commandArray:
                            x_index = self.queueActionParser.commandArray.index(self.cmd.AXIS_X_ACTION)
                            self.last_pozition[0] = self.queueActionParser.floatParameter[x_index]
                        if self.cmd.AXIS_Y_ACTION in self.queueActionParser.commandArray:
                            y_index = self.queueActionParser.commandArray.index(self.cmd.AXIS_Y_ACTION)
                            self.last_pozition[1] = self.queueActionParser.floatParameter[y_index]
                        if self.cmd.AXIS_Z_ACTION in self.queueActionParser.commandArray:
                            z_index = self.queueActionParser.commandArray.index(self.cmd.AXIS_Z_ACTION)
                            self.last_pozition[2] = self.queueActionParser.floatParameter[z_index]
                        try:
                            target_angle = self.robotAction.coordinateToAngle(self.last_pozition)
                            limitAngle = 180 - target_angle[1] - target_angle[2]
                            if self.robotAction.arm_limit_angle1[0] < limitAngle < self.robotAction.arm_limit_angle1[1]:
                                if self.robotAction.arm_limit_angle2[0] < target_angle[1] < self.robotAction.arm_limit_angle2[1] and self.robotAction.arm_limit_angle3[0] < target_angle[2] < self.robotAction.arm_limit_angle3[1]:
                                    data, circle_axis_1, circle_axis_2 = self.robotAction.calculate_valid_axis(self.robotAction.last_axis, self.last_pozition, 80)
                                    if data[0] == 1:
                                        if data[1] == 2:
                                            self.robotAction.moveStepMotorToTargetAxis(circle_axis_1)
                                            self.robotAction.moveStepMotorToTargetAxis(circle_axis_2, 1)
                                            self.robotAction.moveStepMotorToTargetAxis(self.last_pozition)
                                        elif data[1] == 1:
                                            self.robotAction.moveStepMotorToTargetAxis(circle_axis_1)
                                            self.robotAction.moveStepMotorToTargetAxis(self.last_pozition)
                                        else:
                                            self.robotAction.moveStepMotorToTargetAxis(self.last_pozition)
                                    else:
                                        self.robotAction.moveStepMotorToTargetAxis(self.last_pozition)
                        except Exception as e:
                            print("threadingRobotAction move error:", e)
                elif self.queueActionParser.commandArray[0] == self.cmd.CUSTOM_ACTION:
                    if self.queueActionParser.intParameter[0] == 10:
                        if self.queueActionParser.intParameter[1] == 0:
                            self.robotAction.setArmToSensorPoint(0)
                            if self.armState[0] == 0:
                                self.armState[0] = 1
                                self.robotFile.writeJsonObject("Arm State", self.armState)
                            self.robotActionCheck = 2
                        elif self.queueActionParser.intParameter[1] == 1:
                            self.robotAction.setArmToSensorPoint(1)
                            if self.armState[0] == 1 and self.armState[1] == 1 and self.armState[2] == 1 and self.armState[3] == 1:
                                self.robotActionCheck = 12
                            else:
                                self.robotActionCheck = 3
                    elif self.queueActionParser.intParameter[0] == 9:
                        index = self.queueActionParser.intParameter[1]
                        original_angle = self.queueActionParser.intParameter[2]
                        range_angle = self.robotAction.constrain(original_angle, 0, 150)
                        offset_angle = self.robotAction.map(range_angle, 0, 150, 10, 160)
                        self.robotServo.setServoAngle(index, offset_angle)

    def threadingRobotLed(self):
        
        while True:
            if self.queueLed.len() > 0:
                self.queueLed.clear()
            time.sleep(0.1)

    def threadingRobotBuzzer(self):
        while True:
            if self.queueBuzzer.len() > 0:
                data = self.queueBuzzer.get()
                self.queueBuzzerParser.parser(data)
                if self.queueBuzzerParser.intParameter[1] != 0:
                    self.robotBuzzer.buzzerRun(self.queueBuzzerParser.intParameter[1])
                elif self.queueBuzzerParser.intParameter[1] == 0:
                    if len(self.queueBuzzerParser.intParameter) == 2:
                        self.robotBuzzer.buzzerRun(self.queueBuzzerParser.intParameter[1])
                    elif len(self.queueBuzzerParser.intParameter) == 5:
                        self.robotBuzzer.buzzerRunXms(self.queueBuzzerParser.intParameter[2], self.queueBuzzerParser.intParameter[3], self.queueBuzzerParser.intParameter[4])
            else:
                time.sleep(0.1)

    def threadingRobotActionFeedback(self):
        while True:
            if self.threadings_state == 3:
                quese_count = self.queueAction.len()
                cmd = self.cmd.CUSTOM_ACTION + str("12") + self.cmd.DECOLLATOR_CHAR + self.cmd.ARM_QUERY + str(quese_count) + str('\r\n')
                self.serverSend(cmd)
            time.sleep(0.1)

    def threadingCheckServer(self):
        while self.threadings_state != 4:
            if self.threadings_state == 0:
                self.threadings_state = 3
                self.turn_on_server()
                time.sleep(0.1)
            elif self.threadings_state == 1:
                self.threadings_state = 0
                self.turn_off_server()
                time.sleep(0.1)
            elif self.threadings_state == 2:
                self.threadings_state = 4
                self.turn_off_server()
                break
            elif self.threadings_state == 3:
                time.sleep(0.5)
            else:
                pass
        print("main.py, The robot arm stops running, please press ctrl+c to exit.")


if __name__ == '__main__':
    arm = ArmServer()
    try:
        print("Please use your computer or mobile phone to connect the robot arm.")
    except KeyboardInterrupt:
        pass