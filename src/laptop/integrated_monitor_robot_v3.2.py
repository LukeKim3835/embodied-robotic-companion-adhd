"""
integrated_monitor_robot_v3.py  —  v3.2
───────────────────────────────────────
v2 감지/로봇 로직 + 유저 스터디 로깅 훅 + latency/sensitivity 튜닝.

Changelog:
  v3.0 — initial study version with logging hooks, task markers (b/e)
  v3.1 — fast robot look (skip via-points): 5.6s → 2.0s latency
  v3.2 — remove looking_down warning (FP on laptop task);
         looking_away 1.0s → 2.0s; gimbal cooldown 10s → 15s

Usage:
    python integrated_monitor_robot_v3.py --participant P01 --condition robot_on
    python integrated_monitor_robot_v3.py --participant P02 --condition robot_off --note "noisy room"
    python integrated_monitor_robot_v3.py --participant dev --condition robot_on --no-log   # dev mode

Runtime keys (during monitoring window):
    b : mark task_start    (누르는 순간을 "proofreading 시작"으로 기록)
    e : mark task_end      (누르는 순간을 "proofreading 종료"로 기록)
    q / ESC : quit
"""

import argparse
import time
import socket
import threading
from collections import deque
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ultralytics import YOLO

from session_logger import SessionLogger



MODEL_PATH = Path("models/pose_landmarker_full.task")

ROBOT_IP   = "172.20.10.3"
ROBOT_PORT = 5000

YOLO_MODEL_NAME = "yolo11n.pt"
CAMERA_INDEX    = 0


LOOK_CMD         = "G0 X140.0 Y90.0 Z265.0"
SENSOR_POINT_CMD = "G0 X0.0 Y160.0 Z220.0"
LOOK_VIA1_CMD    = "G0 X43.0  Y143.0 Z235.0"
LOOK_VIA2_CMD    = "G0 X107.0 Y116.0 Z250.0"


NOSE           = 0
LEFT_SHOULDER  = 11
RIGHT_SHOULDER = 12
LEFT_WRIST     = 15
RIGHT_WRIST    = 16

POSE_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 7),
    (0, 4), (4, 5), (5, 6), (6, 8),
    (9, 10),
    (11, 12),
    (11, 13), (13, 15),
    (12, 14), (14, 16),
    (15, 17), (15, 19), (15, 21),
    (16, 18), (16, 20), (16, 22),
    (17, 19), (18, 20),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27),
    (24, 26), (26, 28),
    (27, 29), (29, 31),
    (28, 30), (30, 32),
    (27, 31), (28, 32),
]



VISIBILITY_THRESHOLD  = 0.5
BASELINE_SECONDS      = 10.0
BASELINE_MAX_SECONDS  = 30.0
BASELINE_MIN_SAMPLES  = 5

FACE_OFFSET_THRESHOLD  = 0.18
TORSO_OFFSET_THRESHOLD = 0.10

LOOK_DOWN_DELTA_Y          = 0.02
TORSO_SHIFT_DELTA_X        = 0.08
TORSO_SHIFT_DELTA_Y        = 0.08
WRIST_TO_TORSO_X_THRESHOLD = 0.16
WRIST_BELOW_SHOULDER_DELTA = 0.02

LOOKING_AWAY_SECONDS    = 2.0   
LOOKING_DOWN_SECONDS    = 2.0   
TORSO_SHIFT_SECONDS     = 2.0
POSSIBLE_PHONE_SECONDS  = 0.7
CONFIRMED_PHONE_SECONDS = 0.5
ABSENT_SECONDS          = 1.5

DETECTOR_CONF           = 0.25
DETECTOR_EVERY_N_FRAMES = 5
PHONE_BOX_MARGIN        = 50

SMOOTHING_WINDOW  = 8

FAST_LOOK_MODE    = True   
ROBOT_LOOK_WAIT   = 2.0    # wait after final LOOK_CMD 
ROBOT_RETURN_WAIT = 3.0    # wait after SENSOR_POINT_CMD 
SENSOR_POINT_WAIT = 3.0   
GIMBAL_COOLDOWN  = 15.0   

_robot_lock       = threading.Lock()
_robot_busy       = False
_robot_pose_state = "SENSOR_POINT"

prev_main_state  = "focused"
gimbal_triggered = False
last_gimbal_time = 0.0



def get_robot_busy() -> bool:
    with _robot_lock:
        return _robot_busy

def get_robot_pose_state() -> str:
    with _robot_lock:
        return _robot_pose_state

def _set_robot_busy(val: bool):
    global _robot_busy
    with _robot_lock:
        _robot_busy = val

def _set_robot_pose_state(val: str):
    global _robot_pose_state
    with _robot_lock:
        _robot_pose_state = val



def get_visibility(lm):
    return getattr(lm, "visibility", 1.0)

def xy(lm):
    return np.array([lm.x, lm.y], dtype=np.float32)

def draw_pose_landmarks(frame, detection_result):
    if not detection_result.pose_landmarks:
        return frame
    annotated = frame.copy()
    h, w = annotated.shape[:2]
    for pose_landmarks in detection_result.pose_landmarks:
        for start_idx, end_idx in POSE_CONNECTIONS:
            start = pose_landmarks[start_idx]
            end   = pose_landmarks[end_idx]
            if (get_visibility(start) < VISIBILITY_THRESHOLD
                    or get_visibility(end) < VISIBILITY_THRESHOLD):
                continue
            x1, y1 = int(start.x * w), int(start.y * h)
            x2, y2 = int(end.x * w), int(end.y * h)
            cv2.line(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        for lm in pose_landmarks:
            if get_visibility(lm) < VISIBILITY_THRESHOLD:
                continue
            px, py = int(lm.x * w), int(lm.y * h)
            cv2.circle(annotated, (px, py), 4, (0, 255, 255), -1)
    return annotated


def phone_class_ids(model):
    ids = []
    names = model.names
    iterable = names.items() if isinstance(names, dict) else enumerate(names)
    for class_id, name in iterable:
        if str(name).strip().lower() in {"cell phone", "cellphone", "mobile phone", "phone"}:
            ids.append(int(class_id))
    return ids

def detect_phone_boxes(frame, model, phone_ids):
    if not phone_ids:
        return []
    result = model.predict(source=frame, verbose=False, conf=DETECTOR_CONF, classes=phone_ids)[0]
    boxes_out = []
    if result.boxes is None or len(result.boxes) == 0:
        return boxes_out
    xyxy  = result.boxes.xyxy.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy()
    clss  = result.boxes.cls.cpu().numpy().astype(int)
    for box, conf, cls_id in zip(xyxy, confs, clss):
        x1, y1, x2, y2 = box.astype(int)
        boxes_out.append({
            "xyxy":   (x1, y1, x2, y2),
            "conf":   float(conf),
            "cls_id": int(cls_id),
            "label":  str(model.names[int(cls_id)]),
            "center": ((x1 + x2) // 2, (y1 + y2) // 2),
        })
    return boxes_out

def wrist_points_px(landmarks, w, h):
    points = []
    for idx in (LEFT_WRIST, RIGHT_WRIST):
        lm = landmarks[idx]
        if get_visibility(lm) >= VISIBILITY_THRESHOLD:
            points.append((int(lm.x * w), int(lm.y * h)))
    return points

def is_phone_near_wrist(phone_boxes, wrist_pts, margin=PHONE_BOX_MARGIN):
    if not phone_boxes or not wrist_pts:
        return False
    for box in phone_boxes:
        x1, y1, x2, y2 = box["xyxy"]
        x1 -= margin; y1 -= margin; x2 += margin; y2 += margin
        for wx, wy in wrist_pts:
            if x1 <= wx <= x2 and y1 <= wy <= y2:
                return True
    return False



# robot control

_robot_socket = None

def _connect_robot(retries: int = 5, delay: float = 2.0) -> bool:
    global _robot_socket
    for attempt in range(1, retries + 1):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((ROBOT_IP, ROBOT_PORT))
            sock.settimeout(None)
            _robot_socket = sock
            print(f"Robot connected (attempt {attempt}).")
            return True
        except Exception as e:
            print(f"Robot connect failed (attempt {attempt}/{retries}): {e}")
            _robot_socket = None
            if attempt < retries:
                time.sleep(delay)
    return False

def _send(cmd: str):
    global _robot_socket
    if _robot_socket is None:
        print("Socket is None, attempting reconnect...")
        if not _connect_robot():
            print("Reconnect failed, skipping:", cmd)
            return
    try:
        _robot_socket.sendall((cmd.strip() + "\r\n").encode("utf-8"))
        print("Sent:", cmd)
    except Exception as e:
        print("Send failed:", e)
        _robot_socket = None

def _send_to_sensor_point():
    with _robot_lock:
        global _robot_busy
        if _robot_busy:
            return
        _robot_busy = True
    try:
        _send(SENSOR_POINT_CMD)
        time.sleep(SENSOR_POINT_WAIT)
        _set_robot_pose_state("SENSOR_POINT")
    except Exception as e:
        print("Sensor point move failed:", e)
    finally:
        _set_robot_busy(False)

def _send_cmd_blocking(target_cmd: str, target_state: str):
    with _robot_lock:
        global _robot_busy
        if _robot_busy:
            return
        _robot_busy = True
    try:
        _send(target_cmd)
        time.sleep(ROBOT_RETURN_WAIT)
        _set_robot_pose_state(target_state)
    except Exception as e:
        print(f"Robot move failed ({target_state}):", e)
    finally:
        _set_robot_busy(False)

def move_robot_to_sensor_point():
    _send_to_sensor_point()

def _send_cmd_via(target_cmd: str, wait: float = 0.8):
    _send(target_cmd)
    time.sleep(wait)

def move_robot_look(logger=None, t_mono=None):
    """Move arm to LOOK pose. FAST_LOOK_MODE skips via-points (~3.6s faster).
    Logs robot_motion_start (when arm begins moving) and robot_motion_complete
    (when pose state becomes LOOKING) for precise latency analysis."""
    with _robot_lock:
        global _robot_busy
        if _robot_busy:
            return
        _robot_busy = True
    try:
        if logger is not None:
            logger.log_event("robot_motion_start",
                             target="LOOKING",
                             fast_mode=FAST_LOOK_MODE,
                             t_mono=t_mono)
        _send("S6 Q800")
        if not FAST_LOOK_MODE:
            _send_cmd_via(LOOK_VIA1_CMD, wait=0.8)
            _send_cmd_via(LOOK_VIA2_CMD, wait=0.8)
        _send(LOOK_CMD)
        time.sleep(ROBOT_LOOK_WAIT)
        _set_robot_pose_state("LOOKING")
        _send("GIMBAL_SHAKE_NO")
        if logger is not None:
            logger.log_event("robot_motion_complete", target="LOOKING")
    except Exception as e:
        print("move_robot_look failed:", e)
    finally:
        _set_robot_busy(False)

def move_robot_to_sensor_point_with_nod():
    with _robot_lock:
        global _robot_busy
        if _robot_busy:
            return
        _robot_busy = True
    try:
        _send("S6 Q1000")
        _send(SENSOR_POINT_CMD)
        time.sleep(SENSOR_POINT_WAIT)
        _set_robot_pose_state("SENSOR_POINT")
        _send("S6 Q1000")
        _send("GIMBAL_NOD_YES")
    except Exception as e:
        print("move_robot_to_sensor_point_with_nod failed:", e)
    finally:
        _set_robot_busy(False)



# CLASSIFICATION
def classify_focus_and_phone_posture(landmarks, baseline):
    nose = landmarks[NOSE]
    ls   = landmarks[LEFT_SHOULDER]
    rs   = landmarks[RIGHT_SHOULDER]
    lw   = landmarks[LEFT_WRIST]
    rw   = landmarks[RIGHT_WRIST]

    needed = [nose, ls, rs]
    if any(get_visibility(lm) < VISIBILITY_THRESHOLD for lm in needed):
        return {"state": "unknown", "looking_away": False, "looking_down": False,
                "torso_shift": False, "hands_midline": False,
                "possible_phone_posture": False, "face_offset": 0.0, "torso_offset": 0.0}

    nose_xy      = xy(nose)
    ls_xy        = xy(ls)
    rs_xy        = xy(rs)
    shoulder_mid = (ls_xy + rs_xy) / 2.0
    shoulder_width = max(np.linalg.norm(ls_xy - rs_xy), 1e-6)

    face_offset  = abs(nose_xy[0] - shoulder_mid[0]) / shoulder_width
    torso_offset = abs(shoulder_mid[0] - 0.5)
    looking_away = face_offset > FACE_OFFSET_THRESHOLD or torso_offset > TORSO_OFFSET_THRESHOLD

    looking_down = nose_xy[1] > baseline["nose_y"] + LOOK_DOWN_DELTA_Y
    torso_shift  = (
        abs(shoulder_mid[0] - baseline["shoulder_mid_x"]) > TORSO_SHIFT_DELTA_X
        or abs(shoulder_mid[1] - baseline["shoulder_mid_y"]) > TORSO_SHIFT_DELTA_Y
    )

    hands_midline = False
    for wrist in (lw, rw):
        if get_visibility(wrist) >= VISIBILITY_THRESHOLD:
            wrist_xy = xy(wrist)
            if (abs(wrist_xy[0] - shoulder_mid[0]) < WRIST_TO_TORSO_X_THRESHOLD
                    and wrist_xy[1] > shoulder_mid[1] + WRIST_BELOW_SHOULDER_DELTA):
                hands_midline = True

    possible_phone_posture = looking_down and (hands_midline or torso_shift)

    if possible_phone_posture:
        state = "possible_phone_posture"
    elif looking_away:
        state = "looking_away"
    elif looking_down:
        state = "looking_down"
    elif torso_shift:
        state = "torso_shift"
    else:
        state = "focused"

    return {"state": state, "looking_away": looking_away, "looking_down": looking_down,
            "torso_shift": torso_shift, "hands_midline": hands_midline,
            "possible_phone_posture": possible_phone_posture,
            "face_offset": float(face_offset), "torso_offset": float(torso_offset)}




def collect_baseline(cap, landmarker, mono_start: float):
    start   = time.monotonic()
    samples = []
    consecutive_read_failures = 0
    MAX_READ_FAILURES = 30

    while True:
        ok, frame = cap.read()
        if not ok:
            consecutive_read_failures += 1
            if consecutive_read_failures >= MAX_READ_FAILURES:
                print("Calibration failed: repeated camera read errors")
                return None
            continue
        consecutive_read_failures = 0

        timestamp_ms = int((time.monotonic() - mono_start) * 1000)
        rgb      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result   = landmarker.detect_for_video(mp_image, timestamp_ms)

        preview   = draw_pose_landmarks(frame, result)
        elapsed   = time.monotonic() - start
        remaining = max(0.0, BASELINE_SECONDS - elapsed)

        cv2.putText(preview, f"BASELINE CALIBRATING: {remaining:.1f}s",
                    (20, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(preview, "Sit in your normal focused posture",
                    (20, 65),  cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(preview, f"Samples: {len(samples)}",
                    (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
        cv2.imshow("Integrated Monitor Robot", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            return None

        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]
            nose = landmarks[NOSE]
            ls   = landmarks[LEFT_SHOULDER]
            rs   = landmarks[RIGHT_SHOULDER]
            if (get_visibility(nose) >= VISIBILITY_THRESHOLD
                    and get_visibility(ls) >= VISIBILITY_THRESHOLD
                    and get_visibility(rs) >= VISIBILITY_THRESHOLD):
                nose_xy      = xy(nose)
                ls_xy        = xy(ls)
                rs_xy        = xy(rs)
                shoulder_mid = (ls_xy + rs_xy) / 2.0
                samples.append({
                    "nose_y":         float(nose_xy[1]),
                    "shoulder_mid_x": float(shoulder_mid[0]),
                    "shoulder_mid_y": float(shoulder_mid[1]),
                    "shoulder_width": float(np.linalg.norm(ls_xy - rs_xy)),
                })

        if elapsed >= BASELINE_SECONDS and len(samples) >= 20:
            break
        if elapsed >= BASELINE_MAX_SECONDS:
            if len(samples) >= BASELINE_MIN_SAMPLES:
                print(f"Calibration: max time exceeded, proceeding with {len(samples)} samples")
                break
            else:
                print(f"Calibration failed: {len(samples)} samples (min {BASELINE_MIN_SAMPLES} required)")
                return None

    keys = samples[0].keys()
    return {k: float(np.mean([s[k] for s in samples])) for k in keys}



def parse_args():
    p = argparse.ArgumentParser(description="Lamp robot attention monitor with user-study logging")
    p.add_argument("--participant", "-p", required=True,
                   help="Participant ID (e.g. P01, dev)")
    p.add_argument("--condition",   "-c", required=True,
                   choices=["robot_on", "robot_off"],
                   help="Study condition")
    p.add_argument("--note",        "-n", default="",
                   help="Optional note (e.g. 'noisy room')")
    p.add_argument("--log-dir",     default="logs",
                   help="Where to put session folders (default: ./logs)")
    p.add_argument("--no-log",      action="store_true",
                   help="Disable logging (dev/testing mode)")
    return p.parse_args()



def main():
    global prev_main_state, gimbal_triggered, last_gimbal_time

    args = parse_args()

    # ── Logger ──
    logger = None
    if not args.no_log:
        logger = SessionLogger(
            participant_id=args.participant,
            condition=args.condition,
            log_dir=args.log_dir,
            note=args.note,
        )
        print(f"[LOG] Session dir: {logger.session_dir}")

    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")

    yolo_model = YOLO(YOLO_MODEL_NAME)
    phone_ids  = phone_class_ids(yolo_model)

    base_options = python.BaseOptions(model_asset_path=str(MODEL_PATH))
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        output_segmentation_masks=False,
    )

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")

    mono_start = time.monotonic()
    prev_time  = time.monotonic()
    frame_idx  = 0

    # Task timing markers b/e
    task_start_t = None
    task_end_t   = None

    looking_away_start    = None
    looking_down_start    = None
    torso_shift_start     = None
    phone_posture_start   = None
    confirmed_phone_start = None
    absent_start          = None
    state_history         = deque(maxlen=SMOOTHING_WINDOW)
    last_phone_boxes      = []

    with vision.PoseLandmarker.create_from_options(options) as landmarker:

        if not _connect_robot():
            if logger: logger.log_event("error", msg="robot_connect_failed")
            cap.release()
            cv2.destroyAllWindows()
            if logger: logger.close()
            return

        if logger: logger.log_event("robot_connected")

        baseline = collect_baseline(cap, landmarker, mono_start)
        if baseline is None:
            if logger: logger.log_event("error", msg="baseline_failed")
            cap.release()
            cv2.destroyAllWindows()
            if logger: logger.close()
            return

        if logger: logger.write_baseline(baseline)

        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from webcam.")
                break

            frame_idx   += 1
            timestamp_ms = int((time.monotonic() - mono_start) * 1000)
            rgb      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result   = landmarker.detect_for_video(mp_image, timestamp_ms)
            annotated = draw_pose_landmarks(frame, result)

            main_state        = "focused"
            reason_text       = ""
            detector_enabled  = False
            phone_near_hand   = False
            phone_visible     = False
            confirmed_elapsed = 0.0
            absent_elapsed    = 0.0
            robot_busy        = get_robot_busy()
            robot_pose_state  = get_robot_pose_state()
            now               = time.monotonic()

            # Fields we want in the frame log, initialized with defaults
            log_raw_state              = "no_pose"
            log_smoothed_state         = "no_pose"
            log_face_offset            = 0.0
            log_torso_offset           = 0.0
            log_looking_away           = False
            log_looking_down           = False
            log_torso_shift            = False
            log_hands_midline          = False
            log_possible_phone_posture = False

            if result.pose_landmarks:
                absent_start = None
                landmarks    = result.pose_landmarks[0]
                posture      = classify_focus_and_phone_posture(landmarks, baseline)
                state_history.append(posture["state"])

                # Capture for logging
                log_raw_state              = posture["state"]
                log_face_offset            = posture["face_offset"]
                log_torso_offset           = posture["torso_offset"]
                log_looking_away           = bool(posture["looking_away"])
                log_looking_down           = bool(posture["looking_down"])
                log_torso_shift            = bool(posture["torso_shift"])
                log_hands_midline          = bool(posture["hands_midline"])
                log_possible_phone_posture = bool(posture["possible_phone_posture"])

                current_state = posture["state"]
                focused_count = sum(1 for s in state_history if s == "focused")
                away_count    = sum(1 for s in state_history if s == "looking_away")
                down_count    = sum(1 for s in state_history if s == "looking_down")
                torso_count   = sum(1 for s in state_history if s == "torso_shift")

                half = max(2, len(state_history) // 2)
                if away_count >= half:
                    smoothed_state = "looking_away"
                elif down_count >= half:
                    smoothed_state = "looking_down"
                elif torso_count >= half:
                    smoothed_state = "torso_shift"
                elif focused_count >= half:
                    smoothed_state = "focused"
                else:
                    smoothed_state = current_state

                log_smoothed_state = smoothed_state

                
                if smoothed_state == "looking_away":
                    if looking_away_start is None:
                        looking_away_start = now
                    if (now - looking_away_start) >= LOOKING_AWAY_SECONDS:
                        main_state = "warning"; reason_text = "looking away"
                    else:
                        main_state = "suspicious"; reason_text = "possible looking away"
                else:
                    looking_away_start = None

                # ── looking_down: DETECTED but does NOT trigger warning (v3.2) ──
                if smoothed_state == "looking_down":
                    if looking_down_start is None:
                        looking_down_start = now
                    # No warning/suspicious assignment — just track for logs
                else:
                    looking_down_start = None

                if smoothed_state == "torso_shift":
                    if torso_shift_start is None:
                        torso_shift_start = now
                    if (now - torso_shift_start) >= TORSO_SHIFT_SECONDS:
                        main_state = "warning"; reason_text = "posture shift"
                    else:
                        main_state = "suspicious"; reason_text = "possible posture shift"
                else:
                    torso_shift_start = None

               
                detector_enabled = True
                if phone_ids and frame_idx % DETECTOR_EVERY_N_FRAMES == 0:
                    last_phone_boxes = detect_phone_boxes(frame, yolo_model, phone_ids)

                phone_visible = len(last_phone_boxes) > 0

                if phone_visible:
                    if confirmed_phone_start is None:
                        confirmed_phone_start = now
                    confirmed_elapsed = now - confirmed_phone_start
                    if confirmed_elapsed >= CONFIRMED_PHONE_SECONDS:
                        main_state = "warning"; reason_text = "phone detected"
                else:
                    confirmed_phone_start = None
                    confirmed_elapsed     = 0.0

            else:
                state_history.clear()
                looking_away_start = looking_down_start = torso_shift_start = None
                confirmed_phone_start = None

                
                detector_enabled = True
                if phone_ids and frame_idx % DETECTOR_EVERY_N_FRAMES == 0:
                    last_phone_boxes = detect_phone_boxes(frame, yolo_model, phone_ids)
                if not last_phone_boxes:
                    last_phone_boxes = []
                phone_visible = len(last_phone_boxes) > 0

                if absent_start is None:
                    absent_start = now
                absent_elapsed = now - absent_start
                if absent_elapsed >= ABSENT_SECONDS:
                    main_state = "warning"; reason_text = "user absent"
                else:
                    main_state = "suspicious"; reason_text = "pose lost"

            state_changed = (main_state != prev_main_state)
            if state_changed:
                gimbal_triggered = False

            gimbal_cooled = (now - last_gimbal_time) >= GIMBAL_COOLDOWN

            
            if main_state == "warning":
                cv2.putText(annotated, "LOOK AT USER",
                            (20, 275), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

                if args.condition == "robot_on":
                    if robot_pose_state != "LOOKING" and not robot_busy:
                        if logger:
                            logger.log_event("robot_dispatch",
                                             action="LOOK",
                                             reason=reason_text,
                                             t_mono=now - mono_start)
                        threading.Thread(target=move_robot_look,
                                         args=(logger, now - mono_start),
                                         daemon=True).start()
                        gimbal_triggered = True
                        last_gimbal_time = now
                else:
                    
                    if not gimbal_triggered and robot_pose_state != "SENSOR_POINT_MOCK":
                        if logger:
                            logger.log_event("robot_dispatch_suppressed",
                                             action="LOOK",
                                             reason=reason_text,
                                             t_mono=now - mono_start,
                                             note="condition=robot_off")
                        gimbal_triggered = True
                        last_gimbal_time = now

            else:
                if args.condition == "robot_on":
                    if robot_pose_state != "SENSOR_POINT" and not robot_busy:
                        if prev_main_state == "warning" and not gimbal_triggered and gimbal_cooled:
                            if logger:
                                logger.log_event("robot_dispatch",
                                                 action="SENSOR_POINT_WITH_NOD",
                                                 t_mono=now - mono_start)
                            threading.Thread(target=move_robot_to_sensor_point_with_nod, daemon=True).start()
                            last_gimbal_time = now
                        else:
                            if logger:
                                logger.log_event("robot_dispatch",
                                                 action="SENSOR_POINT",
                                                 t_mono=now - mono_start)
                            threading.Thread(target=move_robot_to_sensor_point, daemon=True).start()
                        gimbal_triggered = True

            prev_main_state = main_state

            for box in last_phone_boxes:
                x1, y1, x2, y2 = box["xyxy"]
                color = (0, 128, 255) if detector_enabled else (128, 128, 128)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated, f'{box["label"]} {box["conf"]:.2f}',
                            (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            current_time = time.monotonic()
            fps       = 1.0 / max(current_time - prev_time, 1e-6)
            prev_time = current_time

            
            cv2.putText(annotated, f"FPS: {fps:.1f}",
                        (20, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(annotated, f"STATE: {main_state}",
                        (20, 65),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.putText(annotated, f"REASON: {reason_text}",
                        (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            if detector_enabled:
                cv2.putText(annotated, "DETECTOR ENABLED",
                            (20, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (0, 165, 255), 3)
            if phone_visible:
                cv2.putText(annotated, f"PHONE DETECTED | timer={confirmed_elapsed:.2f}",
                            (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

            cv2.putText(annotated, f"ROBOT POSE: {robot_pose_state}",
                        (20, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            if robot_busy:
                cv2.putText(annotated, "ROBOT MOVING",
                            (20, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

            
            h_frame, w_frame = annotated.shape[:2]
            if logger is not None:
                cv2.putText(annotated, "REC",
                            (w_frame - 80, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                cv2.circle(annotated, (w_frame - 95, 22), 6, (0, 0, 255), -1)

                # Task timer
                if task_start_t is not None:
                    elapsed_task = (now - mono_start) - task_start_t if task_end_t is None \
                                   else task_end_t - task_start_t
                    mm = int(elapsed_task) // 60
                    ss = int(elapsed_task) % 60
                    label = "TASK" if task_end_t is None else "DONE"
                    # Target 7:00 — flash when past 7 min as reminder to press 'e'
                    color = (0, 200, 255)
                    if task_end_t is None and elapsed_task >= 420:
                        # blink red after 7 min to prompt researcher to end
                        color = (0, 0, 255) if int(now * 2) % 2 == 0 else (0, 165, 255)
                    cv2.putText(annotated, f"{label} {mm:02d}:{ss:02d} / 07:00",
                                (w_frame - 310, 65), cv2.FONT_HERSHEY_SIMPLEX,
                                0.75, color, 2)
                else:
                    # Task not started — persistent reminder
                    cv2.putText(annotated, "PRESS 'b' TO START TASK",
                                (w_frame - 410, 65), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 255), 2)

            cv2.putText(annotated, f"COND: {args.condition}",
                        (w_frame - 230, h_frame - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (200, 200, 200), 2)

            # ── Frame log ──
            if logger is not None:
                logger.log_frame(
                    frame_idx=frame_idx,
                    t_mono=now - mono_start,
                    fps=round(fps, 2),
                    pose_present=bool(result.pose_landmarks),
                    raw_state=log_raw_state,
                    smoothed_state=log_smoothed_state,
                    main_state=main_state,
                    reason=reason_text,
                    face_offset=round(log_face_offset, 4),
                    torso_offset=round(log_torso_offset, 4),
                    looking_away=log_looking_away,
                    looking_down=log_looking_down,
                    torso_shift=log_torso_shift,
                    hands_midline=log_hands_midline,
                    possible_phone_posture=log_possible_phone_posture,
                    detector_enabled=detector_enabled,
                    phone_visible=phone_visible,
                    phone_box_count=len(last_phone_boxes),
                    phone_confirmed_elapsed=round(confirmed_elapsed, 3),
                    absent_elapsed=round(absent_elapsed, 3),
                    robot_pose_state=robot_pose_state,
                    robot_busy=robot_busy,
                )

            cv2.imshow("Integrated Monitor Robot", annotated)
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q"):
                break

            # ── Task markers ──
            elif key == ord("b"):
                if task_start_t is None:
                    task_start_t = now - mono_start
                    if logger:
                        logger.log_event("task_start",
                                         t_mono=task_start_t,
                                         planned_duration_sec=420,
                                         announced_duration="3-5 min")
                    print(f"[TASK] start @ t_mono={task_start_t:.2f}s")
                else:
                    print(f"[TASK] already started at {task_start_t:.2f}s")

            elif key == ord("e"):
                if task_start_t is not None and task_end_t is None:
                    task_end_t = now - mono_start
                    duration = task_end_t - task_start_t
                    if logger:
                        logger.log_event("task_end",
                                         t_mono=task_end_t,
                                         duration_sec=duration)
                    print(f"[TASK] end @ t_mono={task_end_t:.2f}s "
                          f"(duration={duration:.2f}s)")
                elif task_start_t is None:
                    print("[TASK] cannot end — never started (press 'b' first)")
                else:
                    print("[TASK] already ended")

    # ── Cleanup ──
    print("Returning to sensor point...")
    try:
        if args.condition == "robot_on":
            move_robot_to_sensor_point()
    except Exception as e:
        print("Failed to return to sensor point:", e)

    cap.release()
    cv2.destroyAllWindows()

    if logger:
        logger.close()
        print(f"[LOG] Session saved to: {logger.session_dir}")


if __name__ == "__main__":
    main()
