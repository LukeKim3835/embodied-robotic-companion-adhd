"""
Writes 3 files per session into a per-participant folder:
    logs/<participant>_<condition>_<timestamp>/
        ├── meta.json       study metadata + baseline
        ├── frames.csv      per-frame state (for time-series analysis)
        └── events.jsonl    discrete events (state transitions, robot actions, warnings)
"""

import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path


# Columns written to frames.csv (order matters for readability)
FRAME_FIELDS = [
    "frame_idx", "t_mono", "t_wall", "fps",
    "pose_present",
    "raw_state", "smoothed_state", "main_state", "reason",
    "face_offset", "torso_offset",
    "looking_away", "looking_down", "torso_shift",
    "hands_midline", "possible_phone_posture",
    "detector_enabled",
    "phone_visible", "phone_box_count", "phone_confirmed_elapsed",
    "absent_elapsed",
    "robot_pose_state", "robot_busy",
]


def _now_wall_iso():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


class SessionLogger:
    """Single-session logger. One instance per participant run."""

    def __init__(self, participant_id, condition, log_dir="logs", note="",
                 code_version="integrated_monitor_robot_v3.2"):
        ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        folder_name = f"{participant_id}_{condition}_{ts}"
        self.session_dir = Path(log_dir) / folder_name
        self.session_dir.mkdir(parents=True, exist_ok=True)

        # meta.json
        self.meta = {
            "participant_id": participant_id,
            "condition": condition,
            "note": note,
            "code_version": code_version,
            "start_wall_time": _now_wall_iso(),
            "folder": str(self.session_dir.resolve()),
        }
        self._write_meta()

        # frames.csv
        self.frames_path = self.session_dir / "frames.csv"
        self._frames_fh = open(self.frames_path, "w", newline="", encoding="utf-8")
        self._frames_writer = csv.DictWriter(
            self._frames_fh,
            fieldnames=FRAME_FIELDS,
            extrasaction="ignore",  # silently drop unknown keys
        )
        self._frames_writer.writeheader()
        self._frames_fh.flush()

        # events.jsonl
        self.events_path = self.session_dir / "events.jsonl"
        self._events_fh = open(self.events_path, "w", encoding="utf-8")

        # transition tracking (for auto-derived events)
        self._prev = {
            "main_state":       None,
            "smoothed_state":   None,
            "phone_visible":    False,
            "pose_present":     None,
            "robot_pose_state": None,
            "warning_start_t":  None,
            "warning_reason":   None,
        }

        self._closed = False
        self.log_event("session_start",
                       participant=participant_id,
                       condition=condition,
                       note=note,
                       code_version=code_version)

  
    def _write_meta(self):
        with open(self.session_dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(self.meta, f, indent=2, ensure_ascii=False, default=str)

    def update_meta(self, **fields):
        self.meta.update(fields)
        self._write_meta()

    def write_baseline(self, baseline):
        """Call right after baseline calibration completes."""
        self.meta["baseline"] = {k: float(v) for k, v in baseline.items()}
        self._write_meta()
        self.log_event("baseline_complete", baseline=self.meta["baseline"])

  
    def log_frame(self, **fields):
        if self._closed:
            return

        fields.setdefault("t_wall", _now_wall_iso())
        self._frames_writer.writerow(fields)
        self._frames_fh.flush()

        t = fields.get("t_mono")

        # main_state transition + warning triggered/resolved
        cur_main = fields.get("main_state")
        prev_main = self._prev["main_state"]
        if cur_main != prev_main:
            self.log_event("state_change",
                           t_mono=t,
                           key="main_state",
                           from_state=prev_main,
                           to_state=cur_main,
                           reason=fields.get("reason"))

            if cur_main == "warning":
                self._prev["warning_start_t"] = t
                self._prev["warning_reason"]  = fields.get("reason")
                self.log_event("warning_triggered",
                               t_mono=t,
                               reason=fields.get("reason"))
            elif prev_main == "warning":
                dur = None
                if self._prev["warning_start_t"] is not None and t is not None:
                    dur = float(t - self._prev["warning_start_t"])
                self.log_event("warning_resolved",
                               t_mono=t,
                               to_state=cur_main,
                               duration=dur,
                               was_reason=self._prev["warning_reason"])
                self._prev["warning_start_t"] = None
                self._prev["warning_reason"]  = None

        # smoothed_state transition
        cur_smoothed = fields.get("smoothed_state")
        prev_smoothed = self._prev["smoothed_state"]
        if cur_smoothed != prev_smoothed:
            self.log_event("smoothed_change",
                           t_mono=t,
                           from_state=prev_smoothed,
                           to_state=cur_smoothed)

        # phone detect / lost
        cur_phone = bool(fields.get("phone_visible", False))
        prev_phone = self._prev["phone_visible"]
        if cur_phone and not prev_phone:
            self.log_event("phone_detected",
                           t_mono=t,
                           box_count=fields.get("phone_box_count"))
        elif prev_phone and not cur_phone:
            self.log_event("phone_lost", t_mono=t)

        
        cur_pose = bool(fields.get("pose_present", False))
        prev_pose = self._prev["pose_present"]
        if prev_pose is not None and cur_pose != prev_pose:
            self.log_event("user_returned" if cur_pose else "user_lost",
                           t_mono=t)

        
        cur_robot = fields.get("robot_pose_state")
        prev_robot = self._prev["robot_pose_state"]
        if cur_robot != prev_robot:
            self.log_event("robot_pose_change",
                           t_mono=t,
                           from_state=prev_robot,
                           to_state=cur_robot)

        # update
        self._prev["main_state"]       = cur_main
        self._prev["smoothed_state"]   = cur_smoothed
        self._prev["phone_visible"]    = cur_phone
        self._prev["pose_present"]     = cur_pose
        self._prev["robot_pose_state"] = cur_robot

    
    def log_event(self, event_type, **fields):
        if self._closed:
            return
        rec = {"event": event_type, "t_wall": _now_wall_iso()}
        rec.update(fields)
        self._events_fh.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        self._events_fh.flush()

    
    def close(self):
        if self._closed:
            return
        try:
            self.log_event("session_end")
        except Exception:
            pass
        self._closed = True
        self.meta["end_wall_time"] = _now_wall_iso()
        self._write_meta()
        try:
            self._frames_fh.close()
        except Exception:
            pass
        try:
            self._events_fh.close()
        except Exception:
            pass