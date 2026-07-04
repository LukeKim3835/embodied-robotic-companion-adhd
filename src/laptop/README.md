# Laptop (vision side)

Runs on the user's laptop. Captures the webcam feed, detects distraction, and sends robot
commands to the Raspberry Pi over TCP.

- `integrated_monitor_robot_v3.2.py` — main vision pipeline (MediaPipe BlazePose +
  YOLOv11n) and distraction state machine. Sends robot commands to the Pi.
- `session_logger.py` — per-frame logging of pose state, warnings, and robot events.
  Writes `frames.csv`, `events.jsonl`, and `meta.json` per session.

## Models

Model weights are **not** committed (see `.gitignore`). Before running:

- **YOLOv11n** (`yolo11n.pt`) — downloaded automatically by Ultralytics on first run.
- **MediaPipe pose landmarker** — download `pose_landmarker_full.task` and place it at
  `models/pose_landmarker_full.task` (path relative to the repo root, where the script
  expects it). Available from the MediaPipe Pose Landmarker model card:
  https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

## Running

```bash
# from the repository root, with models/ in place:
python src/laptop/integrated_monitor_robot_v3.2.py --participant P01 --condition robot_on
python src/laptop/integrated_monitor_robot_v3.2.py --participant dev --condition robot_on --no-log
```

Set `ROBOT_IP` at the top of the script to your Pi's address.
