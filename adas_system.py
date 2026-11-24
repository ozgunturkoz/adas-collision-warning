import cv2
import numpy as np
import serial
import time
import csv
import os
import math
import threading
from collections import deque
from datetime import datetime
from ultralytics import YOLO
import torch
import sys

# ================================================================
# PLATFORM-INDEPENDENT BEEP
# ================================================================
def _beep(freq=1000, dur_ms=200):
    try:
        if sys.platform == "win32":
            import winsound
            winsound.Beep(freq, dur_ms)
        else:
            os.system('afplay /System/Library/Sounds/Ping.aiff &')
    except Exception:
        pass

def beep_async(freq=1000, dur_ms=200):
    threading.Thread(target=_beep, args=(freq, dur_ms), daemon=True).start()

# ================================================================
# SETTINGS
# ================================================================
CAMERA_INDEX      = 0
SERIAL_PORT       = '/dev/cu.usbserial-0001'
BAUD_RATE         = 115200

DEBUG_FOCAL       = False
FOCAL_LENGTH      = 600      # Calibrated: FOCAL = pixel_h * distance / real_height

FUSION_ENABLED    = True
OUTLIER_THRESHOLD = 5.0

TRAP_NEAR_L = 0.28
TRAP_NEAR_R = 0.72
TRAP_FAR_L  = 0.38
TRAP_FAR_R  = 0.62
TRAP_FAR_Y  = 0.20

LIDAR_BEAM_OFFSET_PX = 0
LIDAR_ANGLE_TOL_DEG  = 8.0
FPS_LOW_THRESHOLD    = 15.0
BEEP_COOLDOWN_S      = 1.5

# ── Half precision for MPS/CUDA, False for CPU ──────────────
USE_HALF          = True   
YOLO_IMGSZ        = 640    
FRAME_SKIP        = 1      

# ================================================================
# CLASS CONFIGURATIONS (COCO class ID: (Name, Real Height in meters, Safe Dist Multiplier))
# ================================================================
CLASS_INFO = {
    0: ("Person",     1.75, 2.0),
    2: ("Car",        1.50, 1.5),
    3: ("Motorcycle", 1.20, 1.5),
    5: ("Bus",        3.20, 2.0),
    7: ("Truck",      3.50, 2.0),
}
DETECT_CLASSES = list(CLASS_INFO.keys())

# ================================================================
# MEDIAN FILTER FOR LIDAR
# ================================================================
class MedianFilter:
    def __init__(self, window_size=5):
        self.buffer = deque(maxlen=window_size)
        self.last_median = 0.0

    def update(self, v):
        self.buffer.append(v)
        s = sorted(self.buffer)
        self.last_median = s[len(s) // 2]
        return self.last_median

    def get_median(self):
        return self.last_median

# ================================================================
# 2D KALMAN FILTER FOR DISTANCE & VELOCITY TRACKING
# ================================================================
class KalmanFilter2D:
    def __init__(self, dt=0.033):
        self.dt = dt
        sigma_a = 1.0
        self.x = np.array([0.0, 0.0]) # [distance, velocity]
        self.P = np.diag([1.0, 1.0])
        self.F = np.array([[1.0, dt], [0.0, 1.0]])
        self.H = np.array([[1.0, 0.0]])
        self.Q = sigma_a**2 * np.array([
            [dt**4/4, dt**3/2],
            [dt**3/2, dt**2  ]
        ])
        self.R = np.array([[0.20]])
        self.initialized = False

    def update(self, z, dt=None):
        if dt is not None and dt > 0 and abs(dt - self.dt) > 1e-4:
            self.dt = dt
            self.F[0, 1] = dt
            sigma_a = 1.0
            self.Q = sigma_a**2 * np.array([
                [dt**4/4, dt**3/2],
                [dt**3/2, dt**2  ]
            ])
        if not self.initialized:
            self.x[0] = z
            self.initialized = True
            return float(self.x[0]), 0.0
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q
        S      = self.H @ P_pred @ self.H.T + self.R
        K      = (P_pred @ self.H.T) / S[0, 0]
        self.x = x_pred + K.flatten() * float(z - (self.H @ x_pred)[0])
        self.P = (np.eye(2) - np.outer(K.flatten(), self.H)) @ P_pred
        return float(self.x[0]), float(self.x[1])

# ================================================================
# REGION OF INTEREST (ROI) TRAPEZOID FUNCTIONS
# ================================================================
def build_trapez(w, h):
    return np.array([
        [int(w*TRAP_NEAR_L), h],
        [int(w*TRAP_NEAR_R), h],
        [int(w*TRAP_FAR_R),  int(h*TRAP_FAR_Y)],
        [int(w*TRAP_FAR_L),  int(h*TRAP_FAR_Y)],
    ], dtype=np.int32)

def in_trapez(trapez, cx, cy):
    return cv2.pointPolygonTest(trapez, (float(cx), float(cy)), False) >= 0

def camera_distance(pixel_height, class_id):
    if pixel_height <= 0:
        return 0.0
    real_h   = CLASS_INFO.get(class_id, ("", 1.5, 1.5))[1]
    raw_dist = (FOCAL_LENGTH * real_h) / pixel_height
    return max(0.0, raw_dist)

# ================================================================
# SENSOR FUSION
# ================================================================
def lidar_beam_in_bbox(bbox_x1, bbox_x2, frame_w, cam_dist):
    beam_x    = frame_w // 2 + LIDAR_BEAM_OFFSET_PX
    tol_px    = max(30, int(math.tan(math.radians(LIDAR_ANGLE_TOL_DEG)) * FOCAL_LENGTH))
    dist_bonus= int(cam_dist * 2.0)
    return (bbox_x1 - tol_px - dist_bonus <= beam_x <= bbox_x2 + tol_px + dist_bonus), beam_x

def fuse_distance(cam_dist, lidar_dist, strength, lidar_valid,
                  fusion_on, bbox_x1=0, bbox_x2=0, frame_w=640):
    if not fusion_on or not lidar_valid or lidar_dist <= 0.3:
        return cam_dist, 0.0, 1.0, "CAM_ONLY"
    angle_ok, _ = lidar_beam_in_bbox(bbox_x1, bbox_x2, frame_w, cam_dist)
    dist_ok = abs(cam_dist - lidar_dist) <= OUTLIER_THRESHOLD + cam_dist * 0.15
    if angle_ok and dist_ok:
        w_lidar = max(0.3, min(0.9, strength / 100.0))
        debug   = f"FUSED wL:{w_lidar:.2f}"
    elif dist_ok:
        w_lidar = max(0.15, min(0.45, (strength / 100.0) * 0.5))
        debug   = f"PART wL:{w_lidar:.2f}"
    else:
        return cam_dist, 0.0, 1.0, "MISMATCH"
    w_cam = 1.0 - w_lidar
    return lidar_dist*w_lidar + cam_dist*w_cam, w_lidar, w_cam, debug

# ================================================================
# TTC / SAFE DISTANCE / WARNINGS
# ================================================================
def time_to_collision(dist, approach_vel):
    if approach_vel >= -0.1:
        return float('inf')
    return dist / abs(approach_vel)

def safe_distance(speed_kmph, multiplier):
    v = speed_kmph / 3.6
    return max(3.0, (v*0.8 + v**2/8.0) * multiplier)

def warning_level(dist, s_dist, ttc, fps=30.0):
    ttc_critical = 2.5 if fps < FPS_LOW_THRESHOLD else 2.0
    ttc_warning  = 5.0 if fps < FPS_LOW_THRESHOLD else 4.0
    if ttc < ttc_critical or dist < s_dist * 0.5:
        ttc_str = f"TTC:{ttc:.1f}s" if ttc < float('inf') else "BRAKE!"
        return (0, 0, 255), f"DANGER! {ttc_str}", "DANGER"
    elif ttc < ttc_warning or dist < s_dist:
        ttc_str = f"TTC:{ttc:.1f}s" if ttc < float('inf') else ""
        return (0, 165, 255), f"WARNING {ttc_str}", "WARNING"
    return (0, 255, 0), "SAFE", "SAFE"

# ================================================================
# SERIAL PORT PARSER
# ================================================================
def parse_serial_data(data_str):
    if data_str.startswith('#') or not data_str.strip():
        return None
    try:
        parts = data_str.strip().split(',')
        if len(parts) == 5:
            return {"speed": float(parts[0]), "satellites": int(parts[1]),
                    "sensor_dist": float(parts[2])/100.0,
                    "strength": int(parts[3]), "lidar_valid": int(parts[4])}
        elif len(parts) == 4:
            return {"speed": float(parts[0]), "satellites": int(parts[1]),
                    "sensor_dist": float(parts[2])/100.0,
                    "strength": int(parts[3]), "lidar_valid": 1}
    except Exception:
        pass
    return None

# ================================================================
# CSV LOGGER
# ================================================================
class CSVLogger:
    def __init__(self, log_dir="logs"):
        os.makedirs(log_dir, exist_ok=True)
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(log_dir, f"session_{ts}.csv")
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["timestamp","frame_no","track_id","class","conf",
                         "cam_dist_m","lidar_dist_m","fused_dist_m",
                         "approach_vel_ms","ttc_s","warning","fusion_mode",
                         "fps","speed_kmph","satellites","lidar_valid","strength"])
        self.path = path
        print(f"[LOG] CSV: {path}")

    def write(self, frame_no, track_id, class_name, conf,
              cam_dist, lidar_dist, fused_dist, approach_vel,
              ttc, warning, fusion_mode, fps,
              speed_kmph, satellites, lidar_valid, strength):
        ttc_str = f"{ttc:.3f}" if ttc < 999 else "inf"
        self.w.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                          frame_no, track_id, class_name, f"{conf:.3f}",
                          f"{cam_dist:.3f}", f"{lidar_dist:.3f}", f"{fused_dist:.3f}",
                          f"{approach_vel:.3f}", ttc_str, warning, fusion_mode,
                          f"{fps:.1f}", f"{speed_kmph:.1f}",
                          satellites, lidar_valid, strength])

    def flush(self): self.f.flush()
    def close(self):
        self.f.close()
        print(f"[LOG] Saved: {self.path}")

# ================================================================
# VIDEO RECORDER
# ================================================================
class VideoRecorder:
    def __init__(self, rec_dir="recordings"):
        os.makedirs(rec_dir, exist_ok=True)
        self.rec_dir = rec_dir
        self.writer  = None
        self.active  = False
        self.path    = ""

    def start(self, frame_w, frame_h, fps=20.0):
        if self.active: return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path   = os.path.join(self.rec_dir, f"session_{ts}.mp4")
        self.writer = cv2.VideoWriter(self.path,
                        cv2.VideoWriter_fourcc(*"mp4v"), fps, (frame_w, frame_h))
        self.active = True
        print(f"[REC] Recording started: {self.path}")

    def write(self, frame):
        if self.active and self.writer: self.writer.write(frame)

    def stop(self):
        if not self.active: return
        self.writer.release(); self.writer = None; self.active = False
        print(f"[REC] Recording stopped: {self.path}")

    def toggle(self, w, h):
        self.stop() if self.active else self.start(w, h)

# ================================================================
# YOLO MULTITHREADED INFERENCE THREAD
# ================================================================
class YOLOThread:
    """
    Runs YOLO tracking on a separate background thread.
    This prevents blocking the main capture/rendering loop, ensuring high FPS
    even when running heavier object detection tasks.
    """
    def __init__(self, model, device):
        self.model    = model
        self.device   = device
        self.use_half = USE_HALF and device in ('mps', 'cuda')
        self.frame   = None
        self.result  = None
        self.lock    = threading.Lock()
        self.running = True
        self.thread  = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            with self.lock:
                frame = self.frame
            if frame is None:
                time.sleep(0.001)
                continue
            result = self.model.track(
                frame, classes=DETECT_CLASSES, verbose=False,
                device=self.device, persist=True,
                tracker="bytetrack.yaml", imgsz=YOLO_IMGSZ,
                half=self.use_half, conf=0.45
            )
            with self.lock:
                self.result = result
                self.frame  = None   # processed

    def submit(self, frame):
        with self.lock:
            self.frame = frame.copy()

    def get_result(self):
        with self.lock:
            return self.result

    def stop(self):
        self.running = False

# ================================================================
# MAIN LOOP
# ================================================================
def main():
    global FUSION_ENABLED

    if torch.backends.mps.is_available():
        device = 'mps'
    elif torch.cuda.is_available():
        device = 'cuda'
    else:
        device = 'cpu'
    print(f"Device: {device}")
    if device == 'mps':
        print("[INFO] Apple Silicon MPS — half=True, fast mode active")

    print("Loading YOLO model...")
    model = YOLO('yolov8n.pt')

    print("Warming up model...")
    use_half = USE_HALF and device in ('mps', 'cuda')
    dummy = np.zeros((320, 320, 3), dtype=np.uint8)
    model.track(dummy, verbose=False, device=device, persist=False,
                half=use_half, imgsz=YOLO_IMGSZ)
    print(f"Model ready. [half={use_half} imgsz={YOLO_IMGSZ}]")

    print("Starting camera capture...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"ERROR: Camera at index {CAMERA_INDEX} could not be opened!")
        return

    cam_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    cam_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[CAMERA] {cam_w}x{cam_h} @ {cap.get(cv2.CAP_PROP_FPS):.0f}fps")

    if cam_w != 640 or cam_h != 480:
        print(f"[WARNING] Expected 640x480 resolution, got {cam_w}x{cam_h}")

    for _ in range(5):
        cap.read()

    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Serial port connected: {SERIAL_PORT}")
    except Exception as e:
        print(f"WARNING: Serial port not available ({e})")

    lidar_median   = MedianFilter(window_size=5)
    kalman_filters = {}
    beep_times     = {}

    sensor_data = {"speed": 0.0, "satellites": 0,
                   "sensor_dist": 0.0, "strength": 0, "lidar_valid": 0}

    logger   = CSVLogger()
    recorder = VideoRecorder()

    # Start YOLO Thread
    yolo_thread = YOLOThread(model, device)

    prev_time         = 0.0
    frame_no          = 0
    trapez            = None
    fps               = 30.0
    last_danger_level = "SAFE"
    cached_boxes      = None   # Latest YOLO results from the thread

    print("\nControls: [F] Toggle Fusion  [V] Toggle Video Rec  [Q] Quit\n")

    while cap.isOpened():
        curr_time = time.time()
        dt  = curr_time - prev_time if prev_time > 0 else 0.033
        dt  = max(0.001, min(dt, 0.5))
        fps = 0.9*fps + 0.1*(1.0/dt)

        ret, frame = cap.read()
        if not ret:
            print("Cannot read frame from camera!")
            break

        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame_no += 1
        h, w = frame.shape[:2]

        if trapez is None:
            trapez = build_trapez(w, h)

        # ── 1. SERIAL COMMS ───────────────────────────────────────
        new_data = False
        if ser and ser.is_open:
            while ser.in_waiting > 0:
                try:
                    line   = ser.readline().decode('utf-8', errors='ignore').strip()
                    parsed = parse_serial_data(line)
                    if parsed:
                        sensor_data = parsed; new_data = True
                except Exception:
                    break

        my_speed      = sensor_data["speed"]
        my_satellites = sensor_data["satellites"]
        raw_dist      = sensor_data["sensor_dist"]
        strength      = sensor_data["strength"]
        lidar_valid   = sensor_data["lidar_valid"]

        # ── 2. MEDIAN FILTER ──────────────────────────────────────
        clean_dist = lidar_median.update(raw_dist) if (new_data and lidar_valid) \
                     else lidar_median.get_median()

        # ── 3. KEYBOARD CONTROLS ──────────────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            FUSION_ENABLED = not FUSION_ENABLED
            print(f"[FUSION] {'ENABLED' if FUSION_ENABLED else 'DISABLED'}")
        elif key == ord('v'):
            recorder.toggle(w, h)

        # ── 4. YOLO DELEGATION ────────────────────────────────────
        if frame_no % FRAME_SKIP == 0:
            yolo_thread.submit(frame)

        result = yolo_thread.get_result()
        if result is not None:
            cached_boxes = result[0].boxes

        boxes = cached_boxes

        cv2.polylines(frame, [trapez], True, (255, 255, 255), 1, cv2.LINE_AA)
        frame_max_danger = "SAFE"

        # ── 5. DETECTED OBJECTS LOOP ──────────────────────────────
        if boxes is not None:
            for box in boxes:
                x1 = int(box.xyxy[0][0]); y1 = int(box.xyxy[0][1])
                x2 = int(box.xyxy[0][2]); y2 = int(box.xyxy[0][3])
                class_id = int(box.cls[0])
                conf     = float(box.conf[0])
                track_id = int(box.id[0]) if box.id is not None else -1
                center_x = (x1+x2)//2;  center_y = (y1+y2)//2

                if conf < 0.45: continue
                if not in_trapez(trapez, center_x, center_y): continue

                class_name = CLASS_INFO.get(class_id, ("?",))[0]
                safe_mult  = CLASS_INFO.get(class_id, ("", 1.5, 1.5))[2]
                pixel_h    = y2 - y1
                cam_dist   = camera_distance(pixel_h, class_id)

                if DEBUG_FOCAL and class_id == 0:
                    print(f"[FOCAL] pixel_h={pixel_h} → suggested focal: {pixel_h*3.0/1.75:.0f}")

                raw_meas, w_lidar, w_cam, fusion_dbg = fuse_distance(
                    cam_dist, clean_dist, strength, lidar_valid, FUSION_ENABLED,
                    bbox_x1=x1, bbox_x2=x2, frame_w=w)

                if track_id not in kalman_filters:
                    kalman_filters[track_id] = KalmanFilter2D(dt=dt)
                filtered_dist, approach_vel = kalman_filters[track_id].update(raw_meas, dt=dt)

                ttc    = time_to_collision(filtered_dist, approach_vel)
                s_dist = safe_distance(my_speed, safe_mult)
                color, label, level = warning_level(filtered_dist, s_dist, ttc, fps)

                danger_order = {"SAFE": 0, "WARNING": 1, "DANGER": 2}
                if danger_order.get(level, 0) > danger_order.get(frame_max_danger, 0):
                    frame_max_danger = level

                logger.write(frame_no, track_id, class_name, conf,
                             cam_dist, clean_dist, filtered_dist, approach_vel,
                             ttc, level, fusion_dbg, fps,
                             my_speed, my_satellites, lidar_valid, strength)

                cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                cv2.putText(frame, f"{class_name} #{track_id}",
                    (x1, y1-42), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
                ttc_str = f"  TTC:{ttc:.1f}s" if ttc < float('inf') else ""
                cv2.putText(frame, f"{filtered_dist:.1f}m{ttc_str}",
                    (x1, y1-24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
                cv2.putText(frame, label,
                    (x1, y1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
                if FUSION_ENABLED:
                    outlier = "!" if abs(cam_dist-clean_dist) > OUTLIER_THRESHOLD else ""
                    cv2.putText(frame,
                        f"cam:{cam_dist:.1f} lidar:{clean_dist:.1f}{outlier}"
                        f" vel:{approach_vel:.1f}m/s [{fusion_dbg}]",
                        (x1, y2+14), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (120,120,120), 1, cv2.LINE_AA)
                cv2.line(frame, (w//2, h), (center_x, y2), color, 1, cv2.LINE_AA)

        # ── 5b. AUDIO WARNINGS ────────────────────────────────────
        now = time.time()
        if frame_max_danger == "DANGER":
            if now - beep_times.get("danger", 0) > BEEP_COOLDOWN_S:
                beep_async(1200, 180); beep_times["danger"] = now
        elif frame_max_danger == "WARNING" and last_danger_level == "SAFE":
            beep_async(700, 300)
        last_danger_level = frame_max_danger

        # ── CLEAN FILTER CACHE ────────────────────────────────────
        if boxes is not None and boxes.id is not None:
            active_ids = {int(i) for i in boxes.id}
            kalman_filters = {k: v for k,v in kalman_filters.items() if k in active_ids}

        # ── 6. FPS RENDER ─────────────────────────────────────────
        prev_time = curr_time
        fps_col  = (0,255,0) if fps >= FPS_LOW_THRESHOLD else (0,165,255)
        fps_warn = "" if fps >= FPS_LOW_THRESHOLD else " LOW"
        cv2.putText(frame, f"FPS:{int(fps)}{fps_warn}",
            (w-160, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, fps_col, 2, cv2.LINE_AA)

        # ── 7. TELEMETRY PANEL ────────────────────────────────────
        fusion_str = "FUSION:ON" if FUSION_ENABLED else "FUSION:OFF"
        fusion_col = (0,255,200) if FUSION_ENABLED else (100,100,100)
        str_pct = int(min(100, max(0,
            math.log10(strength/100.0)/math.log10(50.0)*100))) if strength >= 100 else 0

        PX, PY = 6, 6
        lines = [
            (f"Speed {my_speed:.0f}km/h  Sat:{my_satellites}", (200,200,200)),
            (f"LiDAR {clean_dist:.2f}m  Sig:{str_pct}%",       (64,196,255)),
            (f"{fusion_str}  {'REC' if recorder.active else ''}", fusion_col),
            (f"{cam_w}x{cam_h}  imgsz:{YOLO_IMGSZ}  skip:{FRAME_SKIP}", (140,140,140)),
        ]
        line_h = 18
        box_w, box_h = 240, len(lines)*line_h + 8
        overlay = frame.copy()
        cv2.rectangle(overlay, (PX,PY), (PX+box_w,PY+box_h), (20,20,20), -1)
        cv2.addWeighted(overlay, 0.50, frame, 0.50, 0, frame)
        for i, (txt, col) in enumerate(lines):
            cv2.putText(frame, txt, (PX+5, PY+line_h*(i+1)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)
        if fps < FPS_LOW_THRESHOLD:
            cv2.putText(frame, "! Low FPS",
                (PX+5, PY+box_h+12), cv2.FONT_HERSHEY_SIMPLEX,
                0.38, (0,165,255), 1, cv2.LINE_AA)

        # ── 8. WRITE VIDEO FRAME ──────────────────────────────────
        recorder.write(frame)
        if frame_no % 30 == 0:
            logger.flush()

        cv2.imshow("ADAS", frame)

    # ── CLEANUP ───────────────────────────────────────────────
    yolo_thread.stop()
    recorder.stop()
    logger.close()
    if ser and ser.is_open: ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print(f"[FINISHED] {frame_no} frames processed.")


if __name__ == "__main__":
    main()
