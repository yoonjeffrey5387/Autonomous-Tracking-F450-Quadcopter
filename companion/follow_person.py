"""Condensed person-following loop for the Jetson companion computer.

This is a short, readable version of the on-board tracker. The flight version adds
OSNet re-identification, TensorRT FP16 engines, and the operator web UI.

Pipeline: camera -> YOLO person detection -> ByteTrack IDs -> distance + heading error
-> forward-velocity and yaw-rate setpoints over MAVLink2 (ArduCopter GUIDED mode).

Run:  python3 follow_person.py --target-id 1 --distance 3.0 --port /dev/ttyTHS0
"""
import argparse
import time

import cv2
from pymavlink import mavutil
from ultralytics import YOLO

PERSON_HEIGHT_M = 1.70   # assumed height of the person being followed
FOCAL_PX = 330.0         # vertical focal length at 640x360, from camera calibration
KD = 0.4                 # forward gain [1/s]
KYAW = 0.8               # yaw gain [rad/s per unit of normalized horizontal error]
VX_MAX = 1.0             # m/s, deliberately conservative
YAW_MAX = 0.6            # rad/s
MIN_ALT_M = 1.5
HEARTBEAT_TIMEOUT_S = 2.0
GUIDED = 4               # ArduCopter custom_mode number for GUIDED

# Use vx/vy/vz and yaw_rate; ignore position, acceleration and absolute yaw.
VEL_YAWRATE_MASK = 0b0000_0101_1100_0111


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Vehicle:
    """Tracks the flight controller's state and sends body-frame velocity setpoints."""

    def __init__(self, port, baud):
        self.mav = mavutil.mavlink_connection(port, baud=baud)
        self.mav.wait_heartbeat()
        self.last_heartbeat = time.time()
        self.mode = None
        self.armed = False
        self.alt_m = 0.0

    def poll(self):
        while (msg := self.mav.recv_match(blocking=False)) is not None:
            kind = msg.get_type()
            if kind == "HEARTBEAT" and msg.get_srcSystem() == self.mav.target_system:
                self.last_heartbeat = time.time()
                self.mode = msg.custom_mode
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "GLOBAL_POSITION_INT":
                self.alt_m = msg.relative_alt / 1000.0

    def safe_to_command(self):
        """The Jetson only commands the drone when every safety condition holds."""
        return (
            time.time() - self.last_heartbeat < HEARTBEAT_TIMEOUT_S
            and self.mode == GUIDED
            and self.armed
            and self.alt_m >= MIN_ALT_M
        )

    def send(self, vx, yaw_rate):
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, VEL_YAWRATE_MASK,
            0, 0, 0,          # position (ignored)
            vx, 0, 0,         # velocity: forward only, vz = 0 holds altitude
            0, 0, 0,          # acceleration (ignored)
            0, yaw_rate,      # yaw (ignored), yaw rate
        )


def find_target(result, target_id):
    if result.boxes.id is None:
        return None
    for box, tid in zip(result.boxes.xyxy.tolist(), result.boxes.id.int().tolist()):
        if tid == target_id:
            return box
    return None


def control(box, width, height, follow_dist):
    x1, y1, x2, y2 = box
    box_h = max(y2 - y1, 1.0)
    distance = PERSON_HEIGHT_M * FOCAL_PX / box_h          # pinhole camera model
    ex = ((x1 + x2) / 2 - width / 2) / (width / 2)        # -1 (left) .. +1 (right)
    fully_visible = y1 > 2 and y2 < height - 2            # don't advance on a cut-off person
    vx = clamp(KD * (distance - follow_dist), 0.0, VX_MAX) if fully_visible else 0.0
    yaw_rate = clamp(KYAW * ex, -YAW_MAX, YAW_MAX)
    return vx, yaw_rate, distance


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-id", type=int, default=1)
    ap.add_argument("--distance", type=float, default=3.0, help="follow distance in meters (3-15)")
    ap.add_argument("--port", default="/dev/ttyTHS0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()

    vehicle = Vehicle(args.port, args.baud)
    model = YOLO("yolo26n.pt")
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        vehicle.poll()
        result = model.track(frame, persist=True, classes=[0], tracker="bytetrack.yaml", verbose=False)[0]
        box = find_target(result, args.target_id)

        vx, yaw_rate = 0.0, 0.0   # lost target or unsafe state: hold position
        if box is not None:
            vx, yaw_rate, distance = control(box, frame.shape[1], frame.shape[0], args.distance)
            print(f"id={args.target_id} d={distance:4.1f} m vx={vx:4.2f} yaw={yaw_rate:+.2f}")
        if vehicle.safe_to_command():
            vehicle.send(vx, yaw_rate)


if __name__ == "__main__":
    main()
