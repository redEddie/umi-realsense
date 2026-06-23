#!/usr/bin/env python3
"""
Convert RealSense .db3 recordings into an OpenICC (mynteye-style) dataset.

OpenICC's run_mynteye_calibration.py expects:
    <dataset>/cam/cam0/<ts_ns>.png          # camera-intrinsics clip (board, slow)
    <dataset>/cam_imu/cam0/<ts_ns>.png      # cam-IMU clip (board, excite 6 axes)
    <dataset>/cam_imu/imu0.csv              # EuRoC: ts_ns,gx,gy,gz,ax,ay,az  (NO header)

We use the LEFT IR image (cam0) because ORB-SLAM3 tracks on the IR stereo. IMU is
read losslessly at sensor level (~200 Hz). Record both clips with the IR emitter
OFF (configs/cameras/d435i_slam.yaml) so the charuco board is clean.

    conda activate umi
    python calibration/db3_to_openicc.py \
        --cam      data/recordings/<cam_clip>.db3 \
        --cam_imu  data/recordings/<cam_imu_clip>.db3 \
        --out      data/calib/d435i_openicc
"""
from __future__ import annotations
import argparse, csv, os, sys
import numpy as np
import cv2
import pyrealsense2 as rs

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def extract(db3: str, img_dir: str, imu_csv: str | None):
    """Extract LEFT IR frames (-> img_dir/<ts_ns>.png) and, if imu_csv given,
    a header-less EuRoC imu0.csv. Sensor-level read = full-rate IMU, no drops."""
    os.makedirs(img_dir, exist_ok=True)
    pb = rs.context().load_device(db3).as_playback()
    pb.set_real_time(False)
    q = rs.frame_queue(8192, keep_frames=True)
    opened = []
    for s in pb.query_sensors():
        profs = s.get_stream_profiles()
        if profs:
            s.open(profs); s.start(q); opened.append(s)

    accel, gyro = [], []
    n_img = 0
    while True:
        try:
            f = q.wait_for_frame(1500)
        except RuntimeError:
            break
        p = f.get_profile(); st = p.stream_type()
        t_ns = int(round(f.get_timestamp() * 1e6))  # ms -> ns
        if st == rs.stream.infrared and p.stream_index() == 1:
            img = np.asanyarray(f.as_video_frame().get_data())
            cv2.imwrite(os.path.join(img_dir, f"{t_ns}.png"), img)
            n_img += 1
        elif imu_csv is not None and st == rs.stream.accel:
            d = f.as_motion_frame().get_motion_data(); accel.append((t_ns, d.x, d.y, d.z))
        elif imu_csv is not None and st == rs.stream.gyro:
            d = f.as_motion_frame().get_motion_data(); gyro.append((t_ns, d.x, d.y, d.z))
    for s in opened:
        try: s.stop(); s.close()
        except RuntimeError: pass

    n_imu = 0
    if imu_csv is not None:
        n_imu = write_imu_csv(imu_csv, accel, gyro)
    return n_img, n_imu


def write_imu_csv(path, accel, gyro) -> int:
    """Header-less EuRoC rows: ts_ns,gx,gy,gz,ax,ay,az (accel interp -> gyro times)."""
    if not accel or not gyro:
        open(path, "w").close(); return 0
    accel.sort(); gyro.sort()
    at = np.array([a[0] for a in accel], float)
    av = np.array([[a[1], a[2], a[3]] for a in accel], float)
    rows = []
    for t, gx, gy, gz in gyro:
        if t < at[0] or t > at[-1]:
            continue
        a = [np.interp(t, at, av[:, k]) for k in range(3)]
        rows.append((t, gx, gy, gz, a[0], a[1], a[2]))
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        for r in rows:
            w.writerow([r[0], f"{r[1]:.9f}", f"{r[2]:.9f}", f"{r[3]:.9f}",
                        f"{r[4]:.9f}", f"{r[5]:.9f}", f"{r[6]:.9f}"])
    return len(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description="RealSense .db3 -> OpenICC dataset")
    ap.add_argument("--cam", required=True, help="camera-intrinsics clip .db3")
    ap.add_argument("--cam_imu", required=True, help="cam-IMU clip .db3")
    ap.add_argument("--out", required=True, help="output dataset dir")
    args = ap.parse_args()

    out = os.path.abspath(args.out)
    print(f"[cam]     {args.cam}")
    ni, _ = extract(args.cam, os.path.join(out, "cam", "cam0"), None)
    print(f"          -> cam/cam0: {ni} frames")
    print(f"[cam_imu] {args.cam_imu}")
    ni, nimu = extract(args.cam_imu, os.path.join(out, "cam_imu", "cam0"),
                       os.path.join(out, "cam_imu", "imu0.csv"))
    print(f"          -> cam_imu/cam0: {ni} frames | imu0.csv: {nimu} rows")
    print(f"\nDataset: {out}")
    print("Next: calibration/run_openicc.sh  (after OpenICC is built)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
