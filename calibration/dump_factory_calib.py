#!/usr/bin/env python3
"""
Stage A1 - dump factory calibration from the connected RealSense.

Reads intrinsics (IR1/IR2/color/depth), stereo + cam->IMU extrinsics, IMU
motion intrinsics and depth scale straight from librealsense, and writes:
  calibration/factory_calib_<serial>.json   (full raw record)

It also prints the ORB-SLAM3-relevant values (fx,fy,cx,cy, stereo baseline,
and a T_b_c1 seed) so the slam/configs yaml can be updated with this unit's
real numbers instead of the generic defaults.

Intrinsics/baseline are reliable factory values. The cam->IMU extrinsic is only
a *seed* - refine it in Stage B (OpenICC/Kalibr).

    conda activate umi
    python calibration/dump_factory_calib.py
"""
import json, os, sys
import numpy as np
import pyrealsense2 as rs

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rs_extrinsics_to_T(ext):
    """rs2 extrinsics (column-major 3x3 R + t) -> 4x4 homogeneous (row-major)."""
    R = np.array(ext.rotation, dtype=float).reshape(3, 3).T  # column-major -> row-major
    t = np.array(ext.translation, dtype=float)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def intr_dict(vsp):
    i = vsp.get_intrinsics()
    return {"width": i.width, "height": i.height, "fps": vsp.fps(),
            "model": str(i.model), "fx": i.fx, "fy": i.fy,
            "ppx": i.ppx, "ppy": i.ppy, "coeffs": list(i.coeffs)}


def main():
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        print("No RealSense device connected."); return 1

    cfg = rs.config()
    cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.accel)
    cfg.enable_stream(rs.stream.gyro)

    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    try:
        dev = prof.get_device()
        sp = {
            "ir1": prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile(),
            "ir2": prof.get_stream(rs.stream.infrared, 2).as_video_stream_profile(),
            "color": prof.get_stream(rs.stream.color).as_video_stream_profile(),
            "depth": prof.get_stream(rs.stream.depth).as_video_stream_profile(),
            "accel": prof.get_stream(rs.stream.accel).as_motion_stream_profile(),
            "gyro": prof.get_stream(rs.stream.gyro).as_motion_stream_profile(),
        }
        intrinsics = {k: intr_dict(sp[k]) for k in ("ir1", "ir2", "color", "depth")}

        T_ir1_ir2 = rs_extrinsics_to_T(sp["ir1"].get_extrinsics_to(sp["ir2"]))
        T_b_c1 = rs_extrinsics_to_T(sp["ir1"].get_extrinsics_to(sp["gyro"]))  # cam1 -> IMU(body)
        baseline = float(abs(sp["ir1"].get_extrinsics_to(sp["ir2"]).translation[0]))

        imu = {}
        for k in ("accel", "gyro"):
            mi = sp[k].get_motion_intrinsics()
            imu[k] = {"fps": sp[k].fps(), "data": [list(r) for r in mi.data],
                      "noise_variances": list(mi.noise_variances),
                      "bias_variances": list(mi.bias_variances)}
        depth_scale = dev.first_depth_sensor().get_depth_scale()

        serial = dev.get_info(rs.camera_info.serial_number)
        out = {
            "device": {"name": dev.get_info(rs.camera_info.name), "serial": serial,
                       "firmware": dev.get_info(rs.camera_info.firmware_version)},
            "intrinsics": intrinsics,
            "stereo_baseline_m": baseline,
            "T_ir1_to_ir2": T_ir1_ir2.tolist(),
            "T_b_c1_seed": T_b_c1.tolist(),
            "imu": imu, "depth_scale": depth_scale,
        }
        path = os.path.join(REPO, "calibration", f"factory_calib_{serial}.json")
        with open(path, "w") as f:
            json.dump(out, f, indent=2)
    finally:
        pipe.stop()

    i = intrinsics["ir1"]
    print(f"Device {out['device']['name']} SN {serial}\n")
    print("== ORB-SLAM3 stereo params (left IR = camera 1) ==")
    print(f"  Camera1.fx: {i['fx']:.4f}")
    print(f"  Camera1.fy: {i['fy']:.4f}")
    print(f"  Camera1.cx: {i['ppx']:.4f}")
    print(f"  Camera1.cy: {i['ppy']:.4f}")
    print(f"  Stereo.b:   {baseline:.7f}   (m)")
    print(f"  distortion coeffs (should be ~0, rectified): {i['coeffs']}")
    print(f"\n  T_b_c1 seed (IMU<-cam1, refine in Stage B):\n{np.array2string(T_b_c1, precision=5)}")
    print(f"\nWrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
