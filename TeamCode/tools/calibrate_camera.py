#!/usr/bin/env python3
"""Camera calibration helper for the FTC webcam calibration file.

  calibrate_camera.py board board.png          write a checkerboard to show on a screen
  calibrate_camera.py solve FRAMES_DIR         solve intrinsics from captured frames

Requires python-opencv. The solve step prints a <Camera> element ready to paste into
TeamCode/src/main/res/xml/teamwebcamcalibrations.xml. Intrinsics do not depend on the physical
square size, so any checkerboard size works as long as the inner-corner counts below match.
"""
import glob
import os
import sys

import cv2
import numpy as np

COLS, ROWS = 9, 6          # inner corners; the board has (COLS+1) x (ROWS+1) squares
VID, PID = "0x0C45", "0x6366"  # Arducam OV9281


def make_board(path, square_px=120, margin_px=120):
    w = (COLS + 1) * square_px + 2 * margin_px
    h = (ROWS + 1) * square_px + 2 * margin_px
    img = np.full((h, w), 255, np.uint8)
    for r in range(ROWS + 1):
        for c in range(COLS + 1):
            if (r + c) % 2 == 0:
                y, x = margin_px + r * square_px, margin_px + c * square_px
                img[y:y + square_px, x:x + square_px] = 0
    cv2.imwrite(path, img)
    print(f"wrote {path} ({w}x{h}); inner corners {COLS}x{ROWS}")


def solve(frames_dir):
    paths = sorted(p for ext in ("png", "jpg", "jpeg") for p in glob.glob(os.path.join(frames_dir, f"*.{ext}")))
    if not paths:
        sys.exit(f"no images in {frames_dir}")
    objp = np.zeros((ROWS * COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2)
    obj_pts, img_pts, size = [], [], None
    for p in paths:
        gray = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        if size is None:
            size = (gray.shape[1], gray.shape[0])
        elif size != (gray.shape[1], gray.shape[0]):
            sys.exit(f"{p} is {gray.shape[1]}x{gray.shape[0]}, expected {size[0]}x{size[1]}")
        found, corners = cv2.findChessboardCorners(
            gray, (COLS, ROWS), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            print(f"skip {os.path.basename(p)}: board not found")
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj_pts.append(objp)
        img_pts.append(corners)
    if len(obj_pts) < 10:
        sys.exit(f"only {len(obj_pts)} usable frames; need at least 10 (20 or more is better)")
    rms, k, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    d = np.zeros(8)
    d[:5] = dist.ravel()[:5]  # OpenCV order k1 k2 p1 p2 k3, which is the FTC file's order
    print(f"{len(obj_pts)} of {len(paths)} frames used, image {size[0]}x{size[1]}")
    print(f"free solve: reprojection RMS {rms:.3f} px")
    print(f"free solve: fx={k[0, 0]:.3f} fy={k[1, 1]:.3f} cx={k[0, 2]:.3f} cy={k[1, 2]:.3f}")
    inside = 0 <= k[0, 2] < size[0] and 0 <= k[1, 2] < size[1]
    if not inside:
        print("principal point is outside the image. That happens when every frame shows the")
        print("board on one side, and it makes bearing wrong by the offset. Tighten it:")
        flags = cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_ZERO_TANGENT_DIST
        rms2, k2, dist2, _, _ = cv2.calibrateCamera(
            obj_pts, img_pts, size, None, None, flags=flags)
        print(f"constrained: reprojection RMS {rms2:.3f} px")
        print(f"constrained: fx={k2[0, 0]:.3f} fy={k2[1, 1]:.3f} cx={k2[0, 2]:.3f} cy={k2[1, 2]:.3f}")
        if rms2 <= max(rms * 1.5, rms + 0.4):
            print("The constrained fit costs little accuracy, so use it:")
            k, d = k2, np.zeros(8)
            d[:5] = dist2.ravel()[:5]
        else:
            print("The constrained fit is much worse, which means the board really is off to")
            print("one side; recapture with the board near the centre of the view.")
    print(f"horizontal field of view {2 * np.degrees(np.arctan(size[0] / 2 / k[0, 0])):.1f} deg")
    print()
    print(f'<Camera vid="{VID}" pid="{PID}">')
    print("    <Calibration")
    print(f'        size="{size[0]} {size[1]}"')
    print(f'        focalLength="{k[0, 0]:.3f}f, {k[1, 1]:.3f}f"')
    print(f'        principalPoint="{k[0, 2]:.3f}f, {k[1, 2]:.3f}f"')
    print(f'        distortionCoefficients="{", ".join(f"{v:.6f}f" for v in d)}"')
    print("        />")
    print("</Camera>")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "board":
        make_board(sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == "solve":
        solve(sys.argv[2])
    else:
        sys.exit(__doc__)
