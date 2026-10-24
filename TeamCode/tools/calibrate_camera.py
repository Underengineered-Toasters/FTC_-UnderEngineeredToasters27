#!/usr/bin/env python3
"""Camera calibration helper for the FTC webcam calibration file.

  calibrate_camera.py board board.png          write a checkerboard to show on a screen
  calibrate_camera.py solve FRAMES_DIR         solve intrinsics from captured frames
  calibrate_camera.py crop BIG.png SMALL.png   locate a small mode inside a big mode

Requires python-opencv. The solve step prints a <Camera> element ready to paste into
TeamCode/src/main/res/xml/teamwebcamcalibrations.xml. Intrinsics do not depend on the physical
square size, so any checkerboard size works as long as the inner-corner counts below match.

The crop step answers whether a low resolution is a centre crop of a higher one. If it is, the
principal point of the low mode equals the high mode's minus the reported offset, which is a
cheap way to check a principal point that landed outside the image.
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
        print("principal point is outside the image. That usually means the board sat on one")
        print("side in every frame, which leaves the offset poorly determined. Tighten it:")
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


def locate_crop(big_path, small_path):
    big = cv2.imread(big_path, cv2.IMREAD_GRAYSCALE)
    small = cv2.imread(small_path, cv2.IMREAD_GRAYSCALE)
    if big is None or small is None:
        sys.exit(f"cannot read {big_path if big is None else small_path}")
    orb = cv2.ORB_create(nfeatures=4000)
    kb, db = orb.detectAndCompute(big, None)
    ks, ds = orb.detectAndCompute(small, None)
    if db is None or ds is None or len(kb) < 10 or len(ks) < 10:
        print("too few features to compare; use a scene with texture")
        return
    matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(ds, db, k=2)
    good = [m for m, n in (pair for pair in matches if len(pair) == 2) if m.distance < 0.75 * n.distance]
    if len(good) < 8:
        print(f"only {len(good)} consistent matches; the two images probably show different scenes")
        return
    src = np.float32([ks[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kb[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    matrix, inliers = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=3)
    if matrix is None:
        print("could not fit the two views together")
        return
    scale = (matrix[0, 0] ** 2 + matrix[1, 0] ** 2) ** 0.5
    ox, oy = matrix[0, 2], matrix[1, 2]
    inlier_count = int(inliers.sum()) if inliers is not None else 0
    print(f"{small.shape[1]}x{small.shape[0]} sits in {big.shape[1]}x{big.shape[0]} "
          f"at ({ox:.1f}, {oy:.1f}) with scale {scale:.4f}, {inlier_count} of {len(good)} matches agreed")
    # The small image's origin maps to (ox, oy) in big pixels; a centred crop starts at
    # (big - small*scale) / 2.
    cx_expected = (big.shape[1] - small.shape[1] * scale) / 2
    cy_expected = (big.shape[0] - small.shape[0] * scale) / 2
    print(f"a centred crop would start at ({cx_expected:.1f}, {cy_expected:.1f})")
    if abs(ox - cx_expected) < 4 and abs(oy - cy_expected) < 4:
        print("the low resolution is a centred crop, so its principal point is the high one scaled")
    else:
        print("the low resolution is off centre: shift its principal point by "
              f"(-{ox:.1f}, -{oy:.1f}) relative to the high mode's scaled value")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "board":
        make_board(sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == "solve":
        solve(sys.argv[2])
    elif len(sys.argv) == 4 and sys.argv[1] == "crop":
        locate_crop(sys.argv[2], sys.argv[3])
    else:
        sys.exit(__doc__)
