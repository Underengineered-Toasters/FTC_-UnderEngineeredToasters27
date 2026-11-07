#!/usr/bin/env python3
"""Solve webcam intrinsics from MonoCalibrationCapture frames.

The FTC OpMode MonoCalibrationCapture saves grayscale frames as
    /sdcard/FIRST/calib/calib_<W>x<H>_<NN>.png
Pull that directory off the Control Hub and run:

    python3 calibrate_camera.py <image_dir> --cols 9 --rows 6 --square-mm 25

The solver prints (and writes) a <Camera> element ready to paste into
TeamCode/src/main/res/xml/teamwebcamcalibrations.xml.

Run `python3 calibrate_camera.py --selftest` to check the pipeline against a
synthetic camera with known intrinsics.
"""

import argparse
import os
import re
import sys

import cv2
import numpy as np

FRAME_RE = re.compile(r"^calib_(\d+)x(\d+)_(\d+)\.png$", re.IGNORECASE)

MIN_VIEWS = 10
GOOD_VIEWS = 15
MAX_RMS_PX = 0.8
MAX_PRINCIPAL_OFFSET = 0.10


def parse_size(text):
    match = re.match(r"^\s*(\d+)\s*[xX ]\s*(\d+)\s*$", text)
    if not match:
        raise argparse.ArgumentTypeError("expected WIDTHxHEIGHT, e.g. 640x480")
    return int(match.group(1)), int(match.group(2))


def discover_frames(image_dir, requested_size):
    """Return (size, [(name, index)]) for the requested or most common resolution."""
    try:
        names = sorted(os.listdir(image_dir))
    except OSError as exc:
        raise SystemExit("cannot read %s: %s" % (image_dir, exc))

    found = []
    for name in names:
        match = FRAME_RE.match(name)
        if match:
            found.append((name, int(match.group(1)), int(match.group(2)), int(match.group(3))))

    if not found:
        raise SystemExit("no calib_<W>x<H>_<NN>.png files in %s" % image_dir)

    if requested_size is not None:
        size = requested_size
    else:
        counts = {}
        for _, width, height, _ in found:
            counts[(width, height)] = counts.get((width, height), 0) + 1
        size = max(counts, key=counts.get)

    selected = [(name, index) for name, width, height, index in found
                if (width, height) == size]
    selected.sort(key=lambda item: item[1])
    if not selected:
        raise SystemExit("no frames at %dx%d in %s" % (size[0], size[1], image_dir))
    return size, selected


def load_frames(image_dir, size, selected):
    frames = []
    for name, _ in selected:
        path = os.path.join(image_dir, name)
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            print("skip %s: cannot decode" % name)
            continue
        if (image.shape[1], image.shape[0]) != size:
            print("skip %s: %dx%d, expected %dx%d"
                  % (name, image.shape[1], image.shape[0], size[0], size[1]))
            continue
        frames.append({"name": name, "image": image})
    return frames


def find_corners(gray, cols, rows):
    """Return (corners float32 (N,2), method) or (None, None)."""
    pattern = (cols, rows)
    try:
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=0)
    except cv2.error:
        ok, corners = False, None
    if ok and corners is not None and len(corners) == cols * rows:
        return corners.reshape(-1, 2).astype(np.float32), "SB"

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok or corners is None or len(corners) != cols * rows:
        return None, None

    points = corners.reshape(-1, 1, 2).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    points = cv2.cornerSubPix(gray, points, (11, 11), (-1, -1), criteria)
    return points.reshape(-1, 2), "classic"


def calibrate(frames, cols, rows, square_mm, size, debug_dir=None):
    """Detect corners in every frame and solve intrinsics. Returns a result dict."""
    pattern = (cols, rows)
    object_template = np.zeros((cols * rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_mm)

    object_points = []
    image_points = []
    used_names = []
    used_methods = []
    skipped = []

    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)

    for frame in frames:
        gray = frame["image"]
        corners, method = find_corners(gray, cols, rows)
        if corners is None:
            skipped.append(frame["name"])
            continue
        object_points.append(object_template.copy())
        image_points.append(corners.reshape(-1, 1, 2))
        used_names.append(frame["name"])
        used_methods.append(method)

        if debug_dir:
            color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(color, pattern, corners.reshape(-1, 1, 2), True)
            cv2.imwrite(os.path.join(debug_dir, frame["name"]), color)

    total = len(frames)
    used = len(object_points)
    print("frames with a full %dx%d board: %d of %d" % (cols, rows, used, total))
    if skipped:
        print("skipped: " + ", ".join(skipped))
    if used < MIN_VIEWS:
        raise SystemExit("need at least %d usable frames, found %d" % (MIN_VIEWS, used))

    rms, camera_matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, size, None, None,
        flags=cv2.CALIB_RATIONAL_MODEL)

    per_view = []
    for index, (objp, imgp, rvec, tvec) in enumerate(
            zip(object_points, image_points, rvecs, tvecs)):
        projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist)
        error = cv2.norm(imgp, projected, cv2.NORM_L2) / max(1, len(projected))
        per_view.append((used_names[index], used_methods[index], float(error)))

    return {
        "rms": float(rms),
        "camera_matrix": camera_matrix,
        "dist": dist.reshape(-1),
        "per_view": per_view,
        "used": used,
        "total": total,
        "size": size,
    }


def report(result):
    matrix = result["camera_matrix"]
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    width, height = result["size"]

    print("")
    print("used %d of %d frames" % (result["used"], result["total"]))
    print("RMS reprojection error: %.4f px" % result["rms"])
    print("per-view error:")
    for name, method, error in result["per_view"]:
        print("  %-28s %-8s %.4f px" % (name, method, error))
    print("focal length: fx=%.3f fy=%.3f" % (fx, fy))
    print("principal point: cx=%.3f cy=%.3f (image centre %.1f, %.1f)"
          % (cx, cy, width / 2.0, height / 2.0))

    warnings = []
    if result["rms"] > MAX_RMS_PX:
        warnings.append("RMS %.4f px exceeds %.1f px; recapture sharper, more varied views"
                        % (result["rms"], MAX_RMS_PX))
    if result["used"] < GOOD_VIEWS:
        warnings.append("only %d views (%.0f recommended); more views improve the solve"
                        % (result["used"], GOOD_VIEWS))
    if abs(cx - width / 2.0) > MAX_PRINCIPAL_OFFSET * width:
        warnings.append("principal point cx is %.1f%% of the width off centre"
                        % (100.0 * abs(cx - width / 2.0) / width))
    if abs(cy - height / 2.0) > MAX_PRINCIPAL_OFFSET * height:
        warnings.append("principal point cy is %.1f%% of the height off centre"
                        % (100.0 * abs(cy - height / 2.0) / height))
    for warning in warnings:
        print("WARNING: " + warning)


def _f(value):
    return "%.6ff" % float(value)


def format_xml(result, vid, pid):
    """Emit the <Camera> element in teamwebcamcalibrations.xml form.

    OpenCV returns distortion as [k1, k2, p1, p2, k3, k4, k5, k6]. The SDK
    documents [r0, r1, t0, t1, r2, r3, r4, r5] with r radial and t tangential,
    i.e. the same sequence: k1, k2, p1, p2, k3, k4, k5, k6.
    """
    matrix = result["camera_matrix"]
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    dist = list(result["dist"][:8])
    while len(dist) < 8:
        dist.append(0.0)
    width, height = result["size"]

    lines = [
        '<Camera vid="%s" pid="%s">' % (vid, pid),
        "    <Calibration",
        '        size="%d %d"' % (width, height),
        '        focalLength="%s, %s"' % (_f(fx), _f(fy)),
        '        principalPoint="%s, %s"' % (_f(cx), _f(cy)),
        '        distortionCoefficients="%s"' % ", ".join(_f(value) for value in dist),
        "        />",
        "</Camera>",
    ]
    return "\n".join(lines)


def render_board(camera_matrix, rvec, tvec, cols, rows, square_mm, size,
                 square_px=40, margin_squares=1):
    """Render a checkerboard from a pose; None when it is not fully in view."""
    width, height = size
    squares_x = cols + 1
    squares_y = rows + 1
    board_w = (squares_x + 2 * margin_squares) * square_px
    board_h = (squares_y + 2 * margin_squares) * square_px

    board = np.full((board_h, board_w), 255, np.uint8)
    for row in range(squares_y):
        for col in range(squares_x):
            if (row + col) % 2 == 0:
                x0 = (col + margin_squares) * square_px
                y0 = (row + margin_squares) * square_px
                board[y0:y0 + square_px, x0:x0 + square_px] = 0

    offset = margin_squares * square_mm
    extent_x = (squares_x + margin_squares) * square_mm
    extent_y = (squares_y + margin_squares) * square_mm
    object_corners = np.array([
        [-offset, -offset, 0.0],
        [extent_x, -offset, 0.0],
        [extent_x, extent_y, 0.0],
        [-offset, extent_y, 0.0],
    ], np.float32)

    rotation, _ = cv2.Rodrigues(np.asarray(rvec, np.float64))
    camera_points = (rotation @ object_corners.T).T + np.asarray(tvec, np.float64).reshape(1, 3)
    if np.any(camera_points[:, 2] <= 1.0):
        return None

    projected, _ = cv2.projectPoints(object_corners, rvec, tvec, camera_matrix, None)
    projected = projected.reshape(-1, 2).astype(np.float32)
    if np.any(projected[:, 0] < 5) or np.any(projected[:, 0] > width - 6):
        return None
    if np.any(projected[:, 1] < 5) or np.any(projected[:, 1] > height - 6):
        return None
    if cv2.contourArea(projected) < 0.12 * width * height:
        return None

    source = np.array([[0, 0], [board_w, 0], [board_w, board_h], [0, board_h]], np.float32)
    transform = cv2.getPerspectiveTransform(source, projected)
    return cv2.warpPerspective(board, transform, (width, height),
                               borderMode=cv2.BORDER_CONSTANT, borderValue=128)


def run_selftest(args):
    width, height = args.size if args.size else (640, 480)
    truth_f = 0.82 * width
    truth_cx = width / 2.0 - 3.0
    truth_cy = height / 2.0 + 4.0
    camera_matrix = np.array([
        [truth_f, 0.0, truth_cx],
        [0.0, truth_f, truth_cy],
        [0.0, 0.0, 1.0],
    ], np.float64)

    rng = np.random.default_rng(20261106)
    frames = []
    attempts = 0
    while len(frames) < GOOD_VIEWS and attempts < GOOD_VIEWS * 60:
        attempts += 1
        rvec = rng.uniform(-0.45, 0.45, 3)
        tvec = np.array([
            rng.uniform(-0.10, 0.10) * 450.0,
            rng.uniform(-0.10, 0.10) * 450.0,
            rng.uniform(340.0, 620.0),
        ])
        image = render_board(camera_matrix, rvec, tvec, args.cols, args.rows,
                             args.square_mm, (width, height))
        if image is None:
            continue
        frames.append({"name": "synth_%02d.png" % len(frames), "image": image})

    print("selftest: rendered %d synthetic views in %d attempts" % (len(frames), attempts))
    if len(frames) < GOOD_VIEWS:
        raise SystemExit("selftest could not render enough views")

    result = calibrate(frames, args.cols, args.rows, args.square_mm, (width, height),
                       debug_dir=args.debug_dir)
    report(result)

    matrix = result["camera_matrix"]
    for label, value in (("fx", matrix[0, 0]), ("fy", matrix[1, 1])):
        error = abs(float(value) - truth_f) / truth_f
        print("selftest %s=%.3f truth=%.3f error=%.3f%%" % (label, value, truth_f, 100.0 * error))
        if error > 0.02:
            raise SystemExit("selftest FAILED: %s off by %.2f%% (> 2%%)" % (label, 100.0 * error))

    xml = format_xml(result, args.vid, args.pid)
    if '<Camera vid="%s" pid="%s">' % (args.vid, args.pid) not in xml:
        raise SystemExit("selftest FAILED: XML lacks the camera ids")
    distortion = re.search(r'distortionCoefficients="([^"]+)"', xml)
    focal = re.search(r'focalLength="([^"]+)"', xml)
    principal = re.search(r'principalPoint="([^"]+)"', xml)
    if not distortion or not focal or not principal:
        raise SystemExit("selftest FAILED: XML lacks calibration attributes")
    coefficients = distortion.group(1).split(", ")
    if len(coefficients) != 8 or not all(value.endswith("f") for value in coefficients):
        raise SystemExit("selftest FAILED: expected 8 float coefficients with an f suffix")
    if not all(value.endswith("f") for value in focal.group(1).split(", ")):
        raise SystemExit("selftest FAILED: focalLength lacks the f suffix")
    print("selftest XML:")
    print(xml)
    print("selftest PASSED")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Solve webcam intrinsics from MonoCalibrationCapture frames.")
    parser.add_argument("image_dir", nargs="?",
                        help="directory holding calib_<W>x<H>_<NN>.png frames")
    parser.add_argument("--cols", type=int, default=9, help="inner corners across (default 9)")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down (default 6)")
    parser.add_argument("--square-mm", type=float, default=25.0,
                        help="checker square edge in mm (default 25)")
    parser.add_argument("--size", type=parse_size, default=None,
                        help="capture resolution to use, e.g. 640x480 (default: most common)")
    parser.add_argument("--vid", default="0x0C45", help="USB vendor id (default 0x0C45)")
    parser.add_argument("--pid", default="0x6366", help="USB product id (default 0x6366)")
    parser.add_argument("--out", default="calib_result.xml",
                        help="XML output path (default calib_result.xml)")
    parser.add_argument("--debug-dir", default=None,
                        help="write frames with detected corners into this directory")
    parser.add_argument("--selftest", action="store_true",
                        help="check the pipeline against a synthetic camera")
    args = parser.parse_args(argv)

    if args.selftest:
        return run_selftest(args)

    if not args.image_dir:
        parser.error("image_dir is required unless --selftest is given")
    if args.cols < 2 or args.rows < 2:
        parser.error("--cols and --rows must be at least 2")

    size, selected = discover_frames(args.image_dir, args.size)
    print("using %dx%d frames from %s (%d candidates)"
          % (size[0], size[1], args.image_dir, len(selected)))
    frames = load_frames(args.image_dir, size, selected)
    if not frames:
        raise SystemExit("no readable frames at %dx%d" % size)

    result = calibrate(frames, args.cols, args.rows, args.square_mm, size,
                       debug_dir=args.debug_dir)
    report(result)

    xml = format_xml(result, args.vid, args.pid)
    print("")
    print(xml)
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write(xml + "\n")
    print("")
    print("wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
