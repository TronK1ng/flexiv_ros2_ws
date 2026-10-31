"""Offline solver: samples.json -> handeye_calibration.yaml + report.

Usage:
    ros2 run flexiv_handeye_calib handeye_solve --samples ~/.ros/flexiv_handeye/samples.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

from . import solver, storage


def main(argv=None):
    parser = argparse.ArgumentParser(description="Solve eye-in-hand calibration from saved samples")
    parser.add_argument("--samples", required=True, help="path to samples.json")
    parser.add_argument("--output", default="", help="output yaml (default: next to samples.json)")
    parser.add_argument("--method", default="PARK", choices=list(solver._ALL_METHODS))
    parser.add_argument("--no-refine", action="store_true", help="skip non-linear refinement")
    parser.add_argument("--no-reject", action="store_true", help="keep all samples")
    parser.add_argument("--json", default="", help="also write the metrics as json")
    parser.add_argument("--no-normalize", action="store_true",
                        help="skip the chessboard symmetry realignment")
    args = parser.parse_args(argv)

    samples, meta = storage.load_samples(args.samples)
    if len(samples) < 3:
        print(f"error: {len(samples)} samples in {args.samples}, need at least 3", file=sys.stderr)
        return 2

    flipped = []
    if not args.no_normalize and meta.get("target_type", "") == "chessboard":
        from .chessboard import symmetry_transforms

        cols = int(meta.get("chessboard_cols", 5))
        rows = int(meta.get("chessboard_rows", 5))
        samples, flipped = solver.align_symmetric_targets(samples, symmetry_transforms(cols, rows))
        if flipped:
            print(f"symmetry    : realigned {len(flipped)} samples {flipped}")

    result = solver.solve(
        samples,
        method=args.method,
        do_refine=not args.no_refine,
        reject_outliers=not args.no_reject,
    )
    parent = meta.get("flange_frame", "flange")
    child = meta.get("camera_frame", "camera_link")
    print(storage.format_report(result, parent, child))
    print("verdict     : " + storage.verdict(result))

    output = args.output or os.path.join(
        os.path.dirname(os.path.abspath(args.samples)), "handeye_calibration.yaml"
    )
    calibration = storage.build_calibration(
        result, meta.get("robot_sn", ""), parent, child
    )
    storage.write_calibration_yaml(output, calibration)
    print(f"written     : {output}")

    if args.json:
        payload = {
            "X": result.X.tolist(),
            "Y": result.Y.tolist(),
            "metrics": result.metrics,
            "method_spread": result.method_spread,
            "outliers": result.outliers,
            "used": result.used,
        }
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, default=float)
        print(f"metrics     : {args.json}")

    return 0 if math.isfinite(result.metrics["translation_rms_mm"]) else 1
