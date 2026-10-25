#!/usr/bin/env python3
"""Compute CSV metrics for time-aligned target and independently observed Gazebo GT."""

import argparse
from bisect import bisect_left, bisect_right
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

if __package__:
    from .trajectory_io import NANOSECONDS, load_trajectory, sha256_file, write_json
else:
    from trajectory_io import NANOSECONDS, load_trajectory, sha256_file, write_json


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Comparison:
    person_id: str
    timestamp: float
    target_x: float
    target_y: float
    gt_x: float
    gt_y: float
    position_error: float
    absolute_yaw_error: float


def compare_samples(target, ground_truth, start_ns, *, interpolation="previous", max_gap=0.5,
                    allow_partial=False):
    """Evaluate the target at each GT time; no extrapolation or fitted time shift.

    Previous-sample hold matches the current controller. Linear interpolation
    optionally evaluates a continuous desired path, with shortest-arc yaw.
    """
    if not target or not ground_truth:
        raise ValueError("target and ground truth must both contain observations")
    if interpolation not in ("previous", "linear") or not math.isfinite(max_gap) or max_gap <= 0:
        raise ValueError("invalid interpolation or max_gap")
    times = [start_ns + sample.timestamp_ns for sample in target]
    gt_times = [sample.timestamp_ns for sample in ground_truth]
    if any(b <= a for a, b in zip(times, times[1:])) or any(b <= a for a, b in zip(gt_times, gt_times[1:])):
        raise ValueError("timestamps must increase strictly per person")
    complete = gt_times[0] <= times[0] and gt_times[-1] >= times[-1]
    if not complete and not allow_partial:
        raise ValueError("GT does not cover the complete target time range; record before playback and stop after it, or explicitly use --allow-partial")
    if gt_times[0] > times[-1] or gt_times[-1] < times[0]:
        raise ValueError("no target/GT time overlap; check the replay metadata and experiment")
    limit = max_gap * NANOSECONDS
    for a, b in zip(gt_times, gt_times[1:]):
        if b > times[0] and a < times[-1] and b-a > limit:
            raise ValueError("GT dropout exceeds max_gap within the target interval")
    comparisons = []
    # The controller holds the final goal indefinitely. Include the first GT
    # observation at/after its deadline so a missing last command cannot hide
    # between differently phased sampling grids. No motion is extrapolated.
    endpoint_index = bisect_left(gt_times, times[-1])
    endpoint_time = gt_times[endpoint_index] if endpoint_index < len(gt_times) else None
    if endpoint_time is not None and endpoint_time-times[-1] > limit:
        raise ValueError("GT endpoint observation is too far from the final target deadline")
    for observed in ground_truth:
        when = observed.timestamp_ns
        if when < times[0] or (when > times[-1] and when != endpoint_time):
            continue
        index = bisect_right(times, when) - 1
        desired = target[index]
        x, y, yaw = desired.x, desired.y, desired.yaw
        if index+1 < len(target):
            span = times[index+1]-times[index]
            if span > limit:
                raise ValueError("target gap exceeds max_gap; choose a limit appropriate to the sampling rate")
            if interpolation == "linear":
                fraction = (when-times[index]) / span
                after = target[index+1]
                x += fraction * (after.x-x)
                y += fraction * (after.y-y)
                yaw += fraction * wrap_angle(after.yaw-yaw)
        comparisons.append(Comparison(
            observed.person_id, (when-start_ns)/NANOSECONDS, x, y,
            observed.x, observed.y, math.hypot(observed.x-x, observed.y-y),
            abs(wrap_angle(observed.yaw-yaw)),
        ))
    if not comparisons:
        raise ValueError("no target/GT time overlap; check the replay metadata and experiment")
    return comparisons, complete


def summarize(person, comparisons, complete):
    errors = [row.position_error for row in comparisons]
    return {
        "person_id": person, "samples": len(errors), "full_time_coverage": complete,
        "mean_position_error_m": math.fsum(errors)/len(errors),
        "rmse_m": math.sqrt(math.fsum(value*value for value in errors)/len(errors)),
        "maximum_position_error_m": max(errors),
        "mean_absolute_yaw_error_rad": math.fsum(row.absolute_yaw_error for row in comparisons)/len(errors),
    }


def validate_experiment(experiment_dir, output_dir=None, *, target_path=None, gt_path=None,
                        interpolation="previous", max_gap=0.5, allow_partial=False,
                        overwrite=False):
    directory = Path(experiment_dir)
    target_path = Path(target_path) if target_path else directory / "target.csv"
    gt_path = Path(gt_path) if gt_path else directory / "ground_truth.csv"
    if target_path.resolve() == gt_path.resolve():
        raise ValueError("target and GT must be separate files")
    gt_metadata = json.loads((directory / "ground_truth_metadata.json").read_text())
    playback = json.loads((directory / "playback.json").read_text())
    if gt_metadata.get("gt_source") != "gazebo_rendered_actor_world_pose":
        raise ValueError("GT metadata must identify the actual Gazebo Actor WorldPose source")
    if gt_metadata.get("status") != "complete":
        raise ValueError("stop the recorder cleanly before validation; its status must be complete")
    if gt_metadata.get("experiment_id") != playback.get("experiment_id"):
        raise ValueError("target and GT belong to different experiments")
    if gt_metadata.get("frame_id") != "world" or playback.get("frame_id") != "world":
        raise ValueError("both trajectories must use the world coordinate frame")
    if gt_metadata.get("time_basis") != "simulation_absolute" or playback.get("time_basis") != "simulation_absolute":
        raise ValueError("both recordings must share the simulation time epoch")
    if playback.get("target_time_basis") != "relative_seconds":
        raise ValueError("unsupported target time convention")
    if sha256_file(target_path) != playback.get("target_sha256"):
        raise ValueError("target CSV differs from the exact file used in playback")
    start_ns = playback.get("start_ros_time_ns")
    if not isinstance(start_ns, int) or start_ns <= 0:
        raise ValueError("missing valid replay start time")
    mappings = gt_metadata["mappings"]
    aliases = {actor: person for person, actor in mappings.items()}
    aliases.update({person: person for person in mappings})
    targets, truth = {}, {}
    raw_ids = {}
    for sample in load_trajectory(target_path):
        person = aliases.get(sample.person_id)
        if person is None:
            raise ValueError(f"target ID has no actor mapping: {sample.person_id}")
        if person in raw_ids and raw_ids[person] != sample.person_id:
            raise ValueError("multiple target IDs map to the same actor")
        raw_ids[person] = sample.person_id
        targets.setdefault(person, []).append(sample)
    for sample in load_trajectory(gt_path):
        if sample.person_id not in mappings:
            raise ValueError(f"unknown GT ID: {sample.person_id}")
        truth.setdefault(sample.person_id, []).append(sample)
    comparisons, summaries = {}, []
    for person, points in targets.items():
        rows, complete = compare_samples(points, truth.get(person, []), start_ns,
            interpolation=interpolation, max_gap=max_gap, allow_partial=allow_partial)
        comparisons[person] = rows
        summaries.append(summarize(person, rows, complete))

    # Do not create result artifacts until all input and alignment checks pass.
    output = Path(output_dir) if output_dir else directory / "validation"
    protected = [target_path.resolve(), gt_path.resolve()]
    names = ("validation_results.csv", "aligned_samples.csv", "validation_metadata.json")
    if any((output/name).resolve() in protected for name in names):
        raise ValueError("output would replace an input CSV")
    if not overwrite and any((output/name).exists() for name in names):
        raise ValueError("validation outputs already exist; choose another output directory or --overwrite")
    output.mkdir(parents=True, exist_ok=True)
    with (output/"validation_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    with (output/"aligned_samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(Comparison.__dataclass_fields__))
        writer.writeheader()
        for rows in comparisons.values():
            writer.writerows(vars(row) for row in rows)
    write_json(output/"validation_metadata.json", {
        "experiment_id": gt_metadata["experiment_id"], "frame_id": "world",
        "start_ros_time_ns": start_ns, "interpolation": interpolation,
        "max_gap_s": max_gap, "target_sha256": sha256_file(target_path),
        "gt_sha256": sha256_file(gt_path), "allow_partial": allow_partial,
        "evaluation": "target evaluated at GT sample times, including the first post-end observation of the held final goal; no fitted offset or motion extrapolation",
    })
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment-dir", type=Path, required=True)
    parser.add_argument("--target", type=Path, help="default: experiment target.csv")
    parser.add_argument("--ground-truth", type=Path, help="default: experiment ground_truth.csv")
    parser.add_argument("--output-dir", type=Path, help="default: experiment/validation")
    parser.add_argument("--interpolation", choices=("previous", "linear"), default="previous")
    parser.add_argument("--max-gap", type=float, default=0.5, help="largest allowed target/GT sample gap in seconds")
    parser.add_argument("--allow-partial", action="store_true", help="explicitly permit partial time coverage")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    try:
        summaries = validate_experiment(args.experiment_dir, args.output_dir,
            target_path=args.target, gt_path=args.ground_truth,
            interpolation=args.interpolation, max_gap=args.max_gap,
            allow_partial=args.allow_partial, overwrite=args.overwrite)
    except (OSError, ValueError, KeyError, ImportError) as error:
        parser.error(str(error))
    for summary in summaries:
        print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
