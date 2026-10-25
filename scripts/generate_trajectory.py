#!/usr/bin/env python3
"""Generate planar human trajectories with only the Python standard library."""

import argparse
import csv
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterator


CSV_FIELDS = ("person_id", "timestamp", "x", "y", "yaw")


@dataclass(frozen=True)
class TrajectoryPoint:
    person_id: str
    timestamp: float
    x: float
    y: float
    yaw: float


def generate_trajectory(
    *,
    person_id: str = "walking_person",
    scenario: str = "straight",
    duration: float = 4.0,
    rate: float = 10.0,
    start_time: float = 0.0,
    start_x: float = 0.0,
    start_y: float = 0.0,
    yaw: float = 0.0,
    distance: float = 2.0,
    radius: float = 1.0,
    turn_angle: float = math.pi / 2,
    side: float = 2.0,
) -> Iterator[TrajectoryPoint]:
    """Yield constant-speed samples, including both start and exact endpoint.

    Coordinates are meters in the world XY plane; angles are radians. Positive
    turns are counterclockwise. ``yaw`` is the initial heading. ``start_time``
    is an optional offset in seconds, not a wall-clock timestamp. The circle
    scenario completes one counterclockwise revolution, starting at (start_x,
    start_y), with its center to the left of the initial heading.
    """
    if not isinstance(person_id, str) or not person_id.strip():
        raise ValueError("person_id must be a nonempty string")
    if scenario not in ("straight", "turn", "circle", "square"):
        raise ValueError("scenario must be straight, turn, circle, or square")
    values = (duration, rate, start_time, start_x, start_y, yaw,
              distance, radius, turn_angle, side)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("all numeric parameters must be finite")
    if duration <= 0 or rate <= 0:
        raise ValueError("duration and rate must be positive")
    if start_time < 0:
        raise ValueError("start_time must be nonnegative")
    if scenario == "straight" and distance <= 0:
        raise ValueError("distance must be positive")
    if scenario != "straight" and radius <= 0:
        raise ValueError("radius must be positive")
    if scenario == "turn" and turn_angle == 0:
        raise ValueError("turn_angle must be nonzero")
    if scenario == "square" and side <= 0:
        raise ValueError("side must be positive")
    if not math.isfinite(duration * rate):
        raise ValueError("duration * rate must be finite")
    if not math.isfinite(start_time + duration) or start_time + duration <= start_time:
        raise ValueError("start_time + duration must be finite and exceed start_time")

    def sample(elapsed: float) -> TrajectoryPoint:
        fraction = elapsed / duration
        if scenario == "straight":
            local_x, local_y = distance * fraction, 0.0
            heading = yaw
        elif scenario == "square":
            progress = min(4.0, 4 * fraction)
            segment = min(3, int(progress))
            fraction_on_side = progress - segment
            corners = ((0, 0), (side, 0), (side, side), (0, side), (0, 0))
            a, b = corners[segment], corners[segment+1]
            local_x = a[0] + fraction_on_side * (b[0]-a[0])
            local_y = a[1] + fraction_on_side * (b[1]-a[1])
            heading = yaw + segment * math.pi / 2
        else:
            angle = (2 * math.pi if scenario == "circle" else turn_angle) * fraction
            direction = 1 if scenario == "circle" or turn_angle > 0 else -1
            local_x = direction * radius * math.sin(angle)
            local_y = direction * radius * (1 - math.cos(angle))
            heading = yaw + angle
        # Rotate the path from its local frame into the world XY plane.
        x = start_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        y = start_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
        heading = (heading + math.pi) % (2 * math.pi) - math.pi
        return TrajectoryPoint(person_id, start_time + elapsed, x, y, heading)

    for index in range(math.ceil(duration * rate)):
        elapsed = index / rate
        if elapsed >= duration:
            break
        yield sample(elapsed)
    yield sample(duration)


def save_csv(points: Iterator[TrajectoryPoint], output: Path) -> int:
    """Save samples as UTF-8 CSV and return the number of data rows."""
    # Validate the generator before creating a file or replacing existing data.
    first = next(points)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerow(asdict(first))
        count = 1
        for point in points:
            writer.writerow(asdict(point))
            count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("straight", "turn", "circle", "square"), default="straight")
    parser.add_argument("--person-id", default="walking_person", help="CSV identifier; defaults to the current Gazebo actor name")
    parser.add_argument("--duration", type=float, default=4.0, help="total duration in seconds (default: 4)")
    parser.add_argument("--rate", type=float, default=10.0, help="sampling frequency in Hz (default: 10)")
    parser.add_argument("--start-time", type=float, default=0.0, help="timestamp offset in seconds")
    parser.add_argument("--start-x", type=float, default=0.0, help="initial world x in meters")
    parser.add_argument("--start-y", type=float, default=0.0, help="initial world y in meters")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="initial heading in degrees; 0 faces +X")
    parser.add_argument("--distance", type=float, default=2.0, help="straight path length in meters")
    parser.add_argument("--radius", type=float, default=1.0, help="turn/circle radius in meters")
    parser.add_argument("--turn-deg", type=float, default=90.0, help="turn angle in degrees; negative turns clockwise")
    parser.add_argument("--side", type=float, default=2.0, help="square side length in meters")
    parser.add_argument("--output", type=Path, help="CSV output path (default: trajectories/<scenario>.csv; existing file is replaced)")
    args = parser.parse_args()
    output = args.output or Path("trajectories") / f"{args.scenario}.csv"
    points = generate_trajectory(
        person_id=args.person_id, scenario=args.scenario, duration=args.duration,
        rate=args.rate, start_time=args.start_time, start_x=args.start_x,
        start_y=args.start_y, yaw=math.radians(args.yaw_deg),
        distance=args.distance, radius=args.radius,
        turn_angle=math.radians(args.turn_deg),
        side=args.side,
    )
    try:
        count = save_csv(points, output)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(f"Saved {count} samples for {args.person_id} to {output}")


if __name__ == "__main__":
    main()
