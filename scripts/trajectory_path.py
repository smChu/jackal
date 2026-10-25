"""Convert an editable world-XY polyline into the existing trajectory CSV contract."""

from bisect import bisect_right
import csv
from dataclasses import asdict
import io
import math
import re

if __package__:
    from .generate_trajectory import CSV_FIELDS, TrajectoryPoint
else:
    from generate_trajectory import CSV_FIELDS, TrajectoryPoint


MAX_WAYPOINTS = 2000
MAX_SAMPLES = 50000


def clean_points(points):
    """Remove consecutive coincident points, preserving corners and closed loops."""
    result = []
    for point in points:
        if len(point) != 2:
            raise ValueError("Each waypoint must contain x and y")
        x, y = map(float, point)
        if not math.isfinite(x) or not math.isfinite(y) or max(abs(x), abs(y)) > 1e6:
            raise ValueError("Waypoint coordinates must be finite and within +/-1000000 m")
        if not result or math.hypot(x-result[-1][0], y-result[-1][1]) > 1e-9:
            result.append((x, y))
    if len(result) > MAX_WAYPOINTS:
        raise ValueError(f"At most {MAX_WAYPOINTS} waypoints are supported")
    return result


def path_length(points):
    return math.fsum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


def build_path_trajectory(points, person_id="human_1", speed=0.5, rate=20.0):
    """Sample by arc length at constant speed; include the exact final point/time.

    Yaw follows each segment's direction. Corners stay sharp; the final sample
    retains the last segment's heading. No obstacle avoidance or curved fitting.
    """
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", person_id):
        raise ValueError("person_id must be a ROS topic token")
    if not math.isfinite(speed) or not 0 < speed <= 10:
        raise ValueError("Speed must be greater than 0 and at most 10 m/s")
    if not math.isfinite(rate) or not 1 <= rate <= 100:
        raise ValueError("Publication rate must be between 1 and 100 Hz")
    points = clean_points(points)
    if len(points) < 2:
        raise ValueError("Draw at least two different waypoints")
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    length = distances[-1]
    duration = length/speed
    # A one-nanosecond minimum preserves strictly increasing CSV timestamps.
    if duration < 1e-9 or duration > 3600:
        raise ValueError("Path duration must be between 1 ns and 3600 seconds")
    intervals = math.ceil(duration*rate)
    if intervals+len(points) > MAX_SAMPLES:
        raise ValueError(f"Too many trajectory samples (limit {MAX_SAMPLES}); increase speed or reduce rate")
    # Include every waypoint arrival as well as the regular time grid. Otherwise
    # a corner between grid ticks would be cut off by the sampled target path.
    timeline = {round(min(index/rate, duration)*1e9): min(index/rate, duration)
                for index in range(intervals+1)}
    timeline.update({round(distance/speed*1e9): distance/speed for distance in distances})
    if len(timeline) < 2:
        raise ValueError("Path duration is too short for nanosecond timestamps")
    samples = []
    for index, (_, elapsed) in enumerate(sorted(timeline.items())):
        travelled = min(elapsed*speed, length)
        segment = min(len(points)-2, bisect_right(distances, travelled)-1)
        a, b = points[segment], points[segment+1]
        fraction = (travelled-distances[segment])/(distances[segment+1]-distances[segment])
        x = a[0]+fraction*(b[0]-a[0])
        y = a[1]+fraction*(b[1]-a[1])
        if index == len(timeline)-1:
            x, y = points[-1]
        samples.append(TrajectoryPoint(person_id, elapsed, x, y, math.atan2(b[1]-a[1], b[0]-a[0])))
    return samples


def trajectory_csv_bytes(samples):
    """Serialize the same fields as Task 3-1, ready for Publisher snapshots."""
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
    writer.writeheader()
    for sample in samples:
        writer.writerow(asdict(sample))
    return stream.getvalue().encode("utf-8")
