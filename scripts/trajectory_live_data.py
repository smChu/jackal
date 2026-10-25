"""Time-aligned live comparison with explicitly labelled linear GT estimates."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math

if __package__:
    from .trajectory_io import NANOSECONDS, TrajectorySample
else:
    from trajectory_io import NANOSECONDS, TrajectorySample


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate(points, times, when, method="linear"):
    """Return x/y/yaw, interpolation flag; never extrapolate missing endpoints."""
    if not points or when < times[0] or when > times[-1]:
        raise ValueError("interpolation requires observations on both sides")
    index = bisect_right(times, when)-1
    before = points[index]
    if when == times[index] or method == "previous":
        return before.x, before.y, before.yaw, False
    after = points[index+1]
    fraction = (when-times[index])/(times[index+1]-times[index])
    return (before.x+fraction*(after.x-before.x),
            before.y+fraction*(after.y-before.y),
            wrap(before.yaw+fraction*wrap(after.yaw-before.yaw)), True)


@dataclass(frozen=True)
class LiveComparison:
    person_id: str
    timestamp: float
    target_x: float
    target_y: float
    target_yaw: float
    gt_x: float
    gt_y: float
    gt_yaw: float
    gt_interpolated: bool
    position_error: float
    absolute_yaw_error: float


class LiveTracks:
    def __init__(self, samples, mappings, rate=50.0, history_limit=20000):
        if not math.isfinite(rate) or not 0 < rate <= 1000:
            raise ValueError("interpolation rate must be between 0 and 1000 Hz")
        aliases = {actor: person for person, actor in mappings.items()}
        aliases.update({person: person for person in mappings})
        self.targets = {}
        originals = {}
        for sample in samples:
            person = aliases.get(sample.person_id)
            if person is None:
                raise ValueError(f"CSV ID has no actor mapping: {sample.person_id}")
            if person in originals and originals[person] != sample.person_id:
                raise ValueError("multiple CSV IDs map to one actor")
            originals[person] = sample.person_id
            self.targets.setdefault(person, []).append(TrajectorySample(
                person, sample.timestamp_ns, sample.x, sample.y, sample.yaw))
        self.history = {person: [] for person in self.targets}
        self.start_ns = None
        self.step_ns = max(1, round(NANOSECONDS/rate))
        self.history_limit = history_limit
        self.version = 0

    def add(self, sample):
        if sample.person_id not in self.history:
            return
        history = self.history[sample.person_id]
        if history and sample.timestamp_ns <= history[-1].timestamp_ns:
            raise ValueError("GT observations must increase in simulation time")
        history.append(sample)
        if len(history) > self.history_limit:
            del history[:len(history)-self.history_limit]
        self.version += 1

    def set_origin(self, start_ns):
        if not isinstance(start_ns, int) or start_ns <= 0:
            raise ValueError("invalid replay time origin")
        if self.start_ns is not None and self.start_ns != start_ns:
            raise ValueError("new playback requires a fresh session")
        self.start_ns = start_ns
        self.version += 1

    def compare(self, person, target_method="linear"):
        if target_method not in ("linear", "previous"):
            raise ValueError("target interpolation must be linear or previous")
        observed = self.history[person]
        target = self.targets[person]
        if self.start_ns is None or not observed:
            return [], False
        target_times = [self.start_ns+p.timestamp_ns for p in target]
        observed_times = [p.timestamp_ns for p in observed]
        complete = observed_times[0] <= target_times[0] and observed_times[-1] >= target_times[-1]
        # The first command can teleport an actor from its waiting position.
        # Do not turn that pre-playback placement into a fictitious walking line.
        start_index = bisect_left(observed_times, target_times[0])
        observed = observed[start_index:]
        observed_times = observed_times[start_index:]
        if not observed:
            return [], False
        lower = max(target_times[0], observed_times[0])
        upper = min(target_times[-1], observed_times[-1])
        if upper < lower:
            return [], False
        # Bound graph work for long runs, while retaining both ends of the view.
        step = max(self.step_ns, math.ceil((upper-lower)/4999))
        first = target_times[0] + max(0, (lower-target_times[0]+step-1)//step)*step
        times = sorted({lower, upper, *range(first, upper+1, step)})
        rows = []
        for when in times:
            tx, ty, tyaw, _ = interpolate(target, target_times, when, target_method)
            gx, gy, gyaw, estimated = interpolate(observed, observed_times, when)
            rows.append(LiveComparison(person, (when-self.start_ns)/NANOSECONDS,
                tx, ty, tyaw, gx, gy, gyaw, estimated,
                math.hypot(gx-tx, gy-ty), abs(wrap(gyaw-tyaw))))
        return rows, complete

    def measured_in_view(self, person):
        history = self.history[person]
        if self.start_ns is None:
            return history[-1:]
        target = self.targets[person]
        times = [sample.timestamp_ns for sample in history]
        start = self.start_ns+target[0].timestamp_ns
        end = self.start_ns+target[-1].timestamp_ns
        return history[bisect_left(times, start):bisect_right(times, end)]


def live_summary(person, rows, complete):
    if not rows:
        return None
    return {
        "person_id": person, "samples": len(rows), "full_time_coverage": complete,
        "interpolated_gt_samples": sum(row.gt_interpolated for row in rows),
        "mean_position_error_m": math.fsum(row.position_error for row in rows)/len(rows),
        "rmse_m": math.sqrt(math.fsum(row.position_error**2 for row in rows)/len(rows)),
        "maximum_position_error_m": max(row.position_error for row in rows),
        "mean_absolute_yaw_error_rad": math.fsum(row.absolute_yaw_error for row in rows)/len(rows),
    }
