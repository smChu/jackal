"""Shared CSV contract, independent of ROS and Gazebo."""

import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re


NANOSECONDS = 1_000_000_000
FIELDS = ("person_id", "timestamp", "x", "y", "yaw")


@dataclass(frozen=True)
class TrajectorySample:
    person_id: str
    timestamp_ns: int
    x: float
    y: float
    yaw: float


def load_trajectory(path):
    """Validate the complete file before playback; timestamps are relative seconds.

    Times must increase strictly per person (at nanosecond precision). Multiple
    people may share a timestamp; interleaved rows are merged in time order.
    """
    samples = []
    previous = {}
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or set(reader.fieldnames) != set(FIELDS) or len(reader.fieldnames) != len(FIELDS):
            raise ValueError(f"CSV header must contain exactly: {','.join(FIELDS)}")
        for line, row in enumerate(reader, start=2):
            try:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("incorrect number of columns")
                person = row["person_id"].strip()
                if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", person):
                    raise ValueError("person_id must be a ROS topic token, e.g. walking_person or human_1")
                timestamp, x, y, yaw = (float(row[field]) for field in FIELDS[1:])
                if not all(math.isfinite(value) for value in (timestamp, x, y, yaw)):
                    raise ValueError("numeric values must be finite")
                if timestamp < 0 or timestamp > 2_147_483_647:
                    raise ValueError("timestamp must be between 0 and 2147483647 seconds")
                timestamp_ns = round(timestamp * NANOSECONDS)
                if timestamp_ns <= previous.get(person, -1):
                    raise ValueError("timestamps must increase strictly for each person")
                previous[person] = timestamp_ns
                samples.append(TrajectorySample(person, timestamp_ns, x, y, yaw))
            except ValueError as error:
                raise ValueError(f"CSV row {line}: {error}") from error
    if not samples:
        raise ValueError("CSV contains no trajectory samples")
    return sorted(samples, key=lambda sample: sample.timestamp_ns)


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)
