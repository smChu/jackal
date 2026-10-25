#!/usr/bin/env python3
"""Publish CSV human trajectories as ROS 2 PoseStamped target messages."""

import argparse
import json
import math
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args


if __package__:
    from .trajectory_io import NANOSECONDS, FIELDS, TrajectorySample, load_trajectory, sha256_file, write_json
else:
    from trajectory_io import NANOSECONDS, FIELDS, TrajectorySample, load_trajectory, sha256_file, write_json


class ReplaySchedule:
    """Clock-independent scheduling, with no cumulative sleep drift."""

    def __init__(self, samples, start_delay=1.0):
        if not math.isfinite(start_delay) or start_delay < 0:
            raise ValueError("start_delay must be finite and nonnegative")
        self.samples = samples
        self.delay_ns = round(start_delay * NANOSECONDS)
        self.start_ns = None
        self.previous_ns = None
        self.index = 0

    @property
    def finished(self):
        return self.index == len(self.samples)

    def due(self, now_ns):
        if self.previous_ns is not None and now_ns < self.previous_ns:
            raise RuntimeError("ROS time moved backwards. Restart the publisher after resetting the simulation.")
        self.previous_ns = now_ns
        if self.start_ns is None:
            self.start_ns = now_ns + self.delay_ns
        due = []
        while not self.finished:
            sample = self.samples[self.index]
            target_ns = self.start_ns + sample.timestamp_ns
            if target_ns > now_ns:
                break
            due.append((sample, target_ns))
            self.index += 1
        return due


def target_pose(sample, target_ns, frame_id="world", z=0.0):
    message = PoseStamped()
    message.header.stamp = Time(nanoseconds=target_ns).to_msg()
    message.header.frame_id = frame_id
    message.pose.position.x = sample.x
    message.pose.position.y = sample.y
    message.pose.position.z = z
    message.pose.orientation.z = math.sin(sample.yaw / 2)
    message.pose.orientation.w = math.cos(sample.yaw / 2)
    return message


class TrajectoryPublisher(Node):
    def __init__(self, samples, *, start_delay=1.0, frame_id="world", z=0.0,
                 wait_for_subscribers=False, experiment_dir=None, target_path=None):
        super().__init__("trajectory_publisher")
        self.schedule = ReplaySchedule(samples, start_delay)
        self.frame_id = frame_id
        self.z = z
        self.wait_for_subscribers = wait_for_subscribers
        self.finished = False
        self.experiment_dir = Path(experiment_dir) if experiment_dir else None
        self.experiment_metadata = None
        if self.experiment_dir:
            if not self.get_parameter("use_sim_time").value:
                raise ValueError("experiment recording requires --ros-args -p use_sim_time:=true")
            metadata = json.loads((self.experiment_dir / "ground_truth_metadata.json").read_text())
            if metadata["frame_id"] != frame_id or metadata["time_basis"] != "simulation_absolute":
                raise ValueError("recorder and publisher must share frame and simulation time")
            for name in ("target.csv", "playback.json"):
                if (self.experiment_dir / name).exists():
                    raise ValueError(f"{name} already exists; use a new experiment directory")
            if not target_path:
                raise ValueError("target_path required for experiment metadata")
            with (self.experiment_dir / "target.csv").open("xb") as destination:
                destination.write(Path(target_path).read_bytes())
            self.experiment_metadata = {
                "schema_version": 1, "experiment_id": metadata["experiment_id"],
                "frame_id": frame_id, "time_basis": "simulation_absolute",
                "target_time_basis": "relative_seconds", "interpolation": "previous",
                "target_sha256": sha256_file(self.experiment_dir / "target.csv"),
                "person_ids": sorted({sample.person_id for sample in samples}),
            }
        self.publishers_by_person = {}
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        for person in sorted({sample.person_id for sample in samples}):
            topic = f"/humans/{person}/target_pose"
            self.publishers_by_person[person] = self.create_publisher(PoseStamped, topic, qos)
            self.get_logger().info(f"Target topic: {topic} (geometry_msgs/msg/PoseStamped)")
        # A steady timer keeps polling through clock initialization, pauses and
        # resets. Publication deadlines always use the node's ROS clock.
        self.poll_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.005, self.publish_due, clock=self.poll_clock)
        self.get_logger().info(
            f"Loaded {len(samples)} samples. Start delay: {start_delay}s. "
            "Waiting for clock/subscribers if requested."
        )

    def publish_due(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.schedule.start_ns is None:
            if self.get_parameter("use_sim_time").value and now_ns == 0:
                return
            if self.wait_for_subscribers and any(
                publisher.get_subscription_count() == 0
                for publisher in self.publishers_by_person.values()
            ):
                return
        first_tick = self.schedule.start_ns is None
        due = self.schedule.due(now_ns)
        if first_tick and self.experiment_dir:
            self.experiment_metadata["start_ros_time_ns"] = self.schedule.start_ns
            write_json(self.experiment_dir / "playback.json", self.experiment_metadata)
            self.get_logger().info(f"Experiment time origin saved: {self.experiment_dir / 'playback.json'}")
        for sample, target_ns in due:
            self.publishers_by_person[sample.person_id].publish(
                target_pose(sample, target_ns, self.frame_id, self.z)
            )
        if self.schedule.finished:
            self.timer.cancel()
            self.finished = True
            self.get_logger().info(f"Playback complete: {len(self.schedule.samples)} samples published.")


def main(argv=None):
    argv = sys.argv if argv is None else [sys.argv[0], *argv]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True, help="input trajectory CSV")
    parser.add_argument("--start-delay", type=float, default=1.0, help="ROS seconds to wait before trajectory time zero (default: 1)")
    parser.add_argument("--frame-id", default="world", help="target coordinate frame (default: world)")
    parser.add_argument("--z", type=float, default=0.0, help="planar target height in meters (default: 0)")
    parser.add_argument("--wait-for-subscribers", action="store_true", help="wait for a subscriber on every target topic before starting")
    parser.add_argument("--experiment-dir", type=Path, help="save target copy and exact replay time origin into a running recorder experiment")
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    try:
        samples = load_trajectory(args.csv)
        if not math.isfinite(args.start_delay) or args.start_delay < 0:
            raise ValueError("--start-delay must be finite and nonnegative")
        if not math.isfinite(args.z):
            raise ValueError("--z must be finite")
        if not args.frame_id.strip():
            raise ValueError("--frame-id must be nonempty")
    except (OSError, ValueError) as error:
        parser.error(str(error))

    node = None
    rclpy.init(args=argv)
    try:
        node = TrajectoryPublisher(
            samples, start_delay=args.start_delay, frame_id=args.frame_id,
            z=args.z, wait_for_subscribers=args.wait_for_subscribers,
            experiment_dir=args.experiment_dir, target_path=args.csv,
        )
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.finished:
            for publisher in node.publishers_by_person.values():
                if not publisher.wait_for_all_acked(Duration(seconds=1)):
                    node.get_logger().warning("Timed out waiting for subscriber acknowledgment.")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, ValueError, OSError, KeyError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
