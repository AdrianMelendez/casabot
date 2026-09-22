"""Name the places in your house, then tell the robot to go to one.

    ros2 run casabot places save kitchen     # remember where the robot is now
    ros2 run casabot places list
    ros2 run casabot places go kitchen
    ros2 run casabot places remove kitchen

Poses are stored in map frame, so they stay valid as long as you reuse the map
they were recorded against.
"""

import argparse
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener

from casabot.kinematics import quaternion_to_yaw, yaw_to_quaternion

DEFAULT_PLACES_FILE = os.path.expanduser('~/.casabot/places.yaml')


def load_places(path):
    if not os.path.exists(path):
        return {}
    with open(path) as handle:
        return yaml.safe_load(handle) or {}


def save_places(path, places):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as handle:
        yaml.safe_dump(places, handle, sort_keys=True)


class Places(Node):
    def __init__(self, path, tf_timeout=15.0):
        super().__init__('places')
        self.path = path
        self.tf_timeout = tf_timeout
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def current_pose(self, timeout=None):
        """map -> base_footprint, waiting for the localiser to publish it.

        The deadline is wall time on purpose. Under use_sim_time the ROS clock
        reads 0 until the first /clock message arrives, so a ROS-clock deadline
        of now + 5s is already in the past the instant simulation time is
        adopted, and this returns before any transform has been received.
        """
        deadline = time.monotonic() + (self.tf_timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                q = tf.transform.rotation
                return {
                    'x': float(tf.transform.translation.x),
                    'y': float(tf.transform.translation.y),
                    'yaw': float(quaternion_to_yaw(q.x, q.y, q.z, q.w)),
                }
        raise SystemExit(
            f'no map -> base_footprint transform after {self.tf_timeout:.0f}s. '
            'Is slam_toolbox or AMCL running, and has AMCL been given an initial '
            'pose? Raise the wait with --timeout if the stack is just slow to start.')

    def cmd_save(self, name):
        places = load_places(self.path)
        places[name] = self.current_pose()
        save_places(self.path, places)
        pose = places[name]
        print(f"saved '{name}' at x={pose['x']:.2f} y={pose['y']:.2f} yaw={pose['yaw']:.2f}")

    def cmd_list(self, _name=None):
        places = load_places(self.path)
        if not places:
            print(f'no places yet in {self.path}')
            return
        for name, pose in sorted(places.items()):
            print(f"{name:<16} x={pose['x']:7.2f}  y={pose['y']:7.2f}  yaw={pose['yaw']:6.2f}")

    def cmd_remove(self, name):
        places = load_places(self.path)
        if places.pop(name, None) is None:
            raise SystemExit(f"unknown place '{name}'")
        save_places(self.path, places)
        print(f"removed '{name}'")

    def cmd_go(self, name):
        places = load_places(self.path)
        if name not in places:
            raise SystemExit(f"unknown place '{name}'; known: {', '.join(sorted(places)) or 'none'}")
        pose = places[name]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = yaw_to_quaternion(pose['yaw'])

        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise SystemExit('navigate_to_pose action server not available; is Nav2 running?')

        print(f"going to '{name}'...")
        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            raise SystemExit('goal rejected by Nav2')

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)

        # Ctrl-C or a SIGTERM shuts the context down and spin returns with the
        # future unresolved. Say so rather than dying on a None attribute.
        outcome = result.result()
        if outcome is None:
            raise SystemExit(f"interrupted before '{name}' was reached; the goal may still be active")
        if outcome.status == GoalStatus.STATUS_SUCCEEDED:
            print(f"arrived at '{name}'")
        else:
            raise SystemExit(f"failed to reach '{name}' (status {outcome.status})")


def main(argv=None):
    argv = remove_ros_args(args=argv if argv is not None else sys.argv)
    parser = argparse.ArgumentParser(prog='places', description=__doc__.splitlines()[0])
    parser.add_argument('command', choices=['save', 'list', 'go', 'remove'])
    parser.add_argument('name', nargs='?')
    parser.add_argument('--file', default=DEFAULT_PLACES_FILE)
    parser.add_argument('--timeout', type=float, default=15.0,
                        help='seconds to wait for the map transform (default: 15)')
    args = parser.parse_args(argv[1:])

    if args.command != 'list' and not args.name:
        parser.error(f'{args.command} needs a place name')

    rclpy.init()
    node = Places(args.file, args.timeout)
    try:
        getattr(node, f'cmd_{args.command}')(args.name)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
