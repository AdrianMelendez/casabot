"""Map a place with no one driving: go to the nearest frontier until none are left.

Runs next to slam_toolbox and Nav2 (launch/explore.launch.py). Nav2 does the
path planning and the obstacle avoidance; this node only decides where to go.
When nothing unknown is reachable it saves the map to map_path, the file
navigation.launch.py loads by default, and drives back to where it started.
"""

import math
import os

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from casabot.frontiers import UNKNOWN, find_frontiers, save_map
from casabot.kinematics import yaw_to_quaternion
from casabot.places import DEFAULT_PLACES_FILE, load_places, save_places
from casabot.rooms import segment_rooms


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.declare_parameter('map_path', os.path.expanduser('~/.casabot/map'))
        self.declare_parameter('min_frontier_length', 0.4)   # m; smaller gaps are sensor noise
        self.declare_parameter('goal_timeout', 120.0)        # s of ROS time before giving up on a goal
        self.declare_parameter('explored_radius', 0.4)       # m; goal counts as explored once no unknown is this close
        # m from any obstacle. Frontiers hug furniture; a goal wedged between a
        # chair and a plant puts the robot inside Nav2's inflation zone, where
        # neither the controller nor the recoveries will move it again.
        self.declare_parameter('goal_clearance', 0.35)
        self.declare_parameter('return_home', True)
        self.declare_parameter('places_path', DEFAULT_PLACES_FILE)

        self.map_path = self.get_parameter('map_path').value
        self.min_length = self.get_parameter('min_frontier_length').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.explored_radius = self.get_parameter('explored_radius').value
        self.return_home = self.get_parameter('return_home').value
        self.goal_clearance = self.get_parameter('goal_clearance').value
        self.places_path = self.get_parameter('places_path').value

        self.map = None
        self.home = None
        self.goal = None           # (x, y) currently being driven to
        self.goal_handle = None
        self.goal_started = None
        self.cancelled = False     # we cancelled on purpose; not a failure
        self.blacklist = []
        self.idle_ticks = 0
        self.rejections = 0
        self.frontier_xy = []      # candidate goals from the last search, for RViz
        self.rooms_xy = []         # (name, x, y) once the rooms are found
        self.state = 'exploring'   # -> returning -> done
        self.goals_sent = 0
        self.stuck_at = None       # robot position when the last goal failed
        self.stuck_goals = []      # goals that failed from there
        self.recoveries = 0

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_costmaps = [self.create_client(ClearEntireCostmap, name) for name in (
            'local_costmap/clear_entirely_local_costmap',
            'global_costmap/clear_entirely_global_costmap')]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
        self.markers = self.create_publisher(MarkerArray, 'explore/markers', latched)
        self.create_timer(1.0, self.publish_markers)
        self.create_timer(1.0, self.tick)
        self.get_logger().info('waiting for /map and Nav2')

    def on_map(self, msg):
        self.map = msg

    # -- geometry helpers --------------------------------------------------

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def grid(self):
        m = self.map
        return np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)

    def cell_to_xy(self, row, col):
        i = self.map.info
        return (i.origin.position.x + (col + 0.5) * i.resolution,
                i.origin.position.y + (row + 0.5) * i.resolution)

    def around(self, grid, x, y, radius):
        """Cells within radius of (x, y)."""
        i = self.map.info
        col = int((x - i.origin.position.x) / i.resolution)
        row = int((y - i.origin.position.y) / i.resolution)
        r = int(radius / i.resolution)
        r0, c0 = max(row - r, 0), max(col - r, 0)
        window = grid[r0:row + r + 1, c0:col + r + 1]
        rr, cc = np.ogrid[r0:r0 + window.shape[0], c0:c0 + window.shape[1]]
        return window[(rr - row) ** 2 + (cc - col) ** 2 <= r * r]

    def unknown_near(self, grid, x, y, radius):
        return bool((self.around(grid, x, y, radius) == UNKNOWN).any())

    def crowded(self, grid, x, y):
        # Strictly closer than goal_clearance, or a goal find_frontiers accepts
        # could be judged crowded and re-picked forever.
        return bool((self.around(grid, x, y, self.goal_clearance - 1e-3) >= 50).any())

    def blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < 0.5 for bx, by in self.blacklist)

    # -- main loop ---------------------------------------------------------

    def tick(self):
        if self.state != 'exploring' or self.map is None:
            return
        robot = self.robot_xy()
        if robot is None or not self.nav.server_is_ready():
            return
        if self.home is None:
            self.home = robot
            self.get_logger().info('exploring from (%.2f, %.2f)' % robot)

        grid = self.grid()

        if self.goal is not None:
            if self.cancelled:
                return                        # cancel in flight; on_result clears it
            elapsed = (self.get_clock().now() - self.goal_started).nanoseconds * 1e-9
            if not self.unknown_near(grid, *self.goal, self.explored_radius):
                self.cancel_goal()            # seen it already; no need to arrive
            elif self.crowded(grid, *self.goal):
                # Furniture mapped since the goal was picked now hems it in.
                # Re-pick; the same frontier gets a roomier cell next tick.
                self.cancel_goal()
            elif elapsed > self.goal_timeout:
                self.goal_failed('timed out on')
                self.cancel_goal()
            return

        res = self.map.info.resolution
        min_cells = max(1, int(self.min_length / res))
        targets = []
        for row, col, size in find_frontiers(grid, min_cells, self.goal_clearance / res):
            x, y = self.cell_to_xy(row, col)
            if not self.blacklisted(x, y):
                targets.append((math.hypot(x - robot[0], y - robot[1]), x, y, size))

        self.frontier_xy = [(x, y) for _, x, y, _ in targets]
        if not targets:
            # The map only updates about once a second; make sure it is really done.
            self.idle_ticks += 1
            if self.idle_ticks >= 5:
                self.finish(grid)
            return
        self.idle_ticks = 0

        _, x, y, size = min(targets)
        self.send_goal(x, y, robot)
        self.get_logger().info('goal %d: frontier of %d cells at (%.2f, %.2f), %d left' % (
            self.goals_sent, size, x, y, len(targets)))

    # -- Nav2 --------------------------------------------------------------

    def send_goal(self, x, y, robot):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'   # zero stamp: "latest", whatever the clock
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = math.atan2(y - robot[1], x - robot[0])
        q = goal.pose.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quaternion(yaw)
        self.goal = (x, y)
        self.goal_started = self.get_clock().now()
        self.cancelled = False
        self.goals_sent += 1
        self.nav.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            # Nav2 rejects everything until its navigator is active, so a
            # rejection right after launch says nothing about the goal. The
            # first version blacklisted here and lost two whole rooms that way.
            self.rejections += 1
            if self.rejections >= 5:
                self.get_logger().warn('Nav2 keeps rejecting (%.2f, %.2f); skipping it' % self.goal)
                self.blacklist.append(self.goal)
                self.rejections = 0
            self.goal = None
            return
        self.rejections = 0
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)
        if self.cancelled:                    # dropped while Nav2 was still accepting it
            handle.cancel_goal_async()

    def cancel_goal(self):
        self.cancelled = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def on_result(self, future):
        status = future.result().status
        if self.state == 'returning':
            self.state = 'done'
            self.save()
            self.get_logger().info('back home; exploration finished')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Reached, yet it was not cancelled as explored on the way, so the
            # unknown cells next to it cannot be seen even from here (a corner,
            # a gap behind furniture). Without this the same spot is picked
            # again every second, forever.
            self.blacklist.append(self.goal)
        elif not self.cancelled:
            self.goal_failed('could not reach')
        self.goal = None
        self.goal_handle = None

    def goal_failed(self, why):
        """Blacklist the goal, unless the robot, not the goal, is the problem.

        Two failures in a row from the same spot mean the robot is stuck, not
        that both goals are unreachable. Under CPU load the controller can see
        obstacles in open floor; without this, one stuck minute blacklisted
        every frontier it tried and a third of the flat went unmapped.
        """
        self.get_logger().warn('%s (%.2f, %.2f); skipping it' % ((why,) + self.goal))
        self.blacklist.append(self.goal)
        here = self.robot_xy()
        if here is None:
            return
        if self.stuck_at is None or math.dist(here, self.stuck_at) > 0.3:
            self.stuck_at, self.stuck_goals, self.recoveries = here, [self.goal], 0
            return
        self.stuck_goals.append(self.goal)
        if self.recoveries >= 3:
            return                            # genuinely boxed in; let it give up
        self.recoveries += 1
        self.blacklist = [g for g in self.blacklist if g not in self.stuck_goals]
        self.get_logger().warn(
            'robot stuck at (%.2f, %.2f) after %d failed goals: clearing costmaps and '
            'retrying them (recovery %d of 3)' % (here + (len(self.stuck_goals), self.recoveries)))
        self.stuck_goals = []
        for client in self.clear_costmaps:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    # -- the end -----------------------------------------------------------

    def save(self):
        i = self.map.info
        save_map(self.grid(), i.resolution, (i.origin.position.x, i.origin.position.y), self.map_path)
        free = int((self.grid() == 0).sum()) * i.resolution ** 2
        self.get_logger().info('saved map to %s.yaml (%.1f m2 of free floor)' % (self.map_path, free))

    def name_rooms(self):
        """Save the middle of every room as a place: room_1 is the biggest."""
        labels, rooms = segment_rooms(self.grid(), self.map.info.resolution)
        places = load_places(self.places_path)
        mine = [name for name in places if not name.startswith('room_')]
        if mine:
            self.get_logger().warn(
                'places %s were saved against an earlier map; re-save them if they '
                'are off' % ', '.join(sorted(mine)))
        places = {k: v for k, v in places.items() if not k.startswith('room_')}
        for k, (row, col, area) in enumerate(rooms, start=1):
            x, y = self.cell_to_xy(row, col)
            places[f'room_{k}'] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': 0.0}
            self.rooms_xy.append((f'room_{k}', x, y))
            self.get_logger().info('room_%d: %.1f m2, place at (%.2f, %.2f)' % (k, area, x, y))
        save_places(self.places_path, places)

    def publish_markers(self):
        """What the explorer is thinking, for RViz (rviz/casabot.rviz shows it):
        cyan frontiers it could go to, the green goal it is going to, red goals
        it gave up on, and at the end each room with its name."""
        def marker(ns, mid, kind, size, rgba, points=None):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type = ns, mid, kind
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            if points is not None:
                m.points = [Point(x=x, y=y, z=0.05) for x, y in points]
                m.action = Marker.ADD if points else Marker.DELETE
            return m

        out = MarkerArray()
        out.markers.append(marker('frontiers', 0, Marker.SPHERE_LIST, 0.15, (0.1, 0.8, 0.9, 0.9),
                                  self.frontier_xy if self.state == 'exploring' else []))
        out.markers.append(marker('skipped', 0, Marker.CUBE_LIST, 0.15, (0.9, 0.2, 0.2, 0.9),
                                  self.blacklist))
        goal = marker('goal', 0, Marker.SPHERE, 0.35, (0.2, 0.85, 0.3, 0.8))
        if self.goal is None or self.state != 'exploring':
            goal.action = Marker.DELETE
        else:
            goal.pose.position.x, goal.pose.position.y = self.goal
        out.markers.append(goal)
        for k, (name, x, y) in enumerate(self.rooms_xy):
            dot = marker('rooms', k, Marker.SPHERE, 0.3, (0.92, 0.35, 0.05, 1.0))
            dot.pose.position.x, dot.pose.position.y = x, y
            label = marker('room_names', k, Marker.TEXT_VIEW_FACING, 0.5, (0.05, 0.05, 0.1, 1.0))
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y + 0.45, 0.5
            label.text = name
            out.markers += [dot, label]
        self.markers.publish(out)

    def finish(self, grid):
        self.get_logger().info('no reachable frontiers left after %d goals' % self.goals_sent)
        self.save()
        self.name_rooms()
        if self.return_home and self.home is not None:
            self.state = 'returning'
            self.send_goal(*self.home, self.robot_xy() or self.home)
        else:
            self.state = 'done'


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
