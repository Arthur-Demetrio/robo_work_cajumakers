"""Candidate extension of mecanum recovery, with integrated command ownership.

All output is upstream of velocity_smoother and Collision Monitor. This executable
is only wired by the offline candidate launch. No legacy server is changed.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import DriveOnHeading
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .supervised import Scan, Motion, Session, choose, swept_clear, measured_progress, wrap, remaining_path_length


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SupervisedRecovery(Node):
    def __init__(self):
        super().__init__('supervised_recovery')
        defaults = {
            'coverage_certified': False, 'independent_pose_certified': False,
            'supervise_navigation_progress': False,
            'shadow_intervals': [-3.141592653589793, 3.141592653589793],
            'footprint': [-0.205, 0.365, 0.315], 'independent_frame': 'map',
            'max_pose_variance': 0.0004, 'allow_turn': False,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.lock = threading.RLock()
        self.session = Session()
        self.scan = self.pose = self.path = self.cm = self.final = None
        self.cm_polygon = ''
        self.ctrl_received = 0.0
        self.ctrl_nonzero = 0.0
        self.reserved = False
        self.normal_anchor = None
        self.normal_anchor_error = None
        self.last_navigation_report = 0.0
        self.normal_motion_since = None
        self.normal_route_anchor = None
        self.normal_route_goal = None
        self.normal_route_rebases = 0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.group = ReentrantCallbackGroup()
        self.out = self.create_publisher(TwistStamped, 'cmd_vel_nav', 1)
        self.status = self.create_publisher(String, 'recovery/state', 10)
        self.create_subscription(LaserScan, 'scan_front', self.on_scan,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(PoseWithCovarianceStamped, 'recovery/observed_pose',
                                 self.on_pose, 5, callback_group=self.group)
        self.create_subscription(Path, 'plan', self.on_path, 5, callback_group=self.group)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self.on_cm, 5, callback_group=self.group)
        self.create_subscription(TwistStamped, 'cmd_vel', self.on_final, 5,
                                 callback_group=self.group)
        self.create_subscription(TwistStamped, 'cmd_vel_ctrl', self.on_controller, 1,
                                 callback_group=self.group)
        self.create_service(Trigger, 'recovery/authorize', self.authorize,
                            callback_group=self.group)
        self.server = ActionServer(self, DriveOnHeading, 'supervised_escape',
                                   self.execute, goal_callback=self.goal,
                                   cancel_callback=self.cancel, callback_group=self.group)
        self.create_timer(0.05, self.watchdog, callback_group=self.group)

    def p(self, key):
        return self.get_parameter(key).value

    def emit(self, state, reason=''):
        self.status.publish(String(data=json.dumps({'state': state, 'reason': reason,
                              'attempts': self.session.attempts, 'collision_polygon': self.cm_polygon,
                              'owner': 'BLOCKED' if self.session.state == 'FAILED' else
                                       'RECOVERY' if self.session.active or self.reserved else 'NAV2'})))

    def command(self, vx=0.0, vy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.out.publish(msg)

    def fault(self, reason):
        if self.session.state == 'FAILED':
            reason = self.session.reason  # preserve the initiating cause
        self.session.fail(reason)
        self.command()
        self.emit('FAILED', reason)

    def fresh(self, stamp, limit=0.5):
        return 0 <= (self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds) / 1e9 <= limit

    def on_scan(self, msg):
        with self.lock:
            self.scan = (time.monotonic(), msg)

    def on_path(self, msg):
        with self.lock:
            if self.path and self.path[1].poses and msg.poses:
                previous, current = self.path[1].poses[-1].pose, msg.poses[-1].pose
                changed = (math.hypot(previous.position.x-current.position.x,
                                      previous.position.y-current.position.y) > 0.06 or
                           abs(wrap(yaw(previous.orientation)-yaw(current.orientation))) > 0.17)
                if changed and self.session.state != 'FAILED' and not self.session.active:
                    self.reset_normal_progress()
            self.path = (time.monotonic(), msg)

    def reset_normal_progress(self):
        self.normal_anchor = self.normal_anchor_error = self.normal_route_anchor = None
        self.normal_route_goal = self.normal_motion_since = None
        self.normal_route_rebases = 0

    def on_pose(self, msg):
        with self.lock:
            if not self.p('independent_pose_certified') or msg.header.frame_id != self.p('independent_frame'):
                self.pose = None
                return
            cov = msg.pose.covariance
            values = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation))
            if (not all(math.isfinite(v) for v in values) or
                any(not math.isfinite(cov[i]) or cov[i] < 0 or cov[i] > self.p('max_pose_variance')
                    for i in (0, 7, 35)) or not self.fresh(msg.header.stamp)):
                self.pose = None
                return
            # Repeated/old pose stamps do not refresh evidence.
            stamp = Time.from_msg(msg.header.stamp).nanoseconds
            if self.pose and stamp <= self.pose[2]:
                return
            self.pose = (time.monotonic(), values, stamp)

    def on_cm(self, msg):
        with self.lock:
            self.cm = (time.monotonic(), msg.action_type == 1)
            self.cm_polygon = msg.polygon_name
            if self.cm[1] and (self.session.active or time.monotonic() - self.ctrl_nonzero < 0.3):
                self.fault('collision_monitor_stop')

    def on_final(self, msg):
        with self.lock:
            if self.fresh(msg.header.stamp):
                self.final = (time.monotonic(), msg.twist)

    def on_controller(self, msg):
        with self.lock:
            now = time.monotonic()
            self.ctrl_received = now
            nonzero = any(abs(v) > 0.001 for v in
                          (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z))
            if nonzero:
                self.ctrl_nonzero = now
            if self.session.active or self.reserved:
                if nonzero:
                    self.fault('controller_conflict')
                return
            if self.session.state != 'FAILED' and self.fresh(msg.header.stamp, 0.3):
                if nonzero:
                    # Existing Nav2 progress checker remains in force. Independent
                    # progress supervision is opt-in; it is mandatory for recovery.
                    if self.cm is not None and self.cm[1]:
                        self.fault('collision_monitor_stop')
                        return
                    if self.normal_motion_since is None:
                        self.normal_motion_since = now
                    if not self.check_normal_feedback(now):
                        return
                    if (self.p('supervise_navigation_progress') and
                            not self.check_independent_navigation_progress(now)):
                        return
                    if now - self.last_navigation_report >= 0.5:
                        self.emit('NAVIGATING', 'Nav2 owns navigation; final output monitored')
                        self.last_navigation_report = now
                else:
                    self.normal_motion_since = None
                self.out.publish(msg)

    def check_independent_navigation_progress(self, now):
        if not self.pose or now - self.pose[0] > 0.5:
            self.fault('normal_navigation_missing_independent_pose')
            return False
        error = self.orientation_error(self.pose[1])
        if self.normal_anchor is None:
            self.normal_anchor = (now, self.pose[1])
            self.normal_anchor_error = error
        anchor_time, anchor = self.normal_anchor
        current = self.pose[1]
        translated = math.hypot(current[0] - anchor[0], current[1] - anchor[1]) >= 0.03
        useful_turn = (abs(wrap(current[2] - anchor[2])) >= 0.17 and
                       error is not None and self.normal_anchor_error is not None and
                       self.normal_anchor_error - error >= 0.17)
        if translated or useful_turn:
            self.normal_anchor = (now, current)
            self.normal_anchor_error = error
        elif now - anchor_time > 6.0:
            self.fault('normal_navigation_no_independent_progress')
            return False
        return self.check_route_progress(now, current, translated, useful_turn)

    def check_route_progress(self, now, pose, translated, useful_turn):
        if not self.path:
            self.fault('normal_navigation_missing_path')
            return False
        path = self.path[1]
        if path.header.frame_id != self.p('independent_frame') or not path.poses:
            self.fault('normal_navigation_invalid_path_frame')
            return False
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        goal = (*points[-1], yaw(path.poses[-1].pose.orientation))
        remaining = remaining_path_length(points, pose[:2])
        new_goal = (self.normal_route_goal is None or
                    math.dist(goal[:2], self.normal_route_goal[:2]) > 0.06 or
                    abs(wrap(goal[2] - self.normal_route_goal[2])) > 0.17)
        if new_goal or self.normal_route_anchor is None:
            self.normal_route_goal = goal
            self.normal_route_anchor = (now, remaining)
            self.normal_route_rebases = 0
            return True
        anchor_time, anchor_remaining = self.normal_route_anchor
        if useful_turn or (translated and anchor_remaining - remaining >= 0.03):
            self.normal_route_anchor = (now, remaining)
        elif remaining - anchor_remaining > 0.30 and self.normal_route_rebases < 1:
            # One longer reroute may replace the old length baseline. Replans must
            # not reset the watchdog indefinitely while the robot oscillates.
            self.normal_route_anchor = (now, remaining)
            self.normal_route_rebases += 1
        elif now - anchor_time > 6.0:
            self.fault('normal_navigation_no_path_progress')
            return False
        return True

    def check_normal_feedback(self, now):
        if (self.normal_motion_since is not None and now - self.normal_motion_since > 0.5 and
            (self.final is None or now - self.final[0] > 0.5)):
            self.fault('normal_navigation_missing_final_output')
            return False
        return True

    def watchdog(self):
        with self.lock:
            # A foreign publisher cannot be safely multiplexed by topic arrival order.
            for topic, expected in [('cmd_vel_nav', self.get_name()),
                                    ('cmd_vel_ctrl', 'controller_server'),
                                    ('cmd_vel_smoothed', 'velocity_smoother'),
                                    ('cmd_vel', 'collision_monitor')]:
                pubs = self.get_publishers_info_by_topic(self.resolve_topic_name(topic))
                if any(p.node_name != expected for p in pubs) or len(pubs) > 1:
                    self.fault('foreign_publisher:' + topic)
            if not self.session.active and time.monotonic() - self.ctrl_nonzero < 0.3:
                self.check_normal_feedback(time.monotonic())
            if self.session.state == 'FAILED' or (not self.session.active and
                                                 time.monotonic() - self.ctrl_received > 0.3):
                self.command()

    def authorize(self, request, response):
        with self.lock:
            response.success = not self.reserved and self.session.authorize()
            if response.success:
                self.normal_anchor = None
                self.normal_anchor_error = None
                self.normal_motion_since = None
                self.normal_route_anchor = None
                self.normal_route_goal = None
                self.normal_route_rebases = 0
            response.message = 'new recovery authorization' if response.success else 'action still active'
            self.command()
            self.emit(self.session.state, response.message)
        return response

    def goal(self, request):
        with self.lock:
            duration = request.time_allowance.sec + request.time_allowance.nanosec / 1e9
            if self.session.attempts >= 2:
                self.fault('recovery_budget_exhausted')
                return GoalResponse.REJECT
            if (self.reserved or self.session.state == 'FAILED' or self.session.attempts >= 2 or
                not math.isfinite(request.target.x) or not 0.03 <= request.target.x <= 0.10 or
                not math.isfinite(request.speed) or not 0 < request.speed <= 0.03 or
                not 0 < duration <= 20):
                self.emit('REJECTED', 'invalid_request_or_authorization_required')
                return GoalResponse.REJECT
            self.reserved = True
            self.emit('RECOVERY_REQUESTED', 'Nav2 requested bounded geometric recovery')
            self.command()
            return GoalResponse.ACCEPT

    def cancel(self, handle):
        with self.lock:
            self.fault('cancelled')
        return CancelResponse.ACCEPT

    def snapshot(self, handle, deadline, after_scan=0.0, after_pose=0):
        with self.lock:
            if handle.is_cancel_requested:
                self.fault('cancelled')
            if time.monotonic() >= deadline:
                self.fault('timeout')
            if self.session.state == 'FAILED':
                raise RuntimeError(self.session.reason)
            now = time.monotonic()
            if (not self.scan or now - self.scan[0] > 0.5 or
                not self.fresh(self.scan[1].header.stamp) or
                not self.pose or now - self.pose[0] > 0.5 or
                (self.cm is not None and self.cm[1]) or
                not self.final or now - self.final[0] > 0.5):
                raise RuntimeError('missing_or_stale_evidence')
            if self.scan[0] <= after_scan or self.pose[2] <= after_pose:
                return None
            msg = self.scan[1]
            # TF at observation timestamp; never use the old hard-coded laser fallback.
            tr = self.tf.lookup_transform('base_link', msg.header.frame_id,
                                          Time.from_msg(msg.header.stamp))
            intervals = self.p('shadow_intervals')
            if len(intervals) % 2:
                raise RuntimeError('invalid_shadow_mask')
            laser = (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
            scan = Scan(tuple(msg.ranges), msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, laser,
                        tuple(zip(intervals[::2], intervals[1::2])), self.p('coverage_certified'))
            return scan, self.pose[1], self.scan[0], self.pose[2]

    def heading(self, pose):
        with self.lock:
            if not self.path:
                return None
            path = self.path[1]
            if path.header.frame_id != self.p('independent_frame'):
                return None
            if not path.poses:
                return None
            points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
            nearest = min(range(len(points)), key=lambda i: math.hypot(points[i][0] - pose[0], points[i][1] - pose[1]))
            for x, y in points[nearest:]:
                dx, dy = x - pose[0], y - pose[1]
                if math.hypot(dx, dy) >= 0.10:
                    return wrap(math.atan2(dy, dx) - pose[2])
        return None

    def orientation_error(self, pose):
        """A turn counts only if it reduces a relevant orientation error.

        In the final position window, use the required GOAL yaw. Else use the path
        direction. Yaw oscillations alone must not renew the progress allowance.
        """
        with self.lock:
            if not self.path:
                return None
            path = self.path[1]
            if path.header.frame_id != self.p('independent_frame') or not path.poses:
                return None
            goal = path.poses[-1].pose
            if math.hypot(goal.position.x - pose[0], goal.position.y - pose[1]) <= 0.06:
                return abs(wrap(yaw(goal.orientation) - pose[2]))
        heading = self.heading(pose)
        return abs(heading) if heading is not None else None

    def stopped_snapshot(self, handle, deadline):
        self.command()
        self.emit('STOP')
        # Drain smoother and require evidence recorded AFTER the stop, plus zero final output.
        stop_time = time.monotonic()
        with self.lock:
            old_pose = self.pose[2] if self.pose else 0
        while time.monotonic() < stop_time + 0.4:
            self.snapshot(handle, deadline)
            self.command()
            time.sleep(0.025)
        stable = None
        while True:
            snap = self.snapshot(handle, deadline, stop_time + 0.3, old_pose)
            with self.lock:
                final = self.final[1]
                stopped = max(abs(final.linear.x), abs(final.linear.y), abs(final.angular.z)) < 0.001
            if snap is not None and stopped:
                if stable is None:
                    stable = (time.monotonic(), snap[1])
                else:
                    p0 = stable[1]
                    if (math.hypot(snap[1][0] - p0[0], snap[1][1] - p0[1]) > 0.005 or
                        abs(wrap(snap[1][2] - p0[2])) > 0.02):
                        stable = (time.monotonic(), snap[1])
                    elif time.monotonic() - stable[0] >= 0.2:
                        return snap
            else:
                stable = None
            self.command()
            time.sleep(0.025)

    def move(self, handle, deadline, motion, speed, initial):
        start = initial[1]
        target = abs(motion.yaw) if motion.yaw else math.hypot(motion.x, motion.y)
        last_progress, progress_time = 0.0, time.monotonic()
        self.emit('MOVING', motion.name)
        while True:
            scan, pose, _, _ = self.snapshot(handle, deadline)
            progress = measured_progress(start, pose, motion)
            if progress < -0.02 or progress > target + 0.04:
                raise RuntimeError('unexpected_motion')
            if progress >= target - 0.01:
                self.command()
                return
            if progress >= last_progress + 0.005:
                last_progress, progress_time = progress, time.monotonic()
            if time.monotonic() - progress_time > 1.5:
                raise RuntimeError('no_independent_progress')
            remaining = max(0, target - progress)
            f = remaining / target
            rest = Motion(motion.name, motion.x * f, motion.y * f, motion.yaw * f)
            if not swept_clear(scan, rest, tuple(self.p('footprint'))):
                raise RuntimeError('swept_volume_unknown_or_blocked')
            with self.lock:
                if self.session.state == 'FAILED':
                    raise RuntimeError(self.session.reason)
                if motion.yaw:
                    self.command(wz=math.copysign(min(speed, 0.03), motion.yaw))
                else:
                    self.command(speed * motion.x / target, speed * motion.y / target)
            time.sleep(0.05)

    def execute(self, handle):
        result = DriveOnHeading.Result()
        started = time.monotonic()
        deadline = started + handle.request.time_allowance.sec + handle.request.time_allowance.nanosec / 1e9
        try:
            with self.lock:
                if not self.session.begin():
                    raise RuntimeError(self.session.reason or 'authorization_required')
                if time.monotonic() - self.ctrl_nonzero < 0.3:
                    raise RuntimeError('controller_not_quiescent')
            snap = self.stopped_snapshot(handle, deadline)
            footprint = tuple(self.p('footprint'))
            self.emit('ASSESSING', 'check observed swept volumes and continuation')
            motion = choose(snap[0], footprint, self.heading(snap[1]), handle.request.target.x,
                            need_turn=self.p('allow_turn'))
            if motion is None:
                raise RuntimeError('no_observed_safe_alternative')
            self.move(handle, deadline, motion, handle.request.speed, snap)
            snap = self.stopped_snapshot(handle, deadline)
            if motion.name == 'rear':
                self.emit('REEVALUATE', 'new scan and independent pose after rear')
                motion = choose(snap[0], footprint, self.heading(snap[1]), handle.request.target.x,
                                backed=True, need_turn=self.p('allow_turn'))
                if motion is None:
                    raise RuntimeError('rear_completed_no_safe_exit')
                self.move(handle, deadline, motion, handle.request.speed, snap)
                self.stopped_snapshot(handle, deadline)
            with self.lock:
                if not self.session.finish():
                    raise RuntimeError(self.session.reason)
                self.reset_normal_progress()
                self.emit('SUCCEEDED', 'independent movement observed; replan required')
                handle.succeed()
        except Exception as exc:
            with self.lock:
                self.fault(str(exc))
            result.error_code = DriveOnHeading.Result.UNKNOWN
            if hasattr(result, 'error_msg'):
                result.error_msg = str(exc)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            with self.lock:
                self.command()
                self.reserved = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SupervisedRecovery()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.command()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
