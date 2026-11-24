"""Callback integration without creating ROS nodes, publishers or action goals."""
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import DriveOnHeading
from nav2_msgs.msg import CollisionMonitorState
from rclpy.action import GoalResponse
from robo_recovery import supervised_node as mod
from robo_recovery.supervised import Session, Motion


def node():
    n = object.__new__(mod.SupervisedRecovery)
    n.lock = threading.RLock()
    n.session = Session()
    n.reserved = False
    n.normal_anchor = None
    n.normal_anchor_error = None
    n.last_navigation_report = 0.0
    n.normal_motion_since = None
    n.normal_route_anchor = None
    n.normal_route_goal = None
    n.normal_route_rebases = 0
    n.ctrl_nonzero = n.ctrl_received = 0.0
    n.scan = n.pose = n.final = None
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    route = Path()
    route.header.frame_id = 'map'
    a, b = PoseStamped(), PoseStamped()
    a.pose.orientation.w = b.pose.orientation.w = 1.0
    b.pose.position.x = 1.0
    route.poses = [a, b]
    n.path = (time.monotonic(), route)
    n.cm = (time.monotonic(), False)
    n.command = Mock()
    n.emit = Mock()
    n.out = Mock()
    n.fresh = lambda *args: True
    values = {'supervise_navigation_progress': True, 'footprint': [-0.205, 0.365, 0.315], 'allow_turn': False, 'independent_frame': 'map'}
    n.p = lambda k: values[k]
    return n


def handle():
    request = DriveOnHeading.Goal()
    request.target.x = 0.10
    request.speed = 0.03
    request.time_allowance.sec = 20
    return NS(request=request, is_cancel_requested=False,
              succeed=Mock(), abort=Mock(), canceled=Mock())


def test_rear_new_observation_before_second_action(monkeypatch):
    n, h = node(), handle()
    snapshots = [(Mock(), (0, 0, 0), 1, 1), (Mock(), (-0.1, 0, 0), 2, 2),
                 (Mock(), (-0.1, 0.1, 0), 3, 3)]
    trace = []
    n.stopped_snapshot = lambda *args: (trace.append('stop+new_evidence') or snapshots.pop(0))
    n.move = lambda *args: trace.append(args[2].name)
    n.heading = lambda p: 0.0
    choices = iter([Motion('rear', x=-0.1), Motion('left', y=0.1)])
    monkeypatch.setattr(mod, 'choose', lambda *a, **k: next(choices))
    n.execute(h)
    assert trace == ['stop+new_evidence', 'rear', 'stop+new_evidence', 'left', 'stop+new_evidence']
    h.succeed.assert_called_once()


def test_rear_no_exit_aborts_and_latches(monkeypatch):
    n, h = node(), handle()
    n.stopped_snapshot = lambda *args: (Mock(), (0, 0, 0), 1, 1)
    n.move = Mock()
    n.heading = lambda p: 0
    choices = iter([Motion('rear', x=-0.1), None])
    monkeypatch.setattr(mod, 'choose', lambda *a, **k: next(choices))
    n.execute(h)
    assert n.move.call_count == 1
    assert n.session.reason == 'rear_completed_no_safe_exit'
    h.abort.assert_called_once()
    assert n.goal(handle().request) == GoalResponse.REJECT


@pytest.mark.parametrize('reason', ['cancelled', 'communication', 'timeout', 'no_independent_progress'])
def test_interrupted_rear_never_starts_second_action(monkeypatch, reason):
    n, h = node(), handle()
    n.stopped_snapshot = lambda *args: (Mock(), (0, 0, 0), 1, 1)
    n.heading = lambda p: 0
    monkeypatch.setattr(mod, 'choose', lambda *a, **k: Motion('rear', x=-0.1))
    def fail(*args):
        if reason == 'cancelled':
            h.is_cancel_requested = True
        raise RuntimeError(reason)
    n.move = Mock(side_effect=fail)
    n.execute(h)
    assert n.move.call_count == 1 and n.session.state == 'FAILED'
    assert not n.session.begin()
    (h.canceled if reason == 'cancelled' else h.abort).assert_called_once()
    assert n.command.call_args.args == ()


def test_cm_stop_immediately_latches_and_stops():
    n = node()
    n.session.begin()
    msg = CollisionMonitorState()
    msg.action_type = 1
    n.on_cm(msg)
    assert n.session.reason == 'collision_monitor_stop'
    n.command.assert_called()


def test_controller_conflict_not_forwarded():
    n = node()
    n.session.begin()
    msg = TwistStamped()
    msg.twist.linear.x = 0.1
    n.on_controller(msg)
    assert n.session.reason == 'controller_conflict'
    n.out.publish.assert_not_called()


def test_positive_odom_cannot_bypass_missing_independent_pose():
    n = node()
    n.odom = (time.monotonic(), 100, 100, 0)  # deliberately ignored
    msg = TwistStamped()
    msg.twist.linear.x = 0.1
    n.on_controller(msg)
    assert n.session.reason == 'normal_navigation_missing_independent_pose'
    n.out.publish.assert_not_called()


def test_reservation_and_failure_require_new_authorization():
    n = node()
    assert n.goal(handle().request) == GoalResponse.ACCEPT
    assert n.goal(handle().request) == GoalResponse.REJECT
    n.fault('communication')
    response = NS(success=None, message='')
    assert not n.authorize(None, response).success
    n.reserved = False
    assert n.authorize(None, response).success


def test_stationary_independent_pose_overrides_positive_odom():
    n = node()
    now = time.monotonic()
    n.pose = (now, (0, 0, 0), 1)
    n.normal_anchor = (now - 7, (0, 0, 0))
    n.odom = (now, 10, 10, 0)
    msg = TwistStamped()
    msg.twist.linear.x = 0.1
    n.on_controller(msg)
    assert n.session.reason == 'normal_navigation_no_independent_progress'
    n.out.publish.assert_not_called()


def test_snapshot_requires_observation_after_stop():
    n, h = node(), handle()
    n.session.begin()
    now = time.monotonic()
    n.scan = (now, NS(header=NS(stamp=None)))
    n.pose = (now, (0, 0, 0), 5)
    n.cm = (now, False)
    n.final = (now, NS())
    assert n.snapshot(h, now + 2, after_scan=now + 0.01, after_pose=5) is None


def test_normal_navigation_does_not_repeat_cm_blocked_command():
    n = node()
    n.pose = (time.monotonic(), (0, 0, 0), 1)
    n.cm = (time.monotonic(), True)
    msg = TwistStamped()
    msg.twist.linear.x = 0.1
    n.on_controller(msg)
    n.on_controller(msg)
    assert n.session.reason == 'collision_monitor_stop'
    n.out.publish.assert_not_called()
    n.command.assert_called()


def test_cm_stop_during_normal_navigation_immediately_stops():
    n = node()
    n.ctrl_nonzero = time.monotonic()
    msg = CollisionMonitorState()
    msg.action_type = 1
    n.on_cm(msg)
    assert n.session.state == 'FAILED'
    n.command.assert_called()


def test_stable_cm_state_is_event_not_heartbeat():
    n = node()
    n.pose = (time.monotonic(), (0, 0, 0), 1)
    n.cm = (time.monotonic() - 1, False)
    msg = TwistStamped()
    msg.twist.linear.x = 0.1
    n.on_controller(msg)
    assert n.session.state != 'FAILED'
    n.out.publish.assert_called_once()


def test_first_fault_reason_is_preserved():
    n = node()
    n.fault('collision_monitor_stop')
    n.fault('controller_conflict')
    assert n.session.reason == 'collision_monitor_stop'


def test_final_output_loss_is_not_confused_with_silent_cm_state():
    n = node()
    now = time.monotonic()
    n.normal_motion_since = now - 1
    n.final = None
    assert not n.check_normal_feedback(now)
    assert n.session.reason == 'normal_navigation_missing_final_output'
    n.command.assert_called()


def test_yaw_oscillation_without_useful_turn_is_not_progress():
    n = node()
    now = time.monotonic()
    n.normal_anchor = (now - 7, (0, 0, 0))
    n.normal_anchor_error = 0.5
    n.orientation_error = lambda _: 0.8
    n.pose = (now, (0, 0, 0.3), 1)
    msg = TwistStamped()
    msg.twist.angular.z = 0.1
    n.on_controller(msg)
    assert n.session.reason == 'normal_navigation_no_independent_progress'
    n.out.publish.assert_not_called()


def test_useful_turn_can_renew_progress():
    n = node()
    now = time.monotonic()
    n.normal_anchor = (now - 7, (0, 0, 0))
    n.normal_anchor_error = 1.4
    n.orientation_error = lambda _: 1.0
    n.pose = (now, (0, 0, 0.3), 1)
    msg = TwistStamped()
    msg.twist.angular.z = 0.1
    n.on_controller(msg)
    assert n.session.state != 'FAILED'
    n.out.publish.assert_called_once()


def test_progress_uses_required_yaw_in_final_position_window():
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    import math
    n = node()
    path = Path()
    path.header.frame_id = 'map'
    goal = PoseStamped()
    goal.pose.position.x = 0.02
    goal.pose.orientation.z = math.sin(math.pi / 4)
    goal.pose.orientation.w = math.cos(math.pi / 4)
    path.poses = [goal]
    n.path = (time.monotonic(), path)
    assert n.orientation_error((0, 0, 0.5)) == pytest.approx(math.pi / 2 - 0.5)


def test_independent_back_and_forth_without_route_gain_stops():
    n = node()
    now = time.monotonic()
    n.normal_route_goal = (1, 0, 0)
    n.normal_route_anchor = (now - 7, 0.90)
    # Physically moved, but regressed from x=.10 to x=.04.
    assert not n.check_route_progress(now, (0.04, 0, 0), True, False)
    assert n.session.reason == 'normal_navigation_no_path_progress'


def test_replanning_cannot_rebase_progress_indefinitely():
    n = node()
    now = time.monotonic()
    n.normal_route_goal = (1, 0, 0)
    n.normal_route_anchor = (now, 0.1)
    assert n.check_route_progress(now, (0, 0, 0), False, False)
    assert n.normal_route_rebases == 1
    n.normal_route_anchor = (now - 7, 0.1)
    assert not n.check_route_progress(now, (0, 0, 0), False, False)
    assert n.session.reason == 'normal_navigation_no_path_progress'


def test_changed_path_goal_does_not_clear_latched_failure():
    n = node()
    n.fault('cancelled')
    import copy
    path = copy.deepcopy(n.path[1])
    path.poses[-1].pose.position.x = 2.0
    n.on_path(path)
    assert n.session.state == 'FAILED' and n.session.reason == 'cancelled'


def test_exhausted_recovery_budget_latches_explicit_failure():
    n = node()
    n.session.attempts = 2
    assert n.goal(handle().request) == GoalResponse.REJECT
    assert n.session.state == 'FAILED' and n.session.reason == 'recovery_budget_exhausted'
    n.command.assert_called()


def test_preserved_valid_path_is_not_a_two_second_heartbeat():
    n = node()
    n.path = (time.monotonic() - 12.0, n.path[1])
    # Valid geometry may be retained by the BT for 15 seconds. Fresh independent
    # pose/scan are still required separately; old path publication is not a fault.
    assert n.heading((0.0, 0.0, 0.0)) is not None
    assert n.orientation_error((0.0, 0.0, 0.0)) is not None
    assert n.check_route_progress(time.monotonic(), (0.0, 0.0, 0.0), False, False)
    assert n.session.state != 'FAILED'


def test_normal_nav2_command_does_not_require_recovery_certification():
    n = node()
    original = n.p
    n.p = lambda k: False if k == 'supervise_navigation_progress' else original(k)
    msg = TwistStamped()
    msg.twist.linear.y = 0.02
    n.on_controller(msg)
    n.out.publish.assert_called_once_with(msg)
    assert n.pose is None
    assert n.session.state != 'FAILED'


def test_optional_progress_supervision_cannot_bypass_cm():
    n = node()
    original = n.p
    n.p = lambda k: False if k == 'supervise_navigation_progress' else original(k)
    n.cm = (time.monotonic(), True)
    msg = TwistStamped()
    msg.twist.linear.y = 0.02
    n.on_controller(msg)
    n.out.publish.assert_not_called()
    assert n.session.reason == 'collision_monitor_stop'
