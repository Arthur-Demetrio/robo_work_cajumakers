import math
from dataclasses import replace

import pytest
from robo_recovery.supervised import Scan, Motion, Session, choose, covered, swept_clear, measured_progress

FP = (-0.205, 0.365, 0.315)


def scan():
    return Scan((5.0,) * 1440, -math.pi, 2 * math.pi / 1440,
                0.02, 5.0, (0.0, 0.0, 0.0), certified=True)


@pytest.mark.parametrize('motion', [Motion('front', x=0.10), Motion('left', y=0.10),
                                   Motion('right', y=-0.10), Motion('curve', x=0.10, yaw=0.10)])
def test_open_swept_geometry(motion):
    assert swept_clear(scan(), motion, FP)


@pytest.mark.parametrize('heading,name', [(0.2, 'left'), (-0.2, 'right')])
def test_lateral_precedes_rear_and_turn(heading, name):
    assert choose(scan(), FP, heading, need_turn=True).name == name


def test_small_lateral_no_turn():
    choice = choose(scan(), FP, math.pi / 2, need_turn=True)
    assert choice.name == 'left' and choice.yaw == 0


def test_unknown_is_not_free():
    for ranges in [(math.inf,) * 1440, (math.nan,) * 1440, (0.0,) * 1440]:
        assert not swept_clear(replace(scan(), ranges=ranges), Motion('rear', x=-0.10), FP)
    assert not swept_clear(replace(scan(), certified=False), Motion('left', y=0.10), FP)


def test_single_rear_return_does_not_prove_corridor():
    values = [math.nan] * 1440
    values[0] = 2.0
    assert not swept_clear(replace(scan(), ranges=tuple(values)), Motion('rear', x=-0.10), FP)


def test_rear_shadow_blocks():
    assert not swept_clear(replace(scan(), shadows=((-math.pi, -2.0), (2.0, math.pi))),
                           Motion('rear', x=-0.10), FP)


def test_partial_fov_and_near_blind_zone():
    partial = replace(scan(), ranges=(5.0,) * 720, angle_min=-math.pi / 2)
    assert not covered(partial, -0.30, 0)
    assert not covered(replace(scan(), laser=(0.33, 0.005, 0), range_min=0.17), 0.40, 0.005)


def test_corner_single_point_vetoes_turn():
    values = list(scan().ranges)
    a, r = math.atan2(0.39, 0.35), math.hypot(0.39, 0.35)
    values[round((a + math.pi) / scan().increment)] = r
    assert not swept_clear(replace(scan(), ranges=tuple(values)), Motion('turn', yaw=0.7), FP)


def test_rear_then_reassess_no_automatic_second_rear():
    # Front and both sides unknown: no continuation after rear, so explicit failure.
    unknown = replace(scan(), certified=False)
    assert choose(unknown, FP, 0.0, backed=True, need_turn=True) is None


def test_latched_failure_blocks_next_action():
    for reason in ('cancelled', 'timeout', 'communication', 'collision_monitor_stop',
                   'controller_conflict', 'no_independent_progress'):
        state = Session()
        assert state.begin()
        state.fail(reason)
        assert not state.begin() and not state.finish()
        assert state.reason == reason
        assert state.authorize() and state.begin()


def test_attempt_budget_survives_success_and_goal_change():
    state = Session()
    assert state.begin() and state.finish()
    assert state.begin() and state.finish()
    assert not state.begin()
    assert state.authorize() and state.begin()


def test_odom_is_not_progress_input():
    assert measured_progress((0, 0, 0), (0, 0, 0), Motion('rear', x=-0.1)) == 0
    assert measured_progress((0, 0, math.pi / 2), (0, -0.1, math.pi / 2),
                             Motion('rear', x=-0.1)) == pytest.approx(0.1)


def test_turn_requires_explicit_need():
    # No goal orientation is overwritten in this policy.
    assert choose(scan(), FP, math.pi / 2).yaw == 0
    assert choose(scan(), FP, None) is None


def test_remaining_arc_tracks_detour_not_just_goal_distance():
    from robo_recovery.supervised import remaining_path_length
    path = [(0, 0), (0, 1), (1, 1), (1, 0)]
    assert remaining_path_length(path, (0, 0)) == pytest.approx(3.0)
    assert remaining_path_length(path, (0, 0.5)) == pytest.approx(2.5)
    # A useful lateral detour can initially increase Euclidean goal distance.
    assert math.dist((0, 0.5), (1, 0)) > math.dist((0, 0), (1, 0))
