"""Assertions on generated native experiments; not physical acceptance tests."""
import json
import math
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope='module')
def rows():
    path = ROOT / 'review/antecipacao/native_comparison.json'
    if not path.exists():
        pytest.skip('run native experiment matrix first')
    return json.loads(path.read_text())


def get(rows, profile, scenario, replanning=False):
    return next(r for r in rows if r['profile'] == profile and r['scenario'] == scenario and
                r['replanning'] == replanning)


def test_native_processes_finished_cleanly(rows):
    assert len(rows) >= 36
    assert all(r['returncode'] == 0 for r in rows)


def test_native_lateral_shim_counterfactual(rows):
    shim, dwb = get(rows, 'shim', 'lateral'), get(rows, 'dwb', 'lateral')
    assert shim['complete'] and dwb['complete']
    assert shim['first_translation_s'] > dwb['first_translation_s']
    assert shim['intermediate_rotation_rad'] > 1.0
    assert dwb['rotation_rad'] == 0.0


@pytest.mark.parametrize('scenario', ['open', 'lateral', 'curve'])
def test_predictive_normal_navigation(rows, scenario):
    result = get(rows, 'predictive', scenario)
    assert result['complete'] and result['collisions'] == 0


@pytest.mark.parametrize('scenario', ['obstacle', 'dynamic'])
def test_predictive_contour_with_global_replanning(rows, scenario):
    result = get(rows, 'predictive', scenario, True)
    assert result['complete'] and result['failures'] == 0 and result['collisions'] == 0
    assert result['plan_collision_samples'] == 0
    assert result['first_lateral_x'] < 0.30  # box starts at x=.85


def test_final_required_orientation_in_native_controller(rows):
    for profile in ('shim', 'dwb', 'early', 'predictive'):
        result = get(rows, profile, 'final_yaw')
        assert result['complete']
        assert abs(math.atan2(math.sin(result['yaw']-math.pi/2),
                              math.cos(result['yaw']-math.pi/2))) <= 0.17


def test_unresolved_near_and_corner_are_explicit(rows):
    for scenario in ('near', 'corner'):
        result = get(rows, 'predictive', scenario)
        assert not result['complete']
        assert result['collisions'] == 0
