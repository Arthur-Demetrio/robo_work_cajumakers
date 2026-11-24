import numpy as np
import pytest
from robo_virtual_wall.point_probe import inspect_point, measured_check


def calibration():
    return (np.array([[.01,0,0],[0,.01,-.5],[0,0,1.]]),(100,100),
            np.float32([[20,20],[80,20],[80,80],[20,80]]))


def test_click_projection_sides_outside_and_fit_reference():
    left = inspect_point((35,60),calibration())
    assert left['predicted_m'] == pytest.approx([.35,.10])
    right = inspect_point((35,40),calibration())
    assert right['predicted_m'] == pytest.approx([.35,-.10])
    assert inspect_point((10,50),calibration()) is None
    assert inspect_point((float('nan'),50),calibration()) is None
    reference = inspect_point((21,21),calibration(),[[20,20],[80,80]])
    assert reference['fitting_reference']
    with pytest.raises(ValueError,match='Cruz'):
        measured_check(reference,'21 -29')


def test_measured_check_units_signs_and_invalid_input():
    selected = inspect_point((35,40),calibration())
    check = measured_check(selected,'35,0 -10')
    assert check['metres'] == pytest.approx([.35,-.10])
    assert check['error_m'] == pytest.approx(0,abs=1e-12)
    for text in ['35','a b','nan 10','-35 10','35 10 5']:
        with pytest.raises(ValueError):
            measured_check(selected,text)
