from pathlib import Path
import ast
import sys
import pytest
ROOT=Path('/home/marcos/robo_jazzy_ws')
sys.path.insert(0,str(ROOT/'src/robo_navigation/test'))
from test_nav2_principal import metodo


def test_logger_rclpy_real_varias_severidades():
    import rclpy
    from rclpy.logging import get_logger
    from types import SimpleNamespace
    rclpy.init()
    try:
        no=SimpleNamespace(get_logger=lambda:get_logger('regressao_severidade_supervisor'))
        fn=metodo('_log_nav_principal')
        for nivel in ('info','warn','error','info','warning','error'):
            fn(no,nivel,'mensagem de regressao')
        with pytest.raises(ValueError):fn(no,'invalido','teste')
    finally:rclpy.try_shutdown()


def test_regressao_logger_antigo_reproduz_erro_real():
    import rclpy
    from rclpy.logging import get_logger
    rclpy.init()
    try:
        logger=get_logger('repro_severidade_antiga')
        def antigo(nivel):getattr(logger,nivel)('regressao')
        antigo('info')
        with pytest.raises(ValueError,match='severity'):antigo('warning')
    finally:rclpy.try_shutdown()
