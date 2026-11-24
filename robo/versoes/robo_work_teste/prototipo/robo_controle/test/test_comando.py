import pytest

from robo_controle.comando import BotaoSegurado, Limites, joystick_para_velocidade

L = Limites()


def test_analogicos_soltos_param():
    assert joystick_para_velocidade(0.05, -0.1, 0.1) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("entrada", [0.2, 0.5, 1.0, -0.3, -1.0])
def test_nenhum_eixo_cai_na_deadband_do_smoother(entrada):
    for v in joystick_para_velocidade(entrada, entrada, entrada):
        assert abs(v) > 0.015


def test_maximos_sao_os_do_velocity_smoother():
    assert joystick_para_velocidade(1.0, 1.0, 1.0) == pytest.approx((0.08, 0.08, 0.14))
    assert joystick_para_velocidade(-1.0, -1.0, -1.0) == pytest.approx((-0.08, -0.08, -0.14))


def test_eixos_sao_independentes_lateral_pura_e_diagonal():
    vx, vy, w = joystick_para_velocidade(0.0, 1.0, 0.0)
    assert vx == 0.0 and vy > 0 and w == 0.0
    vx, vy, w = joystick_para_velocidade(1.0, -1.0, 0.0)
    assert vx > 0 and vy < 0 and w == 0.0


def test_esquerda_e_positivo_como_no_ros():
    assert joystick_para_velocidade(0.0, 0.8, 0.0)[1] > 0
    assert joystick_para_velocidade(0.0, 0.0, 0.8)[2] > 0


def test_modo_lento_reduz_mas_nao_zera():
    normal = joystick_para_velocidade(1.0, 1.0, 1.0)
    lento = joystick_para_velocidade(1.0, 1.0, 1.0, lento=True)
    for n, l in zip(normal, lento):
        assert 0.015 < l < n


def test_botao_segurado_dispara_uma_vez_apos_o_tempo():
    botao = BotaoSegurado(2.0)
    assert not botao.atualizar(True, 10.0)
    assert botao.segurando
    assert not botao.atualizar(True, 11.9)
    assert botao.atualizar(True, 12.0)
    assert not botao.atualizar(True, 15.0)  # continua segurando: nao repete
    assert not botao.segurando


def test_botao_segurado_solto_antes_nao_dispara():
    botao = BotaoSegurado(2.0)
    botao.atualizar(True, 0.0)
    assert not botao.atualizar(False, 1.5)
    assert not botao.atualizar(True, 1.6)   # recomecou a contar
    assert not botao.atualizar(True, 3.5)
    assert botao.atualizar(True, 3.6)
