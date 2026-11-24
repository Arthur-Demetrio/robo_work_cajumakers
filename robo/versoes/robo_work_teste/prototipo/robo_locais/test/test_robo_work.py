"""Pecas adaptadas ao sistema Robo-WORK: bancadas e pegada no costmap."""

import math

import pytest

from robo_locais.locais import Local, MapaDeLocais, Pose2D, aproximacao_da_bancada
from robo_locais.pegada import Grade, pegada_livre


def test_bancada_robo_para_de_frente_a_0565_da_borda():
    # Borda vertical em x=1, lado livre a esquerda (x=0).
    pose = aproximacao_da_bancada((1.0, 0.0), (1.0, 1.0), (0.0, 0.5))
    assert pose.x == pytest.approx(1.0 - 0.565)
    assert pose.y == pytest.approx(0.5)
    assert pose.yaw == pytest.approx(0.0)  # +X do robo aponta para a bancada


def test_bancada_lado_livre_escolhe_o_lado():
    pose = aproximacao_da_bancada((1.0, 0.0), (1.0, 1.0), (2.0, 0.5))
    assert pose.x == pytest.approx(1.565)
    assert abs(pose.yaw) == pytest.approx(math.pi)


def test_bancada_mesma_conta_do_station_manager():
    # Valores de referencia de robo_station_manager.geometry.compute_approach
    # para p1=(0,0), p2=(0.9,0), livre=(0.45,1) com 0.365 + 0.20.
    pose = aproximacao_da_bancada((0.0, 0.0), (0.9, 0.0), (0.45, 1.0))
    assert (pose.x, pose.y) == pytest.approx((0.45, 0.565))
    assert pose.yaw == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize("p2,livre", [((0.1, 0.0), (0.05, 1.0)), ((0.9, 0.0), (0.45, 0.05))])
def test_bancada_invalida_e_recusada(p2, livre):
    with pytest.raises(ValueError):
        aproximacao_da_bancada((0.0, 0.0), p2, livre)


def test_borda_ida_e_volta_no_yaml(tmp_path):
    mapa = MapaDeLocais()
    mapa.definir(Local("A", "coleta", Pose2D(0, 0), borda=[(1.0, 0.0), (1.0, 0.9)]))
    arquivo = tmp_path / "l.yaml"
    mapa.salvar(str(arquivo))
    assert MapaDeLocais.carregar(str(arquivo)).obter("A").borda == [(1.0, 0.0), (1.0, 0.9)]


def grade(livre=True, desconhecido_em=None, obstaculo_em=None):
    """Costmap 4 x 4 m, 5 cm, origem em (-2, -2)."""
    n = 80
    dados = [0] * (n * n)

    def marca(x, y, valor):
        dados[int((y + 2) / 0.05) * n + int((x + 2) / 0.05)] = valor

    if desconhecido_em:
        marca(*desconhecido_em, -1)
    if obstaculo_em:
        marca(*obstaculo_em, 99)
    return Grade(dados, n, n, 0.05, -2.0, -2.0)


def test_pegada_livre_em_area_vazia():
    assert pegada_livre(grade(), Pose2D(0, 0, 0))[0]


def test_obstaculo_sob_a_frente_do_robo_reprova():
    livre, motivo = pegada_livre(grade(obstaculo_em=(0.33, 0.0)), Pose2D(0, 0, 0))
    assert not livre and "obstaculo" in motivo


def test_obstaculo_logo_alem_da_margem_passa():
    assert pegada_livre(grade(obstaculo_em=(0.50, 0.0)), Pose2D(0, 0, 0))[0]


def test_pegada_gira_com_o_robo():
    # Robo virado para +Y: a frente (0,365 m) fica em y positivo.
    assert not pegada_livre(grade(obstaculo_em=(0.0, 0.33)), Pose2D(0, 0, math.pi / 2))[0]
    # ...e o lado direito fica em x positivo: meia largura 0,315 + margem.
    assert not pegada_livre(grade(obstaculo_em=(0.33, 0.0)), Pose2D(0, 0, math.pi / 2))[0]
    assert pegada_livre(grade(obstaculo_em=(0.45, 0.0)), Pose2D(0, 0, math.pi / 2))[0]


def test_desconhecido_e_fora_do_mapa_reprovam():
    assert not pegada_livre(grade(desconhecido_em=(0.0, 0.0)), Pose2D(0, 0, 0))[0]
    assert not pegada_livre(grade(), Pose2D(1.9, 0, 0))[0]
