import math
from pathlib import Path

import pytest

from robo_locais.locais import Local, MapaDeLocais, Pose2D, yaw_de_quaternion
from robo_locais.missao import (
    AcoesSimuladas,
    ExecutorMissao,
    NavegadorSimulado,
    carregar_missao,
    expandir,
)

CONFIG = Path(__file__).resolve().parent.parent / "config"


def silencioso(_msg):
    pass


def test_yaml_ida_e_volta(tmp_path):
    mapa = MapaDeLocais(mapa="teste")
    mapa.definir(Local("A", "alinhamento", Pose2D(1.0, 2.0, 0.5)))
    mapa.definir(Local("Y", "entrega", Pose2D(0, 0), area=[(0, 0), (1, 0), (1, 1)]))
    arquivo = tmp_path / "locais.yaml"
    mapa.salvar(str(arquivo))

    lido = MapaDeLocais.carregar(str(arquivo))
    assert lido.mapa == "teste"
    assert lido.obter("A").pose == Pose2D(1.0, 2.0, 0.5)
    assert lido.obter("Y").area == [(0, 0), (1, 0), (1, 1)]


def test_arquivo_inexistente_gera_mapa_vazio(tmp_path):
    assert MapaDeLocais.carregar(str(tmp_path / "nao_existe.yaml")).locais == {}


def test_ponto_dentro_da_area():
    local = Local("Y", "entrega", Pose2D(0, 0), area=[(0, 0), (2, 0), (2, 2), (0, 2)])
    assert local.ponto_dentro_da_area(1, 1)
    assert not local.ponto_dentro_da_area(3, 1)


def test_yaw_de_quaternion():
    assert yaw_de_quaternion(math.sin(0.75), math.cos(0.75)) == pytest.approx(1.5)


def test_transportar_expande_em_seis_passos():
    passos = expandir([{"acao": "transportar", "objeto": "M20", "de": "X", "para": "Z"}])
    assert [p.acao for p in passos] == [
        "ir_para", "alinhar", "pegar", "ir_para", "alinhar", "soltar",
    ]
    assert passos[2].objeto == "M20"


def test_missao_exemplo_completa_em_simulacao():
    mapa = MapaDeLocais.carregar(str(CONFIG / "locais_exemplo.yaml"))
    passos = carregar_missao(str(CONFIG / "missao_exemplo.yaml"))
    navegador = NavegadorSimulado(silencioso)

    resultado = ExecutorMissao(mapa, navegador, AcoesSimuladas(silencioso), silencioso).executar(passos)

    assert resultado.sucesso
    assert navegador.historico[-1] == mapa.obter("inicio").pose


def test_local_desconhecido_nao_executa_nada():
    mapa = MapaDeLocais()
    mapa.definir(Local("A", "passagem", Pose2D(0, 0)))
    navegador = NavegadorSimulado(silencioso)
    executor = ExecutorMissao(mapa, navegador, AcoesSimuladas(silencioso), silencioso)

    resultado = executor.executar(expandir([{"acao": "ir_para", "local": "B"}]))

    assert not resultado.sucesso
    assert navegador.historico == []


def test_soltar_sem_carregar_falha_e_para():
    mapa = MapaDeLocais()
    mapa.definir(Local("Z", "entrega", Pose2D(0, 0)))
    executor = ExecutorMissao(
        mapa, NavegadorSimulado(silencioso), AcoesSimuladas(silencioso), silencioso
    )

    resultado = executor.executar(
        expandir([{"acao": "soltar", "local": "Z"}, {"acao": "ir_para", "local": "Z"}])
    )

    assert not resultado.sucesso
    assert len(resultado.passos) == 1


def test_acao_registrada_como_extensao():
    mapa = MapaDeLocais()
    mapa.definir(Local("M", "coleta", Pose2D(0, 0)))
    executor = ExecutorMissao(
        mapa, NavegadorSimulado(silencioso), AcoesSimuladas(silencioso), silencioso
    )
    chamados = []
    executor.registrar_acao("escanear_mesa", lambda p, l: chamados.append(l.nome) or True)

    assert executor.executar(expandir([{"acao": "escanear_mesa", "local": "M"}])).sucesso
    assert chamados == ["M"]


def test_nome_automatico_por_tipo_e_por_area():
    mapa = MapaDeLocais()
    assert mapa.proximo_nome("coleta") == "C1"
    mapa.definir(Local("C1", "coleta", Pose2D(0, 0)))
    assert mapa.proximo_nome("coleta") == "C2"
    assert mapa.proximo_nome("entrega", area_pai="Y") == "Y1"
    assert mapa.proximo_nome("tipo_novo") == "L1"


def test_nome_automatico_do_inicio():
    mapa = MapaDeLocais()
    assert mapa.proximo_nome("inicio") == "inicio"
    mapa.definir(Local("inicio", "inicio", Pose2D(0, 0)))
    assert mapa.proximo_nome("inicio") == "inicio1"
