from pathlib import Path

from robo_locais.locais import Local, MapaDeLocais, Pose2D
from robo_locais.missao import AcoesSimuladas, NavegadorSimulado
from robo_locais.planejador import (
    Entrega,
    ExecutorObjetivo,
    Objetivo,
    obter_objetivo,
    objetivo_dos_locais,
    resolver,
)

CONFIG = Path(__file__).resolve().parent.parent / "config"


def silencioso(_msg):
    pass


def executor(mapa, capacidade=1, acoes=None, navegador=None, **kw):
    return ExecutorObjetivo(
        mapa,
        navegador or NavegadorSimulado(silencioso),
        acoes or AcoesSimuladas(silencioso, capacidade),
        silencioso,
        capacidade=capacidade,
        **kw,
    )


def arena():
    mapa = MapaDeLocais()
    mapa.definir(Local("inicio", "inicio", Pose2D(0, 0)))
    mapa.definir(Local("X", "coleta", Pose2D(3, 0), objetos=["azul", "caixa", "caixa"]))
    mapa.definir(Local("Y", "entrega", Pose2D(0, 3), recebe=["caixa", "caixa"]))
    mapa.definir(Local("Y1", "entrega", Pose2D(0, 3), area_pai="Y"))
    mapa.definir(Local("Y2", "entrega", Pose2D(0.5, 3), area_pai="Y"))
    mapa.definir(Local("Z", "entrega", Pose2D(3, 3), recebe=["azul"]))
    return mapa


def test_anotacoes_ida_e_volta_no_yaml(tmp_path):
    arquivo = tmp_path / "locais.yaml"
    arena().salvar(str(arquivo))
    lido = MapaDeLocais.carregar(str(arquivo))
    assert lido.obter("X").objetos == ["azul", "caixa", "caixa"]
    assert lido.obter("Z").recebe == ["azul"]


def test_objetivo_sai_das_anotacoes_do_mapeamento():
    objetivo = objetivo_dos_locais(arena())
    assert sorted((e.objeto, e.para) for e in objetivo.entregas) == [
        ("azul", "Z"), ("caixa", "Y"), ("caixa", "Y"),
    ]
    assert objetivo.terminar_em == "inicio"


def test_area_vira_os_pontos_dentro_dela():
    tarefas, _, problemas = resolver(objetivo_dos_locais(arena()), arena())
    assert not problemas
    caixa = next(t for t in tarefas if t.objeto == "caixa")
    assert caixa.origens == ["X"] and caixa.destinos == ["Y1", "Y2"]


def test_entrega_tudo_sozinho_e_volta_ao_inicio():
    mapa = arena()
    navegador = NavegadorSimulado(silencioso)
    rel = executor(mapa, navegador=navegador).executar(objetivo_dos_locais(mapa))
    assert rel.sucesso
    assert sorted(rel.entregues) == ["azul em Z", "caixa em Y1", "caixa em Y2"]
    assert navegador.historico[-1] == mapa.obter("inicio").pose


def test_capacidade_maior_encurta_o_caminho():
    mapa = arena()
    objetivo = objetivo_dos_locais(mapa)
    um = executor(mapa, 1).planejar(objetivo)
    dois = executor(mapa, 2).planejar(objetivo)
    assert not um.problemas and not dois.problemas
    assert dois.custo < um.custo


def test_escolhe_a_origem_mais_perto():
    mapa = MapaDeLocais()
    mapa.definir(Local("inicio", "inicio", Pose2D(0, 0)))
    mapa.definir(Local("longe", "coleta", Pose2D(10, 0), objetos=["caixa"]))
    mapa.definir(Local("perto", "coleta", Pose2D(1, 0), objetos=["caixa"]))
    mapa.definir(Local("Z", "entrega", Pose2D(0, 1), recebe=["caixa"]))
    plano = executor(mapa).planejar(objetivo_dos_locais(mapa))
    assert plano.acoes[0].local == "perto"


def test_falta_de_objeto_aparece_antes_de_executar():
    mapa = arena()
    mapa.obter("Z").recebe = ["vermelha"]
    plano = executor(mapa).planejar(objetivo_dos_locais(mapa))
    assert any("vermelha" in p for p in plano.problemas)


def test_sem_entregas_e_problema():
    mapa = MapaDeLocais()
    mapa.definir(Local("inicio", "inicio", Pose2D(0, 0)))
    assert executor(mapa).planejar(objetivo_dos_locais(mapa)).problemas


class AcoesQueFalhamEm(AcoesSimuladas):
    """Nunca consegue pegar nos locais indicados (objeto nao esta la)."""

    def __init__(self, sem_objeto):
        super().__init__(silencioso)
        self.sem_objeto = set(sem_objeto)

    def pegar(self, local, objeto):
        if local.nome in self.sem_objeto:
            return False
        return super().pegar(local, objeto)


def test_objeto_ausente_replaneja_para_outra_origem():
    mapa = MapaDeLocais()
    mapa.definir(Local("inicio", "inicio", Pose2D(0, 0)))
    mapa.definir(Local("perto", "coleta", Pose2D(1, 0), objetos=["caixa"]))
    mapa.definir(Local("longe", "coleta", Pose2D(5, 0), objetos=["caixa"]))
    mapa.definir(Local("Z", "entrega", Pose2D(0, 1), recebe=["caixa"]))
    rel = executor(mapa, acoes=AcoesQueFalhamEm({"perto"})).executar(
        objetivo_dos_locais(mapa)
    )
    assert rel.sucesso
    assert rel.entregues == ["caixa em Z"]


def test_sem_saida_abandona_mas_entrega_o_resto():
    mapa = arena()
    rel = executor(mapa, acoes=AcoesQueFalhamEm({"X"})).executar(
        Objetivo([Entrega("azul", "Z")], "inicio")
    )
    assert not rel.sucesso
    assert len(rel.abandonadas) == 1

    mapa.definir(Local("W", "coleta", Pose2D(2, 2), objetos=["bola"]))
    rel = executor(mapa, acoes=AcoesQueFalhamEm({"X"})).executar(
        Objetivo([Entrega("azul", "Z"), Entrega("bola", "Y")], "inicio")
    )
    assert rel.entregues == ["bola em Y2"]  # Y2 e o ponto de Y mais perto de W
    assert len(rel.abandonadas) == 1


def test_arquivos_de_exemplo_planejam_sem_problemas():
    mapa = MapaDeLocais.carregar(str(CONFIG / "locais_exemplo.yaml"))
    pelos_locais = executor(mapa).planejar(obter_objetivo(mapa))
    pelo_arquivo = executor(mapa).planejar(
        obter_objetivo(mapa, str(CONFIG / "objetivo_exemplo.yaml"))
    )
    assert not pelos_locais.problemas and not pelo_arquivo.problemas
    assert len(pelos_locais.tarefas) == len(pelo_arquivo.tarefas) == 3


def test_de_explicito_sem_anotacao_confia_no_objetivo():
    mapa = arena()
    mapa.definir(Local("M", "coleta", Pose2D(1, 1)))
    objetivo = Objetivo([Entrega("M20", "Z", de="M"), Entrega("M20", "Z", de="M")])
    tarefas, inventario, problemas = resolver(objetivo, mapa)
    assert not problemas and len(tarefas) == 2
    assert inventario["M"]["M20"] == 2
