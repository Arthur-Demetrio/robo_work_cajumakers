import json

from robo_locais.locais import Local, MapaDeLocais, Pose2D
from robo_locais.missao import AcoesSimuladas, ExecutorMissao, NavegadorSimulado, expandir
from robo_locais.planejador import ExecutorObjetivo, objetivo_dos_locais
from robo_locais.registro import ArquivoJsonl, RegistroMissao, formatar, ler


def silencioso(_msg):
    pass


def arena():
    mapa = MapaDeLocais(mapa="teste")
    mapa.definir(Local("inicio", "inicio", Pose2D(0, 0)))
    mapa.definir(Local("perto", "coleta", Pose2D(1, 0), objetos=["caixa"]))
    mapa.definir(Local("longe", "coleta", Pose2D(5, 0), objetos=["caixa"]))
    mapa.definir(Local("Z", "entrega", Pose2D(0, 1), recebe=["caixa"]))
    return mapa


class SemCaixaEmPerto(AcoesSimuladas):
    def pegar(self, local, objeto):
        return False if local.nome == "perto" else super().pegar(local, objeto)


def missao_com_falha(tmp_path):
    mapa = arena()
    registro = RegistroMissao(ArquivoJsonl(str(tmp_path), "missao"), {"modo": "simulado"})
    executor = ExecutorObjetivo(
        mapa, NavegadorSimulado(silencioso), SemCaixaEmPerto(silencioso), silencioso,
        ao_evento=registro.evento,
    )
    objetivo = objetivo_dos_locais(mapa)
    registro.plano_inicial(executor.planejar(objetivo))
    relatorio = executor.executar(objetivo)
    resumo = registro.fechar(relatorio, "concluida" if relatorio.sucesso else "incompleta")
    return registro, resumo


def test_registro_conta_passos_falhas_e_replanejamento(tmp_path):
    registro, resumo = missao_com_falha(tmp_path)
    assert resumo["situacao"] == "concluida"
    assert resumo["entregues"] == ["caixa em Z"]
    assert resumo["objetos_ausentes"] == 1          # desistiu de 'perto'
    assert resumo["replanejamentos"] >= 1           # e foi para 'longe'
    assert resumo["por_acao"]["pegar"]["falhas"] == 2   # max_tentativas = 2
    assert resumo["por_acao"]["soltar"] == {"vezes": 1, "falhas": 0, "tempo_s": 0.0}


def test_arquivo_tem_uma_linha_json_por_evento(tmp_path):
    registro, _ = missao_com_falha(tmp_path)
    linhas = registro.arquivo.caminho.read_text().splitlines()
    eventos = [json.loads(l) for l in linhas]
    tipos = [e["evento"] for e in eventos]
    assert tipos[0] == "inicio" and tipos[1] == "plano_inicial" and tipos[-1] == "fim"
    assert {"passo", "plano", "objeto_ausente", "entregue"} <= set(tipos)
    passo = next(e for e in eventos if e["evento"] == "passo")
    assert {"acao", "local", "sucesso", "duracao_s"} <= set(passo)


def test_ver_registro_mostra_passos_e_fim(tmp_path):
    registro, _ = missao_com_falha(tmp_path)
    texto = formatar(ler(registro.arquivo.caminho))
    assert "PLANO INICIAL" in texto and "FALHOU pegar perto" in texto
    assert "FIM: CONCLUIDA" in texto and "pegar: " in texto


def test_missao_interrompida_fica_sem_fim(tmp_path):
    registro = RegistroMissao(ArquivoJsonl(str(tmp_path), "missao"), {})
    registro.evento("passo", {"acao": "ir_para", "local": "Z", "sucesso": True,
                              "duracao_s": 1.0})
    assert "SEM FIM" in formatar(ler(registro.arquivo.caminho))


def test_fechar_duas_vezes_nao_duplica(tmp_path):
    registro = RegistroMissao(ArquivoJsonl(str(tmp_path), "missao"), {})
    registro.fechar(situacao="concluida")
    registro.fechar(situacao="interrompida")  # ex.: Ctrl+C depois do fim
    tipos = [e["evento"] for e in ler(registro.arquivo.caminho)]
    assert tipos.count("fim") == 1


def test_pausa_de_emergencia_soma_tempo(tmp_path):
    registro = RegistroMissao(None, {})
    registro.evento("pausa_emergencia", {"duracao_s": 3.0})
    registro.evento("pausa_emergencia", {"duracao_s": 2.5})
    resumo = registro.fechar(situacao="concluida")
    assert resumo["pausas_emergencia"] == 2
    assert resumo["tempo_pausado_emergencia_s"] == 5.5


def test_missao_manual_tambem_registra_passos(tmp_path):
    registro = RegistroMissao(ArquivoJsonl(str(tmp_path), "missao_manual"), {})
    mapa = arena()
    ExecutorMissao(mapa, NavegadorSimulado(silencioso), AcoesSimuladas(silencioso),
                   silencioso, registro.evento).executar(
        expandir([{"acao": "ir_para", "local": "Z"}]))
    resumo = registro.fechar(situacao="concluida")
    assert resumo["por_acao"]["ir_para"]["vezes"] == 1
