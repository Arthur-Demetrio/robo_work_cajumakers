import json

from robo_controle.registro import ArquivoJsonl, SessaoControle, formatar, ler


class Relogio:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def sessao(tmp_path):
    relogio = Relogio()
    arquivo = ArquivoJsonl(str(tmp_path), "controle", relogio)
    return SessaoControle(arquivo, {"teste": True}), arquivo, relogio


def eventos(arquivo):
    return [json.loads(l) for l in arquivo.caminho.read_text().splitlines()]


def test_trecho_registra_duracao_e_maximos(tmp_path):
    s, arq, rel = sessao(tmp_path)
    s.comando((0.05, 0.0, 0.0))
    rel.t += 1.0
    s.comando((0.08, 0.03, 0.0), lento=True)
    rel.t += 1.5
    s.comando(None)
    trecho = [e for e in eventos(arq) if e["evento"] == "trecho"][0]
    assert trecho["duracao_s"] == 2.5
    assert (trecho["max_vx"], trecho["max_vy"], trecho["max_w"]) == (0.08, 0.03, 0.0)
    assert trecho["movimentos"] == ["frente", "lateral"]
    assert trecho["lento"] is True


def test_so_grava_nas_transicoes(tmp_path):
    s, arq, rel = sessao(tmp_path)
    for _ in range(50):          # 50 ciclos andando = 1 trecho
        s.comando((0.05, 0.0, 0.0))
        rel.t += 0.05
    for _ in range(50):          # 50 ciclos cedendo = 1 episodio
        s.cedendo(True)
    s.cedendo(False)
    tipos = [e["evento"] for e in eventos(arq)]
    assert tipos.count("cedeu_ao_nav2") == 1
    assert tipos.count("parou_de_ceder") == 1


def test_emergencia_conta_tempo_parado(tmp_path):
    s, arq, rel = sessao(tmp_path)
    s.comando((0.05, 0.0, 0.0))
    s.emergencia(True)
    rel.t += 4.0
    s.emergencia(False)
    acionada, liberada = [e for e in eventos(arq) if e["evento"].startswith("emergencia")]
    assert acionada["dirigindo"] is True
    assert liberada["parado_s"] == 4.0
    r = s.fechar()
    assert r["emergencias"] == 1 and r["tempo_em_emergencia_s"] == 4.0


def test_desconexao_no_meio_do_trecho(tmp_path):
    s, arq, rel = sessao(tmp_path)
    s.joy(False)                 # comecou sem controle: nao e desconexao
    s.joy(True)
    s.comando((0.05, 0.0, 0.0))
    s.joy(False)
    r = s.fechar()
    assert r["desconexoes_do_controle"] == 1
    assert r["desconexoes_dirigindo"] == 1


def test_servicos_contam_sucesso_e_falha(tmp_path):
    s, arq, rel = sessao(tmp_path)
    s.servico("salvar_local", True, "local 'C1'")
    s.servico("salvar_local", True, "local 'C2'")
    s.servico("finalizar", False, "NAO liberada")
    r = s.fechar()
    assert r["locais_salvos"] == 2 and r["finalizacoes_recusadas"] == 1


def test_resumo_fecha_trecho_aberto_e_ver_registro_le(tmp_path):
    s, arq, rel = sessao(tmp_path)
    s.comando((0.0, 0.0, 0.1))
    rel.t += 2.0
    s.fechar()
    evs = ler(arq.caminho)
    assert evs[-1]["evento"] == "resumo" and evs[-1]["trechos"] == 1
    texto = formatar(evs)
    assert "SESSAO DE CONTROLE" in texto and "RESUMO" in texto


def test_sessao_que_caiu_e_avisada(tmp_path):
    s, arq, rel = sessao(tmp_path)
    s.erro("ValueError: Logger severity cannot be changed between calls.")
    texto = formatar(ler(arq.caminho))  # sem fechar: o no caiu
    assert "erro" in texto and "SEM RESUMO" in texto


def test_sem_pasta_nao_grava_nada():
    s = SessaoControle(None, {})
    s.comando((0.05, 0.0, 0.0))
    assert s.fechar()["trechos"] == 1


def test_emergencia_dirigindo_fecha_o_trecho_antes(tmp_path):
    s, arq, rel = sessao(tmp_path)
    s.comando((0.05, 0.0, 0.0))
    rel.t += 1.0
    s.emergencia(True)
    tipos = [e["evento"] for e in eventos(arq)]
    assert tipos[-2:] == ["trecho", "emergencia_acionada"]


def test_resposta_longa_do_finalizar_e_resumida(tmp_path):
    s, arq, rel = sessao(tmp_path)
    plano = "locais salvos; plano:\n3 entrega(s)\n  1. ir_para X\n  2. alinhar X\n  AVISO: costmap indisponivel"
    s.servico("finalizar", True, plano)
    msg = eventos(arq)[-1]["mensagem"]
    assert msg == "locais salvos; plano: | AVISO: costmap indisponivel"
