"""Registro dos testes de controle, sem ROS.

Cada sessao do controle_node vira um arquivo JSONL (um evento JSON por
linha, gravado na hora) em ~/robo_testes_prototipos/controle/. Se o no cair
no meio do teste, tudo ate ali ja esta no disco. A ultima linha e o resumo.

O que importa num teste de direcao manual:
  - trechos dirigidos: duracao, maior |vx|, |vy| e |w|, se usou modo lento;
  - seguranca: paradas de emergencia (e quanto tempo ficou parado), vezes
    que o controle cedeu ao Nav2, controle desconectado no meio do trecho;
  - atalhos: salvar local, salvar mapa e finalizar o mapeamento,
    com a resposta de cada servico.

Leitura: `ros2 run robo_controle ver_registro` (ultima sessao) ou
`ros2 run robo_controle ver_registro <arquivo.jsonl>`.
"""

from __future__ import annotations

import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

PASTA_PADRAO = "~/robo_testes_prototipos/controle"

Velocidade = Tuple[float, float, float]


class ArquivoJsonl:
    """Grava eventos {"t": iso, "s": segundos desde o inicio, "evento": ...}."""

    def __init__(self, pasta: str, sufixo: str, relogio: Callable[[], float] = time.time):
        self.relogio = relogio
        self.inicio = relogio()
        destino = Path(pasta).expanduser()
        destino.mkdir(parents=True, exist_ok=True)
        nome = datetime.fromtimestamp(self.inicio).strftime("%Y-%m-%d_%H-%M-%S")
        self.caminho = destino / f"{nome}_{sufixo}.jsonl"
        self._arquivo = self.caminho.open("a", encoding="utf-8")

    def gravar(self, evento: str, **dados) -> None:
        agora = self.relogio()
        linha = {
            "t": datetime.fromtimestamp(agora).isoformat(timespec="milliseconds"),
            "s": round(agora - self.inicio, 3),
            "evento": evento,
            **dados,
        }
        self._arquivo.write(json.dumps(linha, ensure_ascii=False) + "\n")
        self._arquivo.flush()  # nao perde nada se o processo morrer

    def fechar(self) -> None:
        if not self._arquivo.closed:
            self._arquivo.close()


class SessaoControle:
    """Transforma o que acontece no controle_node em eventos e estatisticas.

    O no chama os metodos a cada ciclo; esta classe so grava nas TRANSICOES
    (comecou/terminou um trecho, entrou/saiu de emergencia...), para o
    arquivo ficar pequeno e legivel.
    """

    def __init__(self, arquivo: Optional[ArquivoJsonl], config: Dict) -> None:
        self.arquivo = arquivo
        self.relogio = arquivo.relogio if arquivo else time.time
        self.inicio = self.relogio()
        # Trecho dirigido em andamento.
        self._trecho_inicio: Optional[float] = None
        self._trecho_max = [0.0, 0.0, 0.0]
        self._trecho_lento = False
        # Episodios.
        self._emergencia_desde: Optional[float] = None
        self._cedendo_desde: Optional[float] = None
        self._joy_ok: Optional[bool] = None
        # Totais para o resumo.
        self.total = {
            "trechos": 0,
            "tempo_dirigindo_s": 0.0,
            "max_vx": 0.0, "max_vy": 0.0, "max_w": 0.0,
            "emergencias": 0,
            "tempo_em_emergencia_s": 0.0,
            "cedeu_ao_nav2": 0,
            "desconexoes_do_controle": 0,
            "desconexoes_dirigindo": 0,
            "locais_salvos": 0, "locais_falharam": 0,
            "mapas_salvos": 0, "mapas_falharam": 0,
            "finalizacoes_ok": 0, "finalizacoes_recusadas": 0,
            "erros": 0,
        }
        self._gravar("inicio", config=config)

    def _gravar(self, evento: str, **dados) -> None:
        if self.arquivo is not None:
            self.arquivo.gravar(evento, **dados)

    # ---------- Chamado a cada ciclo do controle ----------

    def comando(self, v: Optional[Velocidade], lento: bool = False) -> None:
        """v = velocidade publicada neste ciclo, ou None se parado."""
        agora = self.relogio()
        movendo = v is not None and any(abs(x) > 0 for x in v)
        if movendo:
            if self._trecho_inicio is None:
                self._trecho_inicio = agora
                self._trecho_max = [0.0, 0.0, 0.0]
                self._trecho_lento = False
            for i, x in enumerate(v):
                self._trecho_max[i] = max(self._trecho_max[i], abs(x))
            self._trecho_lento |= lento
        elif self._trecho_inicio is not None:
            self._fechar_trecho(agora)

    def _fechar_trecho(self, agora: float) -> None:
        duracao = agora - self._trecho_inicio
        vx, vy, w = self._trecho_max
        self.total["trechos"] += 1
        self.total["tempo_dirigindo_s"] += duracao
        self.total["max_vx"] = max(self.total["max_vx"], vx)
        self.total["max_vy"] = max(self.total["max_vy"], vy)
        self.total["max_w"] = max(self.total["max_w"], w)
        self._gravar(
            "trecho",
            duracao_s=round(duracao, 2),
            max_vx=round(vx, 3), max_vy=round(vy, 3), max_w=round(w, 3),
            movimentos=[n for n, x in zip(("frente", "lateral", "giro"), (vx, vy, w)) if x > 0],
            lento=self._trecho_lento,
        )
        self._trecho_inicio = None

    def joy(self, conectado: bool) -> None:
        if conectado == self._joy_ok:
            return
        if self._joy_ok is not None or conectado:
            if not conectado:
                self.total["desconexoes_do_controle"] += 1
                if self._trecho_inicio is not None:
                    self.total["desconexoes_dirigindo"] += 1
            self._gravar(
                "controle_conectado" if conectado else "controle_desconectado",
                dirigindo=self._trecho_inicio is not None,
            )
        self._joy_ok = conectado

    def cedendo(self, ativo: bool) -> None:
        """O controle quis andar, mas outro no comandava /cmd_vel_nav."""
        agora = self.relogio()
        if ativo and self._cedendo_desde is None:
            self._cedendo_desde = agora
            self.total["cedeu_ao_nav2"] += 1
            self._gravar("cedeu_ao_nav2")
        elif not ativo and self._cedendo_desde is not None:
            self._gravar("parou_de_ceder", duracao_s=round(agora - self._cedendo_desde, 2))
            self._cedendo_desde = None

    # ---------- Eventos pontuais ----------

    def emergencia(self, ativa: bool) -> None:
        agora = self.relogio()
        if ativa:
            dirigindo = self._trecho_inicio is not None
            if dirigindo:
                self._fechar_trecho(agora)  # o trecho termina na parada
            if self._emergencia_desde is None:
                self._emergencia_desde = agora
                self.total["emergencias"] += 1
            self._gravar("emergencia_acionada", dirigindo=dirigindo)
        else:
            duracao = agora - self._emergencia_desde if self._emergencia_desde else 0.0
            self.total["tempo_em_emergencia_s"] += duracao
            self._emergencia_desde = None
            self._gravar("emergencia_liberada", parado_s=round(duracao, 2))

    def servico(self, nome: str, sucesso: bool, mensagem: str) -> None:
        """Resposta de salvar_local / salvar_mapa / finalizar."""
        chaves = {
            "salvar_local": ("locais_salvos", "locais_falharam"),
            "salvar_mapa": ("mapas_salvos", "mapas_falharam"),
            "finalizar": ("finalizacoes_ok", "finalizacoes_recusadas"),
        }
        ok, falha = chaves.get(nome, (None, None))
        if ok:
            self.total[ok if sucesso else falha] += 1
        self._gravar(nome, sucesso=sucesso, mensagem=resumir(mensagem))

    def erro(self, mensagem: str) -> None:
        self.total["erros"] += 1
        self._gravar("erro", mensagem=mensagem)

    # ---------- Fim ----------

    def resumo(self) -> Dict:
        agora = self.relogio()
        if self._trecho_inicio is not None:
            self._fechar_trecho(agora)
        r = {k: round(v, 3) if isinstance(v, float) else v for k, v in self.total.items()}
        r["duracao_sessao_s"] = round(agora - self.inicio, 1)
        r["terminou_em_emergencia"] = self._emergencia_desde is not None
        return r

    def fechar(self) -> Dict:
        r = self.resumo()
        self._gravar("resumo", **r)
        if self.arquivo is not None:
            self.arquivo.fechar()
        return r


def resumir(mensagem: str) -> str:
    """1a linha + PROBLEMAS/AVISOS. O plano completo fica no registro da missao."""
    linhas = mensagem.strip().splitlines()
    importantes = [l.strip() for l in linhas[1:] if "PROBLEMA" in l or "AVISO" in l]
    return " | ".join(linhas[:1] + importantes)


# ---------- Leitura (ros2 run robo_controle ver_registro) ----------


def ler(caminho: Path) -> List[Dict]:
    eventos = []
    for linha in caminho.read_text(encoding="utf-8").splitlines():
        try:
            eventos.append(json.loads(linha))
        except json.JSONDecodeError:
            eventos.append({"s": None, "evento": "linha_invalida", "mensagem": linha[:80]})
    return eventos


def ultimo_arquivo(pasta: str) -> Optional[Path]:
    arquivos = sorted(Path(pasta).expanduser().glob("*.jsonl"))
    return arquivos[-1] if arquivos else None


def formatar(eventos: List[Dict]) -> str:
    linhas = []
    for e in eventos:
        tipo, s = e.get("evento"), e.get("s")
        dados = {k: v for k, v in e.items() if k not in ("t", "s", "evento")}
        if tipo == "inicio":
            linhas.append(f"SESSAO DE CONTROLE {e.get('t', '')}")
            continue
        if tipo == "resumo":
            linhas.append("RESUMO")
            linhas += [f"  {k}: {v}" for k, v in dados.items()]
            continue
        detalhe = ", ".join(f"{k}={v}" for k, v in dados.items())
        quando = f"{s:>8.1f} s" if isinstance(s, (int, float)) else "       ? s"
        linhas.append(f"[{quando}] {tipo}" + (f": {detalhe}" if detalhe else ""))
    if not eventos or eventos[-1].get("evento") != "resumo":
        linhas.append("(SEM RESUMO: a sessao nao terminou normalmente, ex.: o no caiu)")
    return "\n".join(linhas)


def main_ver(argv=None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    alvo = Path(argv[0]).expanduser() if argv else None
    if alvo is None or alvo.is_dir():
        alvo = ultimo_arquivo(str(alvo) if alvo else PASTA_PADRAO)
    if alvo is None or not alvo.exists():
        print(f"nenhum registro encontrado em {argv[0] if argv else PASTA_PADRAO}")
        return
    print(f"# {alvo}")
    print(formatar(ler(alvo)))
