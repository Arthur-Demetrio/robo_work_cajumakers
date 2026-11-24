"""Registro dos testes de missao, sem ROS.

Cada missao (autonomo_node ou missao_node) vira um arquivo JSONL (um evento
JSON por linha, gravado na hora) em ~/robo_testes_prototipos/missao/. Se o
no cair ou for interrompido, tudo ate ali ja esta no disco.

O que importa num teste de missao:
  - contexto: arquivos de locais/objetivo, mapa, modo, capacidade, se o
    movimento estava permitido, o que disparou a missao;
  - plano inicial: entregas, passos, custo e problemas encontrados;
  - execucao: cada passo com duracao e resultado, cada replanejamento e o
    motivo (local bloqueado, objeto ausente), pausas por emergencia e
    timeouts do Nav2;
  - resultado: o que foi entregue, o que foi abandonado e por que, tempos.

Leitura: `ros2 run robo_locais ver_registro` (ultima missao) ou
`ros2 run robo_locais ver_registro <arquivo.jsonl>`.
"""

from __future__ import annotations

import json
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional

PASTA_PADRAO = "~/robo_testes_prototipos/missao"


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
        self._arquivo.write(json.dumps(linha, ensure_ascii=False, default=str) + "\n")
        self._arquivo.flush()  # nao perde nada se o processo morrer

    def fechar(self) -> None:
        if not self._arquivo.closed:
            self._arquivo.close()


class RegistroMissao:
    """Recebe os eventos dos executores (ao_evento) e monta o resumo."""

    def __init__(self, arquivo: Optional[ArquivoJsonl], contexto: Dict) -> None:
        self.arquivo = arquivo
        self.relogio = arquivo.relogio if arquivo else time.time
        self.inicio = self.relogio()
        self.por_acao: Dict[str, Dict[str, float]] = defaultdict(
            lambda: {"vezes": 0, "falhas": 0, "tempo_s": 0.0}
        )
        self.contagem = defaultdict(int)
        self.tempo_pausado_s = 0.0
        self.fechado = False
        self._gravar("inicio", **contexto)

    def _gravar(self, evento: str, **dados) -> None:
        if self.arquivo is not None:
            self.arquivo.gravar(evento, **dados)

    def plano_inicial(self, plano, avisos: Optional[List[str]] = None) -> None:
        self._gravar(
            "plano_inicial",
            entregas=[t.descricao for t in plano.tarefas],
            custo=round(plano.custo, 2),
            passos=[f"{p.acao} {p.local}" + (f" [{p.objeto}]" if p.objeto else "")
                    for p in plano.passos],
            problemas=list(plano.problemas),
            avisos=list(avisos or []),
        )

    def evento(self, tipo: str, dados: Dict) -> None:
        """Gancho ao_evento dos executores e do navegador."""
        if tipo == "passo":
            acao = self.por_acao[dados["acao"]]
            acao["vezes"] += 1
            acao["tempo_s"] += dados.get("duracao_s", 0.0)
            if not dados.get("sucesso"):
                acao["falhas"] += 1
        elif tipo == "pausa_emergencia":
            self.tempo_pausado_s += dados.get("duracao_s", 0.0)
        self.contagem[tipo] += 1
        self._gravar(tipo, **dados)

    def erro(self, mensagem: str) -> None:
        self.contagem["erro"] += 1
        self._gravar("erro", mensagem=mensagem)

    def fechar(self, relatorio=None, situacao: str = "concluida") -> Dict:
        """situacao: concluida | incompleta | interrompida | nao_iniciada | erro."""
        if self.fechado:
            return {}
        self.fechado = True
        resumo = {
            "situacao": situacao,
            "duracao_s": round(self.relogio() - self.inicio, 1),
            "tempo_pausado_emergencia_s": round(self.tempo_pausado_s, 1),
            "replanejamentos": max(self.contagem["plano"] - 1, 0),
            "pausas_emergencia": self.contagem["pausa_emergencia"],
            "timeouts_navegacao": self.contagem["timeout"],
            "locais_bloqueados": self.contagem["local_bloqueado"],
            "objetos_ausentes": self.contagem["objeto_ausente"],
            "por_acao": {
                nome: {k: round(v, 1) if isinstance(v, float) else v for k, v in d.items()}
                for nome, d in sorted(self.por_acao.items())
            },
        }
        if relatorio is not None:
            resumo["entregues"] = list(relatorio.entregues)
            resumo["abandonadas"] = list(getattr(relatorio, "abandonadas", []))
        self._gravar("fim", **resumo)
        if self.arquivo is not None:
            self.arquivo.fechar()
        return resumo


# ---------- Leitura (ros2 run robo_locais ver_registro) ----------


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


def _quando(e: Dict) -> str:
    s = e.get("s")
    return f"[{s:>8.1f} s]" if isinstance(s, (int, float)) else "[       ? s]"


def formatar(eventos: List[Dict]) -> str:
    linhas = []
    for e in eventos:
        tipo = e.get("evento")
        dados = {k: v for k, v in e.items() if k not in ("t", "s", "evento")}
        if tipo == "inicio":
            linhas.append(f"MISSAO {e.get('t', '')}")
            linhas += [f"  {k}: {v}" for k, v in dados.items()]
        elif tipo == "plano_inicial":
            linhas.append(f"PLANO INICIAL: {len(dados.get('entregas', []))} entrega(s), "
                          f"custo {dados.get('custo')}")
            linhas += [f"  {i + 1}. {p}" for i, p in enumerate(dados.get("passos", []))]
            linhas += [f"  PROBLEMA: {p}" for p in dados.get("problemas", [])]
            linhas += [f"  AVISO: {a}" for a in dados.get("avisos", [])]
        elif tipo == "passo":
            marca = "ok    " if dados.get("sucesso") else "FALHOU"
            objeto = f" [{dados['objeto']}]" if dados.get("objeto") else ""
            extra = f" ({dados['mensagem']})" if dados.get("mensagem") else ""
            linhas.append(f"{_quando(e)} {marca} {dados.get('acao')} {dados.get('local')}"
                          f"{objeto} em {dados.get('duracao_s')} s{extra}")
        elif tipo == "plano":
            linhas.append(f"{_quando(e)} PLANO {dados.get('rodada')}: "
                          f"{len(dados.get('acoes', []))} acoes, custo {dados.get('custo')}")
        elif tipo == "fim":
            linhas.append(f"FIM: {str(dados.pop('situacao', '?')).upper()}")
            por_acao = dados.pop("por_acao", {})
            linhas += [f"  {k}: {v}" for k, v in dados.items()]
            for nome, d in por_acao.items():
                linhas.append(f"  {nome}: {d['vezes']}x, {d['falhas']} falha(s), "
                              f"{d['tempo_s']} s no total")
        else:
            detalhe = ", ".join(f"{k}={v}" for k, v in dados.items())
            linhas.append(f"{_quando(e)} {tipo}" + (f": {detalhe}" if detalhe else ""))
    if not eventos or eventos[-1].get("evento") != "fim":
        linhas.append("(SEM FIM: a missao nao terminou normalmente, ex.: o no caiu)")
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
