"""Executor de missao, sem dependencia de ROS.

Uma missao e uma lista de passos. Cada passo e uma acao aplicada a um local
do MapaDeLocais. O executor nao sabe COMO o robo anda ou pega objetos: isso
fica em implementacoes plugaveis (Navegador e Acoes). Assim o prototipo roda
em modo simulado hoje, e depois basta registrar implementacoes reais (Nav2,
visao, braco) sem mudar esta logica.

Formato do YAML de missao:

    missao:
      - {acao: ir_para, local: A}
      - {acao: alinhar, local: A}
      - {acao: transportar, objeto: M20, de: X, para: Z}

"transportar" e expandido em: ir_para(de), alinhar(de), pegar(de),
ir_para(para), alinhar(para), soltar(para).
"""

from __future__ import annotations

import abc
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import yaml

from robo_locais.locais import Local, MapaDeLocais, Pose2D


@dataclass
class Passo:
    acao: str
    local: str
    objeto: Optional[str] = None


@dataclass
class ResultadoPasso:
    passo: Passo
    sucesso: bool
    mensagem: str = ""


@dataclass
class ResultadoMissao:
    passos: List[ResultadoPasso] = field(default_factory=list)

    @property
    def sucesso(self) -> bool:
        return bool(self.passos) and all(r.sucesso for r in self.passos)


# Gancho de registro: recebe (tipo do evento, dados). Padrao: ignora.
AoEvento = Callable[[str, dict], None]


def _ignorar(_tipo: str, _dados: dict) -> None:
    pass


# ---------- Pontos de extensao ----------


class Navegador(abc.ABC):
    """Como o robo chega a uma pose. Implementacao real: Nav2."""

    @abc.abstractmethod
    def ir_para(self, pose: Pose2D, frame_id: str) -> bool:
        ...

    def custo(self, origem: Pose2D, destino: Pose2D, frame_id: str) -> float:
        """Custo de ir de uma pose a outra, usado pelo planejador.

        Padrao: distancia em linha reta. O Nav2 pode trocar pelo
        comprimento do caminho planejado, que desvia das paredes.
        """
        return math.hypot(destino.x - origem.x, destino.y - origem.y)

    def pose_atual(self, frame_id: str) -> Optional[Pose2D]:
        """Onde o robo esta agora; None se nao souber."""
        return None


class Acoes(abc.ABC):
    """O que o robo faz ao chegar. Implementacoes reais: visao + braco."""

    @abc.abstractmethod
    def alinhar(self, local: Local) -> bool:
        ...

    @abc.abstractmethod
    def pegar(self, local: Local, objeto: Optional[str]) -> bool:
        ...

    @abc.abstractmethod
    def soltar(self, local: Local, objeto: Optional[str]) -> bool:
        ...


class NavegadorSimulado(Navegador):
    def __init__(self, log: Callable[[str], None] = print) -> None:
        self.log = log
        self.historico: List[Pose2D] = []

    def ir_para(self, pose: Pose2D, frame_id: str) -> bool:
        self.historico.append(pose)
        self.log(f"[sim] ir_para x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f} ({frame_id})")
        return True

    def pose_atual(self, frame_id: str) -> Optional[Pose2D]:
        return self.historico[-1] if self.historico else None


class AcoesSimuladas(Acoes):
    def __init__(self, log: Callable[[str], None] = print, capacidade: int = 1) -> None:
        self.log = log
        self.capacidade = capacidade
        self.carregando: List[str] = []

    def alinhar(self, local: Local) -> bool:
        self.log(f"[sim] alinhar em '{local.nome}'")
        return True

    def pegar(self, local: Local, objeto: Optional[str]) -> bool:
        if len(self.carregando) >= self.capacidade:
            self.log(f"[sim] ja carregando {self.carregando}, nao pega '{objeto}'")
            return False
        self.carregando.append(objeto or "?")
        self.log(f"[sim] pegar '{self.carregando[-1]}' em '{local.nome}'")
        return True

    def soltar(self, local: Local, objeto: Optional[str]) -> bool:
        if not self.carregando:
            self.log(f"[sim] nada para soltar em '{local.nome}'")
            return False
        solto = objeto if objeto in self.carregando else self.carregando[-1]
        self.carregando.remove(solto)
        self.log(f"[sim] soltar '{solto}' em '{local.nome}'")
        return True


# ---------- Leitura da missao ----------


def expandir(itens: List[dict]) -> List[Passo]:
    passos: List[Passo] = []
    for item in itens:
        acao = str(item.get("acao", "")).strip()
        if acao == "transportar":
            objeto, de, para = item.get("objeto"), item["de"], item["para"]
            passos += [
                Passo("ir_para", de),
                Passo("alinhar", de),
                Passo("pegar", de, objeto),
                Passo("ir_para", para),
                Passo("alinhar", para),
                Passo("soltar", para, objeto),
            ]
        else:
            passos.append(Passo(acao, str(item["local"]), item.get("objeto")))
    return passos


def carregar_missao(caminho: str) -> List[Passo]:
    with Path(caminho).expanduser().open(encoding="utf-8") as arquivo:
        dados = yaml.safe_load(arquivo) or {}
    return expandir(dados.get("missao") or [])


# ---------- Executor ----------


class ExecutorMissao:
    def __init__(
        self,
        mapa: MapaDeLocais,
        navegador: Navegador,
        acoes: Acoes,
        log: Callable[[str], None] = print,
        ao_evento: AoEvento = _ignorar,
    ) -> None:
        self.mapa = mapa
        self.navegador = navegador
        self.acoes = acoes
        self.log = log
        self.ao_evento = ao_evento
        self._handlers: Dict[str, Callable[[Passo, Local], bool]] = {
            "ir_para": lambda p, l: self.navegador.ir_para(l.pose, self.mapa.frame_id),
            "alinhar": lambda p, l: self.acoes.alinhar(l),
            "pegar": lambda p, l: self.acoes.pegar(l, p.objeto),
            "soltar": lambda p, l: self.acoes.soltar(l, p.objeto),
        }

    def registrar_acao(self, nome: str, handler: Callable[[Passo, Local], bool]) -> None:
        """Extensao: novas acoes (ex.: 'escanear_mesa') sem mudar o executor."""
        self._handlers[nome] = handler

    def validar(self, passos: List[Passo]) -> List[str]:
        erros = []
        for i, passo in enumerate(passos):
            if passo.acao not in self._handlers:
                erros.append(f"passo {i}: acao desconhecida '{passo.acao}'")
            if passo.local not in self.mapa.locais:
                erros.append(f"passo {i}: local desconhecido '{passo.local}'")
        return erros

    def executar(self, passos: List[Passo], parar_na_falha: bool = True) -> ResultadoMissao:
        resultado = ResultadoMissao()
        erros = self.validar(passos)
        if erros:
            for erro in erros:
                self.log("ERRO " + erro)
            return resultado

        for i, passo in enumerate(passos):
            local = self.mapa.obter(passo.local)
            self.log(f"passo {i + 1}/{len(passos)}: {passo.acao} {passo.local}")
            inicio = time.monotonic()
            try:
                ok = bool(self._handlers[passo.acao](passo, local))
                msg = "" if ok else "acao retornou falha"
            except Exception as erro:  # prototipo: nao derruba a missao inteira
                ok, msg = False, f"excecao: {erro}"
            resultado.passos.append(ResultadoPasso(passo, ok, msg))
            self.ao_evento("passo", {
                "acao": passo.acao, "local": passo.local, "objeto": passo.objeto,
                "sucesso": ok, "mensagem": msg,
                "duracao_s": round(time.monotonic() - inicio, 2),
            })
            if not ok:
                self.log(f"FALHA no passo {i + 1}: {msg}")
                if parar_na_falha:
                    break
        return resultado
