"""Planejador de objetivos, sem dependencia de ROS.

Em vez de escrever a missao passo a passo, a equipe diz O QUE deve ser
feito e o planejador decide COMO:

    objetivo: "caixa_azul vai para Z, duas caixas vao para a area Y"
        |
        |  resolver(): de onde pegar (locais com 'objetos') e onde soltar
        |              (o ponto, ou os pontos marcados dentro da area)
        v
        |  planejar(): ordem de menor caminho, respeitando quantos objetos
        |              o robo carrega de uma vez (capacidade)
        v
    passos ir_para / alinhar / pegar / soltar -> ExecutorMissao

O objetivo vem de uma de duas fontes:
  1. das anotacoes feitas no mapeamento: 'recebe' nos locais de entrega e
     'objetos' nos de coleta. Terminou de mapear, ja tem objetivo;
  2. de um YAML de objetivo (ex.: tarefa recebida da RefBox):

       objetivo:
         terminar_em: inicio            # opcional; padrao = local tipo inicio
         entregar:
           - {objeto: caixa_azul, para: Z}
           - {objeto: caixa, para: Y, quantidade: 2}   # Y e area: usa os
           - {objeto: M20, de: X, para: W}             # pontos dentro dela

Na execucao, cada falha (nao chegou, nao achou o objeto, nao conseguiu
soltar) atualiza o que se sabe da arena e o plano e refeito com o que
sobrou, ate entregar tudo ou nao haver mais o que tentar.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple

import yaml

from robo_locais.locais import MapaDeLocais, Pose2D
from robo_locais.missao import (
    Acoes,
    AoEvento,
    ExecutorMissao,
    Navegador,
    Passo,
    ResultadoPasso,
    _ignorar,
)

Inventario = Dict[str, Counter]  # local -> {objeto: unidades}


# ---------- Objetivo ----------


@dataclass
class Entrega:
    objeto: str
    para: str                 # ponto ou area de destino
    de: Optional[str] = None  # None: procura nos locais com 'objetos'


@dataclass
class Objetivo:
    entregas: List[Entrega] = field(default_factory=list)
    terminar_em: Optional[str] = None


def local_de_inicio(mapa: MapaDeLocais) -> Optional[str]:
    return next((n for n, l in mapa.locais.items() if l.tipo == "inicio"), None)


def objetivo_dos_locais(mapa: MapaDeLocais) -> Objetivo:
    """Objetivo a partir das anotacoes 'recebe' feitas no mapeamento."""
    entregas = [
        Entrega(objeto, nome)
        for nome, local in mapa.locais.items()
        for objeto in local.recebe
    ]
    return Objetivo(entregas, local_de_inicio(mapa))


def objetivo_de_dict(dados: dict, mapa: MapaDeLocais) -> Objetivo:
    dados = (dados or {}).get("objetivo") or {}
    entregas = []
    for item in dados.get("entregar") or []:
        for _ in range(int(item.get("quantidade", 1))):
            de = item.get("de")
            entregas.append(
                Entrega(str(item["objeto"]), str(item["para"]), str(de) if de else None)
            )
    terminar_em = dados.get("terminar_em", local_de_inicio(mapa))
    return Objetivo(entregas, str(terminar_em) if terminar_em else None)


def obter_objetivo(mapa: MapaDeLocais, arquivo_objetivo: str = "") -> Objetivo:
    """YAML de objetivo se informado; senao, as anotacoes dos locais."""
    if not arquivo_objetivo:
        return objetivo_dos_locais(mapa)
    with Path(arquivo_objetivo).expanduser().open(encoding="utf-8") as arquivo:
        return objetivo_de_dict(yaml.safe_load(arquivo), mapa)


# ---------- Resolucao: objetivo -> tarefas ----------


@dataclass
class Tarefa:
    id: int
    objeto: str
    origens: List[str]   # locais candidatos para pegar
    destinos: List[str]  # locais candidatos para soltar

    @property
    def descricao(self) -> str:
        return f"{self.objeto} -> {'/'.join(self.destinos)}"


def _pontos_da_area(mapa: MapaDeLocais, nome: str) -> List[str]:
    """Uma area com pontos marcados dentro dela vale por esses pontos."""
    return mapa.filhos(nome) or [nome]


def resolver(
    objetivo: Objetivo, mapa: MapaDeLocais
) -> Tuple[List[Tarefa], Inventario, List[str]]:
    """Transforma entregas em tarefas com origens e destinos candidatos.

    Devolve (tarefas, inventario, problemas). Um problema nao impede as
    outras tarefas: o robo entrega o que der.
    """
    problemas: List[str] = []
    inventario: Inventario = {
        n: Counter(l.objetos) for n, l in mapa.locais.items() if l.objetos
    }
    # 'de' explicito num local sem anotacao: confia no objetivo, 1 unidade por entrega.
    confiados: Set[Tuple[str, str]] = set()
    tarefas: List[Tarefa] = []

    for entrega in objetivo.entregas:
        nome = f"{entrega.objeto} -> {entrega.para}"
        if entrega.para not in mapa.locais:
            problemas.append(f"{nome}: destino '{entrega.para}' nao foi marcado")
            continue
        destinos = _pontos_da_area(mapa, entrega.para)

        if entrega.de is None:
            origens = [n for n, inv in inventario.items() if inv[entrega.objeto] > 0]
        elif entrega.de not in mapa.locais:
            problemas.append(f"{nome}: origem '{entrega.de}' nao foi marcada")
            continue
        else:
            candidatos = _pontos_da_area(mapa, entrega.de)
            origens = [
                o for o in candidatos
                if inventario.get(o, Counter())[entrega.objeto] > 0
                and (o, entrega.objeto) not in confiados
            ]
            if not origens or (entrega.de, entrega.objeto) in confiados:
                inventario.setdefault(entrega.de, Counter())[entrega.objeto] += 1
                confiados.add((entrega.de, entrega.objeto))
                origens = [entrega.de]

        if not origens:
            problemas.append(
                f"{nome}: nenhum local tem '{entrega.objeto}' em 'objetos'"
            )
            continue
        tarefas.append(Tarefa(len(tarefas), entrega.objeto, origens, destinos))

    for objeto in sorted({t.objeto for t in tarefas}):
        pedidas = sum(t.objeto == objeto for t in tarefas)
        anotadas = sum(inv[objeto] for inv in inventario.values())
        if pedidas > anotadas:
            problemas.append(
                f"'{objeto}': {pedidas} entregas, mas so {anotadas} unidade(s) "
                "anotada(s) nos locais"
            )
    return tarefas, inventario, problemas


# ---------- Planejamento: tarefas -> ordem das acoes ----------


@dataclass(frozen=True)
class AcaoPlano:
    tipo: str  # "pegar" ou "soltar"
    local: str
    tarefa: int
    objeto: str


@dataclass
class EstadoRobo:
    """O que o planejador precisa saber do momento atual."""

    inventario: Inventario
    carregando: Set[int] = field(default_factory=set)  # ids de tarefas
    ocupados: Set[str] = field(default_factory=set)     # destinos ja usados
    local_atual: Optional[str] = None  # None: esta em pose_atual
    pose_atual: Optional[Pose2D] = None


def planejar(
    tarefas: List[Tarefa],
    estado: EstadoRobo,
    mapa: MapaDeLocais,
    custo: Callable[[Pose2D, Pose2D], float],
    capacidade: int = 1,
    terminar_em: Optional[str] = None,
    limite_nos: int = 50_000,
) -> Tuple[List[AcaoPlano], float]:
    """Busca a sequencia de pegar/soltar que entrega mais tarefas com menor custo.

    Busca em profundidade com poda (branch and bound). As opcoes sao
    tentadas da mais perto para a mais longe, entao a primeira solucao ja e
    a gulosa ("vai no mais perto"); o resto da busca so melhora. Com muitas
    tarefas, o limite de nos devolve a melhor encontrada ate ali.
    """
    por_id = {t.id: t for t in tarefas}
    cache: Dict[Tuple[Optional[str], str], float] = {}

    def c(origem: Optional[str], destino: str) -> float:
        if origem == destino:
            return 0.0
        if origem is None and estado.pose_atual is None:
            return 0.0  # posicao desconhecida: nao penaliza o primeiro trecho
        if (origem, destino) not in cache:
            pose = estado.pose_atual if origem is None else mapa.obter(origem).pose
            cache[(origem, destino)] = custo(pose, mapa.obter(destino).pose)
        return cache[(origem, destino)]

    def opcoes(pos, pendentes, carregando, inventario, ocupados) -> List[AcaoPlano]:
        ops: List[AcaoPlano] = []
        # Tarefas iguais (mesmo objeto, origens e destinos) sao intercambiaveis:
        # so a de menor id entra, para a busca nao repetir a mesma solucao.
        vistos = set()
        if len(carregando) < capacidade:
            for tid in sorted(pendentes):
                t = por_id[tid]
                chave = (t.objeto, tuple(t.origens), tuple(t.destinos))
                if chave in vistos:
                    continue
                vistos.add(chave)
                ops += [
                    AcaoPlano("pegar", o, tid, t.objeto)
                    for o in t.origens
                    if inventario.get(o, Counter())[t.objeto] > 0
                ]
        vistos = set()
        for tid in sorted(carregando):
            t = por_id[tid]
            chave = (t.objeto, tuple(t.destinos))
            if chave in vistos:
                continue
            vistos.add(chave)
            # Espalha objetos pelos pontos livres de uma area antes de repetir.
            livres = [d for d in t.destinos if d not in ocupados] or t.destinos
            ops += [AcaoPlano("soltar", d, tid, t.objeto) for d in livres]
        return sorted(ops, key=lambda a: c(pos, a.local))

    melhor: Dict[str, object] = {"chave": (math.inf, math.inf), "acoes": []}
    nos = 0

    def busca(pos, gasto, acoes, pendentes, carregando, inventario, ocupados, entregues):
        nonlocal nos
        nos += 1
        # Otimista: tudo que falta seria entregue sem custo extra.
        if (-(entregues + len(pendentes) + len(carregando)), gasto) >= melhor["chave"]:
            return
        ops = opcoes(pos, pendentes, carregando, inventario, ocupados)
        if not ops:
            total = gasto + (c(pos, terminar_em) if terminar_em else 0.0)
            if (-entregues, total) < melhor["chave"]:
                melhor["chave"], melhor["acoes"] = (-entregues, total), list(acoes)
            return
        for acao in ops:
            if nos > limite_nos and melhor["acoes"]:
                return
            acoes.append(acao)
            passo = gasto + c(pos, acao.local)
            if acao.tipo == "pegar":
                inv = dict(inventario)
                inv[acao.local] = inventario[acao.local].copy()
                inv[acao.local][acao.objeto] -= 1
                busca(acao.local, passo, acoes, pendentes - {acao.tarefa},
                      carregando | {acao.tarefa}, inv, ocupados, entregues)
            else:
                busca(acao.local, passo, acoes, pendentes, carregando - {acao.tarefa},
                      inventario, ocupados | {acao.local}, entregues + 1)
            acoes.pop()

    pendentes = frozenset(t.id for t in tarefas if t.id not in estado.carregando)
    carregando = frozenset(t.id for t in tarefas if t.id in estado.carregando)
    busca(estado.local_atual, 0.0, [], pendentes, carregando,
          estado.inventario, frozenset(estado.ocupados), 0)
    return list(melhor["acoes"]), float(melhor["chave"][1])


def passos_da_acao(acao: AcaoPlano, local_atual: Optional[str]) -> List[Passo]:
    passos = []
    if acao.local != local_atual:
        passos += [Passo("ir_para", acao.local), Passo("alinhar", acao.local)]
    passos.append(Passo(acao.tipo, acao.local, acao.objeto))
    return passos


def passos_do_plano(
    acoes: List[AcaoPlano],
    local_atual: Optional[str] = None,
    terminar_em: Optional[str] = None,
) -> List[Passo]:
    passos: List[Passo] = []
    for acao in acoes:
        passos += passos_da_acao(acao, local_atual)
        local_atual = acao.local
    if terminar_em and terminar_em != local_atual:
        passos.append(Passo("ir_para", terminar_em))
    return passos


# ---------- Execucao autonoma com replanejamento ----------


@dataclass
class Plano:
    tarefas: List[Tarefa]
    acoes: List[AcaoPlano]
    passos: List[Passo]
    custo: float
    problemas: List[str]

    def resumo(self) -> str:
        linhas = [f"{len(self.tarefas)} entrega(s), custo estimado {self.custo:.1f}"]
        linhas += [f"  {i + 1}. {p.acao} {p.local}" + (f" [{p.objeto}]" if p.objeto else "")
                   for i, p in enumerate(self.passos)]
        linhas += [f"  PROBLEMA: {p}" for p in self.problemas]
        return "\n".join(linhas)


@dataclass
class RelatorioObjetivo:
    entregues: List[str] = field(default_factory=list)
    abandonadas: List[str] = field(default_factory=list)
    passos: List[ResultadoPasso] = field(default_factory=list)

    @property
    def sucesso(self) -> bool:
        return not self.abandonadas


class ExecutorObjetivo:
    """Cumpre um Objetivo sozinho: planeja, executa e replaneja nas falhas."""

    def __init__(
        self,
        mapa: MapaDeLocais,
        navegador: Navegador,
        acoes: Acoes,
        log: Callable[[str], None] = print,
        capacidade: int = 1,
        max_tentativas: int = 2,
        max_replanejamentos: int = 20,
        missao: Optional[ExecutorMissao] = None,
        ao_evento: AoEvento = _ignorar,
    ) -> None:
        self.mapa = mapa
        self.navegador = navegador
        self.log = log
        self.capacidade = capacidade
        self.max_tentativas = max_tentativas
        self.max_replanejamentos = max_replanejamentos
        self.ao_evento = ao_evento
        # Reaproveita o executor de missao: acoes registradas nele valem aqui.
        self.missao = missao or ExecutorMissao(mapa, navegador, acoes, log, ao_evento)

    def _custo(self, origem: Pose2D, destino: Pose2D) -> float:
        return self.navegador.custo(origem, destino, self.mapa.frame_id)

    def _pose_inicial(self) -> Optional[Pose2D]:
        pose = self.navegador.pose_atual(self.mapa.frame_id)
        if pose is None and local_de_inicio(self.mapa):
            pose = self.mapa.obter(local_de_inicio(self.mapa)).pose
        return pose

    def planejar(self, objetivo: Objetivo, pose_inicial: Optional[Pose2D] = None) -> Plano:
        """So planeja, sem mover o robo. Para conferir antes de rodar."""
        tarefas, inventario, problemas = resolver(objetivo, self.mapa)
        if not objetivo.entregas:
            problemas.append("nenhuma entrega definida (anote 'recebe' nos locais "
                             "de entrega ou informe um arquivo de objetivo)")
        if objetivo.terminar_em and objetivo.terminar_em not in self.mapa.locais:
            problemas.append(f"terminar_em '{objetivo.terminar_em}' nao foi marcado")
        estado = EstadoRobo(inventario, pose_atual=pose_inicial or self._pose_inicial())
        acoes, custo = planejar(tarefas, estado, self.mapa, self._custo,
                                self.capacidade, objetivo.terminar_em)
        nao_planejadas = {t.id for t in tarefas} - {a.tarefa for a in acoes if a.tipo == "soltar"}
        problemas += [f"sem plano para {t.descricao}" for t in tarefas if t.id in nao_planejadas]
        return Plano(tarefas, acoes, passos_do_plano(acoes, None, objetivo.terminar_em),
                     custo, problemas)

    def executar(self, objetivo: Objetivo) -> RelatorioObjetivo:
        rel = RelatorioObjetivo()
        tarefas, inventario, problemas = resolver(objetivo, self.mapa)
        for problema in problemas:
            self.log("AVISO " + problema)
        estado = EstadoRobo(inventario, pose_atual=self._pose_inicial())
        ativas = {t.id: t for t in tarefas}
        bloqueados: Set[str] = set()
        falhas: Counter = Counter()
        presos = 0  # objetos abandonados que ficaram na garra

        def abandonar(t: Tarefa, motivo: str) -> None:
            nonlocal presos
            del ativas[t.id]
            if t.id in estado.carregando:
                estado.carregando.discard(t.id)
                presos += 1
            rel.abandonadas.append(f"{t.descricao}: {motivo}")
            self.log(f"ABANDONADA {t.descricao}: {motivo}")
            self.ao_evento("abandonada", {"objeto": t.objeto, "destinos": t.destinos,
                                          "motivo": motivo})

        for rodada in range(self.max_replanejamentos):
            for t in list(ativas.values()):
                t.origens = [o for o in t.origens if o not in bloqueados]
                t.destinos = [d for d in t.destinos if d not in bloqueados]
                if not t.destinos:
                    abandonar(t, "nenhum destino alcancavel")
                elif t.id not in estado.carregando and not any(
                    estado.inventario.get(o, Counter())[t.objeto] > 0 for o in t.origens
                ):
                    abandonar(t, "objeto nao encontrado em nenhum local")
            if not ativas:
                break

            acoes, custo = planejar(list(ativas.values()), estado, self.mapa, self._custo,
                                    max(self.capacidade - presos, 0), objetivo.terminar_em)
            if not acoes:
                for t in list(ativas.values()):
                    abandonar(t, "sem plano possivel")
                break
            self.log(f"plano {rodada + 1}: {len(acoes)} acoes, custo {custo:.1f}")
            self.ao_evento("plano", {
                "rodada": rodada + 1, "custo": round(custo, 2),
                "acoes": [f"{a.tipo} {a.objeto} em {a.local}" for a in acoes],
            })

            for acao in acoes:
                falhou = None
                for passo in passos_da_acao(acao, estado.local_atual):
                    resultado = self.missao.executar([passo])
                    rel.passos += resultado.passos
                    if not resultado.sucesso:
                        falhou = passo
                        break
                if falhou is None:
                    self._sucesso(acao, estado, ativas, rel)
                    continue
                self._falha(acao, falhou, estado, bloqueados, falhas)
                break  # replaneja com o que se sabe agora

        for t in list(ativas.values()):
            abandonar(t, f"desistiu apos {self.max_replanejamentos} replanejamentos")

        fim = objetivo.terminar_em
        if fim and fim != estado.local_atual and fim in self.mapa.locais:
            rel.passos += self.missao.executar([Passo("ir_para", fim)]).passos

        self.log(f"objetivo {'CONCLUIDO' if rel.sucesso else 'INCOMPLETO'}: "
                 f"{len(rel.entregues)} entregue(s), {len(rel.abandonadas)} abandonada(s)")
        return rel

    def _sucesso(self, acao, estado, ativas, rel) -> None:
        estado.local_atual = acao.local
        if acao.tipo == "pegar":
            estado.inventario[acao.local][acao.objeto] -= 1
            estado.carregando.add(acao.tarefa)
        else:
            estado.carregando.discard(acao.tarefa)
            estado.ocupados.add(acao.local)
            tarefa = ativas.pop(acao.tarefa)
            rel.entregues.append(f"{tarefa.objeto} em {acao.local}")
            self.ao_evento("entregue", {"objeto": tarefa.objeto, "local": acao.local})

    def _falha(self, acao, passo, estado, bloqueados, falhas) -> None:
        # Depois de uma falha, a proxima tentativa sempre navega e realinha.
        estado.local_atual = None
        estado.pose_atual = self.navegador.pose_atual(self.mapa.frame_id) or estado.pose_atual
        if passo.acao == "pegar":
            chave = (acao.local, acao.objeto)
            falhas[chave] += 1
            if falhas[chave] >= self.max_tentativas:
                estado.inventario[acao.local][acao.objeto] = 0
                self.log(f"'{acao.objeto}' nao esta em '{acao.local}', procurando em outro local")
                self.ao_evento("objeto_ausente", {"objeto": acao.objeto, "local": acao.local,
                                                  "tentativas": falhas[chave]})
        else:
            falhas[acao.local] += 1
            if falhas[acao.local] >= self.max_tentativas:
                bloqueados.add(acao.local)
                self.log(f"local '{acao.local}' bloqueado apos {falhas[acao.local]} falhas")
                self.ao_evento("local_bloqueado", {"local": acao.local,
                                                   "falhas": falhas[acao.local],
                                                   "ultima_acao": passo.acao})
