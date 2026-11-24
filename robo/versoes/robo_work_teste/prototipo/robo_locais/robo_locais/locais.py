"""Modelo dos locais de interesse, sem dependencia de ROS.

Um local tem um nome, um tipo, uma pose de aproximacao no frame ``map``
(onde o robo deve parar e para onde deve estar virado) e, opcionalmente,
um poligono delimitando a area. Os locais sao salvos em YAML junto com o
mapa, pois as coordenadas so valem para o mapa em que foram marcadas.

Dois campos descrevem o objetivo da prova sem precisar escrever a missao:
``objetos`` (o que ha para pegar no local) e ``recebe`` (o que deve ser
entregue nele). O planejador (planejador.py) monta a missao a partir deles.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

# Tipos previstos para a @Work. Nao e uma lista fechada: um tipo
# desconhecido so gera um aviso, para nao travar a marcacao em campo.
TIPOS_CONHECIDOS = (
    "alinhamento",  # ponto em que o robo se alinha com um objeto/mesa
    "coleta",       # area de onde um objeto deve ser pego
    "entrega",      # area/ponto onde um objeto deve ser deixado
    "passagem",     # ponto intermediario de navegacao
    "inicio",       # posicao de partida/chegada
)

# Prefixo do nome automatico de cada tipo (C1, C2, E1...). Ponto dentro de
# uma area usa o nome da area: Y1, Y2...
PREFIXOS = {
    "alinhamento": "A",
    "coleta": "C",
    "entrega": "E",
    "passagem": "P",
    "inicio": "inicio",
}


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Local:
    nome: str
    tipo: str
    pose: Pose2D
    area: List[Tuple[float, float]] = field(default_factory=list)
    area_pai: Optional[str] = None  # ex.: ponto Z dentro da area Y
    descricao: str = ""
    # Nomes repetidos sao unidades diferentes: [caixa, caixa] = 2 caixas.
    objetos: List[str] = field(default_factory=list)  # o que ha para pegar
    recebe: List[str] = field(default_factory=list)   # o que deve ser entregue aqui
    # Bancada: os 2 extremos da borda frontal. A pose e calculada a partir
    # dela (ver aproximacao_da_bancada), igual ao robo_station_manager.
    borda: List[Tuple[float, float]] = field(default_factory=list)

    def ponto_dentro_da_area(self, x: float, y: float) -> bool:
        """Ray casting no poligono; sem poligono, nunca contem o ponto."""
        if len(self.area) < 3:
            return False
        dentro = False
        j = len(self.area) - 1
        for i, (xi, yi) in enumerate(self.area):
            xj, yj = self.area[j]
            if (yi > y) != (yj > y):
                x_cruzamento = (xj - xi) * (y - yi) / (yj - yi) + xi
                if x < x_cruzamento:
                    dentro = not dentro
            j = i
        return dentro


# Geometria do Robo-WORK (docs/geometria.md e robo_station_manager):
# frente do robo a +0,365 m do base_link e folga de 0,20 m ate a bancada.
FRENTE_ROBO_M = 0.365
FOLGA_BANCADA_M = 0.20


def aproximacao_da_bancada(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    lado_livre: Tuple[float, float],
    frente_robo: float = FRENTE_ROBO_M,
    folga: float = FOLGA_BANCADA_M,
) -> Pose2D:
    """Pose do base_link diante da bancada, com +X apontando para ela.

    Mesma regra do robo_station_manager (geometry.compute_approach): a
    bancada e a borda frontal p1-p2, e lado_livre e um ponto no espaco livre
    na frente dela, que escolhe de qual lado o robo para.
    """
    (x1, y1), (x2, y2), (sx, sy) = p1, p2, lado_livre
    largura = math.hypot(x2 - x1, y2 - y1)
    if largura < 0.25:
        raise ValueError("borda frontal pequena demais (menos de 25 cm)")
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    nx, ny = -(y2 - y1) / largura, (x2 - x1) / largura
    lado = (sx - cx) * nx + (sy - cy) * ny
    if abs(lado) < 0.10:
        raise ValueError("o ponto do lado livre deve ficar a 10 cm ou mais da borda")
    if lado < 0:
        nx, ny = -nx, -ny
    distancia = frente_robo + folga
    return Pose2D(cx + nx * distancia, cy + ny * distancia, math.atan2(-ny, -nx))


def normalizar_angulo(angulo: float) -> float:
    return math.atan2(math.sin(angulo), math.cos(angulo))


def yaw_de_quaternion(z: float, w: float) -> float:
    """Yaw de um quaternion puramente em torno de Z (caso 2D)."""
    return normalizar_angulo(2.0 * math.atan2(z, w))


class MapaDeLocais:
    """Colecao de locais com leitura e escrita em YAML."""

    def __init__(self, frame_id: str = "map", mapa: str = "") -> None:
        self.frame_id = frame_id
        self.mapa = mapa  # nome do mapa do SLAM ao qual os locais pertencem
        self.locais: Dict[str, Local] = {}

    def definir(self, local: Local) -> None:
        if not local.nome:
            raise ValueError("local sem nome")
        if local.area_pai is not None and local.area_pai == local.nome:
            raise ValueError("um local nao pode ser area_pai de si mesmo")
        self.locais[local.nome] = local

    def proximo_nome(self, tipo: str, area_pai: Optional[str] = None) -> str:
        """Primeiro nome livre para um local novo: Y1, Y2... ou C1, E1, P1..."""
        prefixo = area_pai or PREFIXOS.get(tipo, "L")
        if prefixo == "inicio" and prefixo not in self.locais:
            return prefixo  # o planejador procura um local chamado/tipo inicio
        n = 1
        while f"{prefixo}{n}" in self.locais:
            n += 1
        return f"{prefixo}{n}"

    def remover(self, nome: str) -> bool:
        return self.locais.pop(nome, None) is not None

    def obter(self, nome: str) -> Local:
        try:
            return self.locais[nome]
        except KeyError:
            raise KeyError(
                f"local '{nome}' nao existe; conhecidos: {sorted(self.locais)}"
            ) from None

    def adicionar_vertice(self, nome: str, x: float, y: float) -> None:
        self.obter(nome).area.append((x, y))

    def locais_que_contem(self, x: float, y: float) -> List[str]:
        return [n for n, l in self.locais.items() if l.ponto_dentro_da_area(x, y)]

    def filhos(self, nome: str) -> List[str]:
        """Pontos marcados dentro de uma area (area_pai == nome)."""
        return [n for n, l in self.locais.items() if l.area_pai == nome]

    # ---------- YAML ----------

    def para_dict(self) -> dict:
        locais = {}
        for nome, local in self.locais.items():
            item = {
                "tipo": local.tipo,
                "pose": {
                    "x": round(local.pose.x, 3),
                    "y": round(local.pose.y, 3),
                    "yaw": round(local.pose.yaw, 3),
                },
            }
            if local.area:
                item["area"] = [[round(x, 3), round(y, 3)] for x, y in local.area]
            if local.area_pai:
                item["area_pai"] = local.area_pai
            if local.descricao:
                item["descricao"] = local.descricao
            if local.objetos:
                item["objetos"] = list(local.objetos)
            if local.recebe:
                item["recebe"] = list(local.recebe)
            if local.borda:
                item["borda"] = [[round(x, 3), round(y, 3)] for x, y in local.borda]
            locais[nome] = item
        return {"frame_id": self.frame_id, "mapa": self.mapa, "locais": locais}

    @classmethod
    def de_dict(cls, dados: dict) -> "MapaDeLocais":
        dados = dados or {}
        mapa = cls(dados.get("frame_id", "map"), dados.get("mapa", ""))
        for nome, item in (dados.get("locais") or {}).items():
            pose = item.get("pose") or {}
            mapa.definir(
                Local(
                    nome=str(nome),
                    tipo=str(item.get("tipo", "passagem")),
                    pose=Pose2D(
                        float(pose.get("x", 0.0)),
                        float(pose.get("y", 0.0)),
                        float(pose.get("yaw", 0.0)),
                    ),
                    area=[(float(x), float(y)) for x, y in item.get("area", [])],
                    area_pai=item.get("area_pai"),
                    descricao=str(item.get("descricao", "")),
                    objetos=[str(o) for o in item.get("objetos") or []],
                    recebe=[str(o) for o in item.get("recebe") or []],
                    borda=[(float(x), float(y)) for x, y in item.get("borda") or []],
                )
            )
        return mapa

    def salvar(self, caminho: str) -> None:
        destino = Path(caminho).expanduser()
        destino.parent.mkdir(parents=True, exist_ok=True)
        with destino.open("w", encoding="utf-8") as arquivo:
            yaml.safe_dump(
                self.para_dict(), arquivo, allow_unicode=True, sort_keys=False
            )

    @classmethod
    def carregar(cls, caminho: str) -> "MapaDeLocais":
        origem = Path(caminho).expanduser()
        if not origem.exists():
            return cls()
        with origem.open(encoding="utf-8") as arquivo:
            return cls.de_dict(yaml.safe_load(arquivo))
