"""PROTOTIPO: no para marcar locais de interesse durante o mapeamento.

Fluxo de uso (ver README):
  1. Escolhe o tipo (e, se quiser, o nome) do proximo local:
       ros2 param set /locais tipo_atual coleta
       ros2 param set /locais nome_atual X      # opcional
     Sem nome, ele e gerado: C1, E1, P1... ou Y1, Y2... dentro da area Y.
     nome_atual, objetos_atual e recebe_atual valem para UMA marcacao e
     depois sao limpos, para o proximo botao Y nao sobrescrever o local.
  2. Marca a pose de uma destas formas:
       - leva o robo ate o ponto e chama ~/salvar_pose_atual (mais preciso);
       - clica no RViz com o "2D Goal Pose" apontando para /locais/marcar_pose.
  3. Opcional: define a area clicando com o "Publish Point" do RViz
     (/clicked_point); cada clique vira um vertice do poligono do ultimo
     local marcado (ou do nome_atual). ~/limpar_area recomeca o poligono.
  4. Opcional: anota o objetivo no proprio local (separado por virgula):
       ros2 param set /locais objetos_atual "caixa,caixa"   # o que ha aqui
       ros2 param set /locais recebe_atual "caixa_azul"     # o que vem para ca
     Vale na proxima pose marcada, ou na hora com ~/atualizar_atributos.
     "-" apaga a lista.
  Bancadas: ~/marcar_bancada e depois 3 cliques do Publish Point (2 extremos
  da borda frontal e 1 ponto no lado livre), como no robo_station_manager;
  a pose de aproximacao e calculada. ~/importar_bancadas le o cadastro do
  station_manager (~/.ros/robo_station_manager/stations_teste.json).
  5. Terminou de mapear: ~/finalizar_mapeamento salva locais e mapa e, se o
     objetivo tiver plano, avisa em /locais/mapeamento_concluido (o
     autonomo_node escuta e comeca a missao sozinho).

Os locais sao desenhados no RViz em /locais/marcadores e salvos em YAML.
Ao planejar/finalizar, a pegada do robo em cada local usado e conferida no
/global_costmap/costmap do Nav2 (se estiver rodando).

Alvo: sistema Robo-WORK (~/robo_jazzy_ws): frames map/base_link, SLAM e
Nav2 rodando juntos (terminais 04 e 05).
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robo_locais.locais import (
    TIPOS_CONHECIDOS,
    Local,
    MapaDeLocais,
    Pose2D,
    aproximacao_da_bancada,
    yaw_de_quaternion,
)
from robo_locais.pegada import Grade, Pegada, pegada_livre
from robo_locais.missao import AcoesSimuladas, NavegadorSimulado
from robo_locais.planejador import ExecutorObjetivo, obter_objetivo

# Cor (r, g, b) de cada tipo nos marcadores do RViz.
CORES = {
    "alinhamento": (1.0, 0.8, 0.0),
    "coleta": (0.1, 0.8, 0.2),
    "entrega": (0.2, 0.5, 1.0),
    "passagem": (0.7, 0.7, 0.7),
    "inicio": (1.0, 1.0, 1.0),
}


class LocaisNode(Node):
    def __init__(self) -> None:
        super().__init__("locais")

        self.declare_parameter("arquivo", "~/robo_mapas/arena_locais.yaml")
        self.declare_parameter("mapa", "arena")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("nome_atual", "")
        self.declare_parameter("tipo_atual", "passagem")
        self.declare_parameter("area_pai_atual", "")
        self.declare_parameter("objetos_atual", "")
        self.declare_parameter("recebe_atual", "")
        self.declare_parameter("auto_salvar", True)
        # Usados so para conferir o plano ao finalizar o mapeamento.
        self.declare_parameter("arquivo_objetivo", "")
        self.declare_parameter("capacidade", 1)
        # Vazio desliga. Salva o mapa do slam_toolbox ao lado dos locais.
        self.declare_parameter("servico_salvar_mapa", "/slam_toolbox/serialize_map")
        # Bancadas (mesma geometria do robo_station_manager).
        self.declare_parameter("frente_robo_m", 0.365)
        self.declare_parameter("folga_bancada_m", 0.20)
        self.declare_parameter(
            "arquivo_bancadas", "~/.ros/robo_station_manager/stations_teste.json"
        )
        # Conferencia da pegada no costmap global ao planejar/finalizar.
        self.declare_parameter("conferir_pegada", True)
        self.declare_parameter("topico_costmap", "/global_costmap/costmap")
        self.declare_parameter("margem_pegada_m", 0.05)

        self.arquivo = str(self.get_parameter("arquivo").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.mapa = MapaDeLocais.carregar(self.arquivo)
        self.mapa.frame_id = self.frame_id
        self.mapa.mapa = str(self.get_parameter("mapa").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Transient local: o RViz recebe os marcadores ao abrir, igual ao /map.
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marcadores_pub = self.create_publisher(
            MarkerArray, "/locais/marcadores", qos_latched
        )

        # Topico proprio: o /goal_pose padrao faria o Nav2 sair andando.
        self.create_subscription(
            PoseStamped, "/locais/marcar_pose", self.on_pose_clicada, 10
        )
        self.create_subscription(
            PointStamped, "/clicked_point", self.on_ponto_clicado, 10
        )
        # Volatil de proposito: um autonomo_node aberto depois nao deve sair
        # andando por causa de um aviso antigo.
        self.concluido_pub = self.create_publisher(
            String, "/locais/mapeamento_concluido", 10
        )
        self.cliente_mapa = None
        self.avisos: List[str] = []
        # Alvo de area/atributos/remover quando nome_atual esta vazio.
        self.ultimo_local: Optional[str] = None
        # Cliques da bancada em andamento (None = cliques viram vertices de area).
        self.cliques_bancada: Optional[List[tuple]] = None
        self.costmap: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("topico_costmap").value),
            self.on_costmap, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.create_service(Trigger, "~/salvar_pose_atual", self.srv_pose_atual)
        self.create_service(Trigger, "~/limpar_area", self.srv_limpar_area)
        self.create_service(Trigger, "~/remover", self.srv_remover)
        self.create_service(Trigger, "~/salvar_arquivo", self.srv_salvar)
        self.create_service(Trigger, "~/listar", self.srv_listar)
        self.create_service(Trigger, "~/atualizar_atributos", self.srv_atributos)
        self.create_service(Trigger, "~/planejar", self.srv_planejar)
        self.create_service(Trigger, "~/finalizar_mapeamento", self.srv_finalizar)
        self.create_service(Trigger, "~/marcar_bancada", self.srv_marcar_bancada)
        self.create_service(Trigger, "~/importar_bancadas", self.srv_importar_bancadas)

        self.publicar_marcadores()
        self.get_logger().info(
            f"{len(self.mapa.locais)} locais carregados de {self.arquivo}"
        )

    # ---------- Local atual ----------

    def _nome_tipo_pai(self):
        nome = str(self.get_parameter("nome_atual").value).strip()
        tipo = str(self.get_parameter("tipo_atual").value).strip()
        pai = str(self.get_parameter("area_pai_atual").value).strip() or None
        if tipo not in TIPOS_CONHECIDOS:
            self.get_logger().warn(
                f"tipo '{tipo}' fora da lista conhecida {TIPOS_CONHECIDOS}"
            )
        return nome, tipo, pai

    def _lista(self, parametro: str) -> Optional[List[str]]:
        """None = nao informado (mantem o que ja tinha); "-" = lista vazia."""
        texto = str(self.get_parameter(parametro).value).strip()
        if not texto:
            return None
        if texto == "-":
            return []
        return [item.strip() for item in texto.split(",") if item.strip()]

    def _alvo(self) -> Optional[str]:
        nome = str(self.get_parameter("nome_atual").value).strip()
        return nome or self.ultimo_local

    def _consumir(self, nome: str) -> None:
        """Nome e anotacoes valem uma vez: o proximo Y cria um local novo."""
        self.ultimo_local = nome
        self.set_parameters([
            Parameter("nome_atual", value=""),
            Parameter("objetos_atual", value=""),
            Parameter("recebe_atual", value=""),
        ])

    def _definir_pose(self, pose: Pose2D, borda=None) -> str:
        nome, tipo, pai = self._nome_tipo_pai()
        automatico = not nome
        if automatico:
            nome = self.mapa.proximo_nome(tipo, pai)
        anterior = self.mapa.locais.get(nome)
        objetos, recebe = self._lista("objetos_atual"), self._lista("recebe_atual")
        self.mapa.definir(
            Local(
                nome=nome,
                tipo=tipo,
                pose=pose,
                area=anterior.area if anterior else [],
                area_pai=pai,
                descricao=anterior.descricao if anterior else "",
                objetos=objetos if objetos is not None else (anterior.objetos if anterior else []),
                recebe=recebe if recebe is not None else (anterior.recebe if anterior else []),
                borda=list(borda) if borda else [],
            )
        )
        self._consumir(nome)
        self._apos_alteracao()
        return (
            f"local '{nome}'{' (nome automatico)' if automatico else ''} ({tipo}) "
            f"em x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}"
            + self._texto_objetivo(self.mapa.locais[nome])
        )

    @staticmethod
    def _texto_objetivo(local: Local) -> str:
        texto = ""
        if local.objetos:
            texto += f" objetos={local.objetos}"
        if local.recebe:
            texto += f" recebe={local.recebe}"
        return texto

    def _apos_alteracao(self) -> None:
        self.publicar_marcadores()
        if bool(self.get_parameter("auto_salvar").value):
            self.mapa.salvar(self.arquivo)

    # ---------- Entradas do RViz ----------

    def on_pose_clicada(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"pose em '{msg.header.frame_id}'; use o Fixed Frame "
                f"'{self.frame_id}' no RViz"
            )
            return
        q = msg.pose.orientation
        pose = Pose2D(msg.pose.position.x, msg.pose.position.y, yaw_de_quaternion(q.z, q.w))
        try:
            self.get_logger().info("RViz: " + self._definir_pose(pose))
        except ValueError as erro:
            self.get_logger().error(str(erro))

    def on_ponto_clicado(self, msg: PointStamped) -> None:
        if msg.header.frame_id != self.frame_id:
            self.get_logger().error(f"ponto fora do frame '{self.frame_id}'")
            return
        if self.cliques_bancada is not None:
            self._clique_bancada(msg.point.x, msg.point.y)
            return
        nome = self._alvo()
        if nome not in self.mapa.locais:
            self.get_logger().error(
                f"marque primeiro a pose do local ('{nome or ''}') e depois a area"
            )
            return
        self.mapa.adicionar_vertice(nome, msg.point.x, msg.point.y)
        self._apos_alteracao()
        total = len(self.mapa.locais[nome].area)
        self.get_logger().info(f"area de '{nome}': vertice {total} adicionado")

    def _clique_bancada(self, x: float, y: float) -> None:
        self.cliques_bancada.append((x, y))
        etapas = ["1o extremo da borda", "2o extremo da borda", "ponto no lado livre"]
        n = len(self.cliques_bancada)
        if n < 3:
            self.get_logger().info(
                f"bancada: {etapas[n - 1]} ok; agora clique o {etapas[n]}"
            )
            return
        p1, p2, livre = self.cliques_bancada
        self.cliques_bancada = None
        try:
            pose = aproximacao_da_bancada(
                p1, p2, livre,
                float(self.get_parameter("frente_robo_m").value),
                float(self.get_parameter("folga_bancada_m").value),
            )
        except ValueError as erro:
            self.get_logger().error(f"bancada rejeitada: {erro}. Chame ~/marcar_bancada de novo")
            return
        self.get_logger().info("bancada: " + self._definir_pose(pose, borda=[p1, p2]))

    def on_costmap(self, msg: OccupancyGrid) -> None:
        self.costmap = msg

    def _grade(self) -> Optional[Grade]:
        cm = self.costmap
        if cm is None or cm.header.frame_id != self.frame_id:
            return None
        o = cm.info.origin
        return Grade(
            cm.data, cm.info.width, cm.info.height, cm.info.resolution,
            o.position.x, o.position.y, yaw_de_quaternion(o.orientation.z, o.orientation.w),
        )

    # ---------- Servicos ----------

    def srv_marcar_bancada(self, _req, resp):
        self.cliques_bancada = []
        resp.success = True
        resp.message = (
            "clique com o Publish Point: 1o e 2o extremos da BORDA FRONTAL e 1 ponto "
            "no LADO LIVRE (onde o robo vai parar). Usa nome_atual/tipo_atual."
        )
        return resp

    def srv_importar_bancadas(self, _req, resp):
        """Le o cadastro do robo_station_manager (so leitura do arquivo)."""
        arquivo = Path(str(self.get_parameter("arquivo_bancadas").value)).expanduser()
        try:
            dados = json.loads(arquivo.read_text(encoding="utf-8"))
            if dados.get("schema") != 1 or dados.get("map_frame") != self.frame_id:
                raise ValueError(f"esperado schema 1 e map_frame '{self.frame_id}'")
            importadas = []
            for item in dados["stations"]:
                nome = str(item["name"])
                if not re.fullmatch(r"[A-Za-z0-9_-]{1,24}", nome):
                    raise ValueError(f"nome invalido '{nome}'")
                p1, p2, livre = (tuple(map(float, item[k])) for k in ("p1", "p2", "free_side"))
                pose = aproximacao_da_bancada(
                    p1, p2, livre,
                    float(self.get_parameter("frente_robo_m").value),
                    float(self.get_parameter("folga_bancada_m").value),
                )
                anterior = self.mapa.locais.get(nome)
                tipo = anterior.tipo if anterior else self._nome_tipo_pai()[1]
                self.mapa.definir(Local(
                    nome=nome, tipo=tipo, pose=pose,
                    area_pai=anterior.area_pai if anterior else None,
                    descricao=anterior.descricao if anterior else "importada do station_manager",
                    objetos=anterior.objetos if anterior else [],
                    recebe=anterior.recebe if anterior else [],
                    borda=[p1, p2],
                ))
                importadas.append(nome)
        except (OSError, KeyError, TypeError, ValueError) as erro:
            resp.success, resp.message = False, f"nao importou {arquivo}: {erro}"
            return resp
        self._apos_alteracao()
        resp.success = True
        resp.message = (
            f"{len(importadas)} bancada(s) importada(s): {', '.join(importadas)}. "
            "Confira no RViz se o mapa e o MESMO do cadastro."
        )
        return resp

    def _pose_tf(self):
        """Pose do robo no mapa, ou None (ex.: planejando sem robo ligado)."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, Time()
            ).transform
        except TransformException:
            return None
        return Pose2D(
            t.translation.x, t.translation.y, yaw_de_quaternion(t.rotation.z, t.rotation.w)
        )

    def srv_pose_atual(self, _req, resp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as erro:
            resp.success = False
            resp.message = f"sem TF {self.frame_id}->{self.base_frame}: {erro}"
            return resp
        t = tf.transform
        pose = Pose2D(
            t.translation.x, t.translation.y, yaw_de_quaternion(t.rotation.z, t.rotation.w)
        )
        try:
            resp.message = self._definir_pose(pose)
            resp.success = True
        except ValueError as erro:
            resp.success = False
            resp.message = str(erro)
        return resp

    def srv_limpar_area(self, _req, resp):
        nome = self._alvo()
        if nome not in self.mapa.locais:
            resp.success, resp.message = False, f"local '{nome}' nao existe"
            return resp
        self.mapa.locais[nome].area = []
        self._apos_alteracao()
        resp.success, resp.message = True, f"area de '{nome}' apagada"
        return resp

    def srv_remover(self, _req, resp):
        nome = self._alvo()
        resp.success = self.mapa.remover(nome) if nome else False
        resp.message = f"'{nome}' removido" if resp.success else f"'{nome}' nao existe"
        if resp.success:
            if nome == self.ultimo_local:
                self.ultimo_local = None
            self._apos_alteracao()
        return resp

    def srv_salvar(self, _req, resp):
        self.mapa.salvar(self.arquivo)
        resp.success, resp.message = True, f"salvo em {self.arquivo}"
        return resp

    def srv_listar(self, _req, resp):
        linhas = [
            f"{n}: {l.tipo} ({l.pose.x:.2f}, {l.pose.y:.2f}, {l.pose.yaw:.2f})"
            + (f" area={len(l.area)} vertices" if l.area else "")
            + self._texto_objetivo(l)
            for n, l in sorted(self.mapa.locais.items())
        ]
        resp.success = True
        resp.message = "\n".join(linhas) or "nenhum local"
        return resp

    def srv_atributos(self, _req, resp):
        """Aplica tipo, area_pai, objetos e recebe atuais sem mexer na pose."""
        _, tipo, pai = self._nome_tipo_pai()
        nome = self._alvo()
        if nome not in self.mapa.locais:
            resp.success, resp.message = False, f"local '{nome}' nao existe"
            return resp
        local = self.mapa.locais[nome]
        local.tipo, local.area_pai = tipo, pai
        objetos, recebe = self._lista("objetos_atual"), self._lista("recebe_atual")
        if objetos is not None:
            local.objetos = objetos
        if recebe is not None:
            local.recebe = recebe
        self._consumir(nome)
        self._apos_alteracao()
        resp.success = True
        resp.message = f"'{nome}' ({tipo})" + self._texto_objetivo(local)
        return resp

    def _plano(self):
        """Planeja em linha reta a partir da pose atual (se houver TF)."""
        objetivo = obter_objetivo(self.mapa, str(self.get_parameter("arquivo_objetivo").value))
        silencioso = lambda _msg: None  # noqa: E731
        executor = ExecutorObjetivo(
            self.mapa, NavegadorSimulado(silencioso), AcoesSimuladas(silencioso),
            silencioso, capacidade=int(self.get_parameter("capacidade").value),
        )
        plano = executor.planejar(objetivo, self._pose_tf())
        self._conferir_pegadas(plano)
        return plano

    def _conferir_pegadas(self, plano) -> None:
        """Acrescenta aos problemas os locais do plano onde o robo nao cabe."""
        self.avisos = []
        if not bool(self.get_parameter("conferir_pegada").value):
            return
        grade = self._grade()
        if grade is None:
            self.avisos.append("costmap global indisponivel: pegadas NAO conferidas "
                               "(a navegacao esta rodando?)")
            return
        margem = float(self.get_parameter("margem_pegada_m").value)
        for nome in dict.fromkeys(p.local for p in plano.passos):
            livre, motivo = pegada_livre(grade, self.mapa.obter(nome).pose, Pegada(), margem)
            if not livre:
                plano.problemas.append(f"local '{nome}': {motivo}")

    def srv_planejar(self, _req, resp):
        try:
            plano = self._plano()
        except (OSError, KeyError, ValueError) as erro:
            resp.success, resp.message = False, f"objetivo invalido: {erro}"
            return resp
        resp.success = not plano.problemas
        resp.message = plano.resumo() + "".join(f"\n  AVISO: {a}" for a in self.avisos)
        return resp

    def srv_finalizar(self, _req, resp):
        """Fim do mapeamento: salva tudo e, se houver plano, avisa o autonomo."""
        # Salva antes de conferir: o mapa nao se perde mesmo com o objetivo errado.
        self.mapa.salvar(self.arquivo)
        self._salvar_mapa_slam()
        resp = self.srv_planejar(_req, resp)
        if not resp.success:
            resp.message = (
                f"mapa e locais salvos, mas a missao NAO foi liberada. "
                f"Corrija e finalize de novo:\n{resp.message}"
            )
            self.get_logger().warn(resp.message)
            return resp
        self.concluido_pub.publish(String(data=self.arquivo))
        self.get_logger().info("mapeamento finalizado:\n" + resp.message)
        resp.message = f"locais salvos em {self.arquivo}; plano:\n" + resp.message
        return resp

    def _salvar_mapa_slam(self) -> None:
        servico = str(self.get_parameter("servico_salvar_mapa").value)
        if not servico:
            return
        try:
            # Import tardio: o prototipo nao depende do slam_toolbox para rodar.
            from slam_toolbox.srv import SerializePoseGraph
        except ImportError:
            self.get_logger().warn("slam_toolbox nao instalado; mapa nao salvo")
            return
        if self.cliente_mapa is None:
            self.cliente_mapa = self.create_client(SerializePoseGraph, servico)
        if not self.cliente_mapa.service_is_ready():
            self.get_logger().warn(f"{servico} indisponivel; mapa nao salvo")
            return
        # Mesmo nome do campo 'mapa', na pasta do arquivo de locais.
        destino = Path(self.arquivo).expanduser().parent / self.mapa.mapa
        req = SerializePoseGraph.Request()
        req.filename = str(destino)
        self.cliente_mapa.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f"mapa salvo em {destino}" if f.result().result == 0
                else f"falha ao salvar mapa (codigo {f.result().result})"
            )
        )

    # ---------- RViz ----------

    def publicar_marcadores(self) -> None:
        array = MarkerArray()
        limpar = Marker()
        limpar.action = Marker.DELETEALL
        array.markers.append(limpar)

        agora = self.get_clock().now().to_msg()
        marker_id = 0
        for nome, local in self.mapa.locais.items():
            r, g, b = CORES.get(local.tipo, (1.0, 0.0, 1.0))

            seta = Marker()
            seta.header.frame_id = self.frame_id
            seta.header.stamp = agora
            seta.ns, seta.id = "pose", marker_id
            seta.type, seta.action = Marker.ARROW, Marker.ADD
            seta.pose.position.x, seta.pose.position.y = local.pose.x, local.pose.y
            seta.pose.orientation.z = math.sin(local.pose.yaw / 2.0)
            seta.pose.orientation.w = math.cos(local.pose.yaw / 2.0)
            seta.scale.x, seta.scale.y, seta.scale.z = 0.35, 0.06, 0.06
            seta.color.r, seta.color.g, seta.color.b, seta.color.a = r, g, b, 1.0
            array.markers.append(seta)

            texto = Marker()
            texto.header = seta.header
            texto.ns, texto.id = "nome", marker_id
            texto.type, texto.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            texto.pose.position.x = local.pose.x
            texto.pose.position.y = local.pose.y
            texto.pose.position.z = 0.3
            texto.scale.z = 0.18
            texto.color.r = texto.color.g = texto.color.b = texto.color.a = 1.0
            texto.text = f"{nome} ({local.tipo})"
            if local.objetos:
                texto.text += "\ntem: " + ", ".join(local.objetos)
            if local.recebe:
                texto.text += "\nrecebe: " + ", ".join(local.recebe)
            array.markers.append(texto)

            if len(local.borda) == 2:
                borda = Marker()
                borda.header = seta.header
                borda.ns, borda.id = "borda", marker_id
                borda.type, borda.action = Marker.LINE_STRIP, Marker.ADD
                borda.pose.orientation.w = 1.0
                borda.scale.x = 0.06
                borda.color.r, borda.color.g, borda.color.b, borda.color.a = r, g, b, 1.0
                borda.points = [Point(x=x, y=y, z=0.03) for x, y in local.borda]
                array.markers.append(borda)

            if len(local.area) >= 2:
                contorno = Marker()
                contorno.header = seta.header
                contorno.ns, contorno.id = "area", marker_id
                contorno.type, contorno.action = Marker.LINE_STRIP, Marker.ADD
                contorno.pose.orientation.w = 1.0
                contorno.scale.x = 0.03
                contorno.color.r, contorno.color.g, contorno.color.b = r, g, b
                contorno.color.a = 0.9
                vertices = local.area + [local.area[0]]
                contorno.points = [Point(x=x, y=y, z=0.02) for x, y in vertices]
                array.markers.append(contorno)

            marker_id += 1

        self.marcadores_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocaisNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.mapa.salvar(node.arquivo)
        node.destroy_node()
        rclpy.try_shutdown()
