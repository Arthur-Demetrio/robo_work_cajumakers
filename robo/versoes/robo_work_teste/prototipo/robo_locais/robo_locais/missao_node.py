"""PROTOTIPO: executa uma missao (YAML) sobre os locais marcados.

modo:=simulado  -> nao precisa de robo nem de Nav2 (padrao);
modo:=nav2      -> usa o nav2_simple_commander para ir ate as poses. Exige
                   permitir_movimento:=true, como o allow_motion do
                   robo_station_manager: o robo so anda se voce pedir.

pegar/alinhar/soltar continuam simulados nos dois modos: sao os ganchos
onde a visao e o braco serao ligados no futuro (ver README).

Alvo: sistema Robo-WORK (~/robo_jazzy_ws). O NavigateToPose usa a behavior
tree padrao configurada no bt_navigator do robo; este no nao publica
velocidade, entao o Collision Monitor continua no caminho.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robo_locais.locais import MapaDeLocais, Pose2D, yaw_de_quaternion
from robo_locais.missao import (
    AcoesSimuladas,
    AoEvento,
    ExecutorMissao,
    Navegador,
    NavegadorSimulado,
    _ignorar,
    carregar_missao,
)
from robo_locais.registro import PASTA_PADRAO, ArquivoJsonl, RegistroMissao

AVISO_SEM_PERMISSAO = (
    "modo nav2 move o robo: rode com permitir_movimento:=true "
    "(so em teste supervisionado, com a parada de emergencia a mao)"
)


class NavegadorNav2(Navegador):
    """Integracao com o Nav2 via acao NavigateToPose (nav2_simple_commander)."""

    # Somado ao custo em linha reta quando o planner nao acha caminho: o
    # local vai para o fim da fila, mas nao some (o mapa pode ter mudado).
    PENALIDADE_SEM_CAMINHO = 1000.0

    def __init__(
        self,
        localizador: str,
        timeout_minimo_s: float,
        log,
        base_frame: str = "base_link",
        velocidade_media_mps: float = 0.04,
        fator_timeout: float = 3.0,
        topico_emergencia: str = "/emergency_stop",
    ) -> None:
        # Import tardio: o modo simulado nao depende do Nav2 instalado.
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformListener

        self._task_result = TaskResult
        self.timeout_minimo_s = timeout_minimo_s
        self.velocidade_media = velocidade_media_mps
        self.fator_timeout = fator_timeout
        self.log = log
        self.base_frame = base_frame
        # Trocado pelo no a cada missao, para os eventos irem ao registro certo.
        self.ao_evento: AoEvento = _ignorar
        self.nav = BasicNavigator(node_name="missao_navigator")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.nav, spin_thread=True)

        # Parada de emergencia do robo_hardware (travada, transient local).
        # Enquanto ativa, a missao espera em vez de contar falha.
        self.emergencia = False
        if topico_emergencia:
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.nav.create_subscription(
                Bool, topico_emergencia, lambda m: setattr(self, "emergencia", m.data), qos
            )

        self.log(f"aguardando Nav2 ativo (localizador: {localizador})")
        self.nav.waitUntilNav2Active(localizer=localizador)

    def _pose_stamped(self, pose: Pose2D, frame_id: str):
        from geometry_msgs.msg import PoseStamped

        alvo = PoseStamped()
        alvo.header.frame_id = frame_id
        alvo.header.stamp = self.nav.get_clock().now().to_msg()
        alvo.pose.position.x = pose.x
        alvo.pose.position.y = pose.y
        alvo.pose.orientation.z = math.sin(pose.yaw / 2.0)
        alvo.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return alvo

    def custo(self, origem: Pose2D, destino: Pose2D, frame_id: str) -> float:
        """Comprimento do caminho do planner do Nav2 (desvia das paredes)."""
        reta = super().custo(origem, destino, frame_id)
        caminho = self.nav.getPath(
            self._pose_stamped(origem, frame_id),
            self._pose_stamped(destino, frame_id),
            use_start=True,
        )
        if caminho is None or len(caminho.poses) < 2:
            return reta + self.PENALIDADE_SEM_CAMINHO
        pontos = [p.pose.position for p in caminho.poses]
        return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(pontos, pontos[1:]))

    def pose_atual(self, frame_id: str) -> Optional[Pose2D]:
        from rclpy.time import Time
        from tf2_ros import TransformException

        try:
            t = self.tf_buffer.lookup_transform(frame_id, self.base_frame, Time()).transform
        except TransformException:
            return None
        return Pose2D(t.translation.x, t.translation.y,
                      yaw_de_quaternion(t.rotation.z, t.rotation.w))

    def timeout_para(self, pose: Pose2D, frame_id: str) -> float:
        """O robo anda devagar (<= 0,08 m/s): o prazo cresce com a distancia."""
        atual = self.pose_atual(frame_id)
        if atual is None:
            return self.timeout_minimo_s
        distancia = math.hypot(pose.x - atual.x, pose.y - atual.y)
        return max(self.timeout_minimo_s, self.fator_timeout * distancia / self.velocidade_media)

    def _esperar_liberar(self) -> float:
        """Bloqueia enquanto a parada de emergencia estiver ativa; devolve o tempo parado."""
        if not self.emergencia:
            return 0.0
        inicio = time.monotonic()
        self.log("PARADA DE EMERGENCIA ativa: missao pausada ate liberar")
        while self.emergencia and rclpy.ok():
            rclpy.spin_once(self.nav, timeout_sec=0.2)
        parado = time.monotonic() - inicio
        self.log("parada liberada: missao continua")
        self.ao_evento("pausa_emergencia", {"duracao_s": round(parado, 1)})
        return parado

    def ir_para(self, pose: Pose2D, frame_id: str) -> bool:
        alvo = self._pose_stamped(pose, frame_id)
        timeout = self.timeout_para(pose, frame_id)
        self._esperar_liberar()
        if not self.nav.goToPose(alvo):
            self.ao_evento("objetivo_recusado", {"x": round(pose.x, 2), "y": round(pose.y, 2)})
            return False
        inicio = time.monotonic()
        while not self.nav.isTaskComplete():
            if self.emergencia:
                # Cancela para o Nav2 nao acumular falhas de progresso com os
                # motores travados; reenvia o mesmo objetivo depois.
                self.nav.cancelTask()
                inicio += self._esperar_liberar()
                alvo.header.stamp = self.nav.get_clock().now().to_msg()
                if not self.nav.goToPose(alvo):
                    return False
                continue
            if time.monotonic() - inicio > timeout:
                self.log(f"timeout de {timeout:.0f} s, cancelando")
                self.ao_evento("timeout", {"x": round(pose.x, 2), "y": round(pose.y, 2),
                                           "prazo_s": round(timeout, 1)})
                self.nav.cancelTask()
                return False
        return self.nav.getResult() == self._task_result.SUCCEEDED

    def cancelar(self) -> None:
        """Cancela o objetivo em andamento (ex.: Ctrl+C no no da missao)."""
        try:
            self.nav.cancelTask()
        except Exception:  # sem objetivo ativo ou contexto ja encerrado
            pass


def declarar_parametros_nav2(node: Node) -> None:
    node.declare_parameter("permitir_movimento", False)
    node.declare_parameter("localizador", "slam_toolbox")
    node.declare_parameter("base_frame", "base_link")
    node.declare_parameter("timeout_navegacao_s", 60.0)  # minimo; cresce com a distancia
    node.declare_parameter("velocidade_media_mps", 0.04)
    node.declare_parameter("fator_timeout", 3.0)
    node.declare_parameter("topico_emergencia", "/emergency_stop")
    # Registro do teste (JSONL por missao). "" desliga.
    node.declare_parameter("pasta_registro", PASTA_PADRAO)


def abrir_registro(node: Node, sufixo: str, contexto: dict) -> RegistroMissao:
    """Registro da missao; se a pasta falhar, segue sem gravar (so avisa)."""
    pasta = str(node.get_parameter("pasta_registro").value)
    arquivo = None
    if pasta:
        try:
            arquivo = ArquivoJsonl(pasta, sufixo)
            node.get_logger().info(f"registro da missao: {arquivo.caminho}")
        except OSError as erro:
            node.get_logger().warn(f"registro desligado: {erro}")
    return RegistroMissao(arquivo, contexto)


def criar_navegador(node: Node, modo: str, log) -> Navegador:
    """Navegador do modo pedido; nav2 so com permitir_movimento:=true."""
    if modo == "simulado":
        return NavegadorSimulado(log)
    if modo != "nav2":
        raise ValueError(f"modo desconhecido '{modo}' (use simulado ou nav2)")
    if not bool(node.get_parameter("permitir_movimento").value):
        raise ValueError(AVISO_SEM_PERMISSAO)
    g = lambda nome: node.get_parameter(nome).value  # noqa: E731
    return NavegadorNav2(
        str(g("localizador")),
        float(g("timeout_navegacao_s")),
        log,
        str(g("base_frame")),
        float(g("velocidade_media_mps")),
        float(g("fator_timeout")),
        str(g("topico_emergencia")),
    )


class MissaoNode(Node):
    def __init__(self) -> None:
        super().__init__("missao")
        self.declare_parameter("arquivo_locais", "~/robo_mapas/arena_locais.yaml")
        self.declare_parameter("arquivo_missao", "")
        self.declare_parameter("modo", "simulado")
        self.declare_parameter("parar_na_falha", True)
        declarar_parametros_nav2(self)
        self.navegador: Optional[Navegador] = None
        self.registro: Optional[RegistroMissao] = None

    def executar(self) -> bool:
        log = self.get_logger().info
        mapa = MapaDeLocais.carregar(str(self.get_parameter("arquivo_locais").value))
        arquivo_missao = str(self.get_parameter("arquivo_missao").value)
        if not arquivo_missao:
            self.get_logger().error("informe arquivo_missao:=<caminho do YAML>")
            return False
        passos = carregar_missao(arquivo_missao)
        modo = str(self.get_parameter("modo").value)
        self.registro = abrir_registro(self, "missao_manual", {
            "tipo": "missao escrita a mao (missao_node)",
            "arquivo_locais": str(self.get_parameter("arquivo_locais").value),
            "arquivo_missao": arquivo_missao,
            "mapa": mapa.mapa,
            "modo": modo,
            "permitir_movimento": bool(self.get_parameter("permitir_movimento").value),
            "passos": [f"{p.acao} {p.local}" for p in passos],
        })

        try:
            self.navegador = criar_navegador(self, modo, log)
        except ValueError as erro:
            self.get_logger().error(str(erro))
            self.registro.erro(str(erro))
            self.registro.fechar(situacao="nao_iniciada")
            return False
        if isinstance(self.navegador, NavegadorNav2):
            self.navegador.ao_evento = self.registro.evento

        executor = ExecutorMissao(mapa, self.navegador, AcoesSimuladas(log), log,
                                  self.registro.evento)
        resultado = executor.executar(
            passos, parar_na_falha=bool(self.get_parameter("parar_na_falha").value)
        )
        log(
            f"missao {'CONCLUIDA' if resultado.sucesso else 'FALHOU'}: "
            f"{sum(r.sucesso for r in resultado.passos)}/{len(passos)} passos ok"
        )
        self.registro.fechar(situacao="concluida" if resultado.sucesso else "incompleta")
        return resultado.sucesso


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissaoNode()
    try:
        node.executar()
    except (KeyboardInterrupt, ExternalShutdownException):
        if isinstance(node.navegador, NavegadorNav2):
            node.navegador.cancelar()
        if node.registro is not None:
            node.registro.fechar(situacao="interrompida")
    except Exception as erro:
        if node.registro is not None:
            node.registro.erro(f"{type(erro).__name__}: {erro}")
            node.registro.fechar(situacao="erro")
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
