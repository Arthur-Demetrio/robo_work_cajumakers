"""PROTOTIPO: dirigir o robo mecanum com um controle (USB/Bluetooth).

Alvo: sistema Robo-WORK (~/robo_jazzy_ws). Cadeia:

  controle --(joy_node)--> /joy --(este no)--> /cmd_vel_nav (TwistStamped)
    --> velocity_smoother --> collision_monitor --> /cmd_vel --> robo_hardware

Publicar em /cmd_vel_nav (e nao direto em /cmd_vel) mantem o Collision
Monitor no caminho, como o teleop da equipe (scripts/teleop_geometry_*.py).
Por isso a navegacao (terminal 05) precisa estar rodando para o robo andar.

Seguranca:
  - so anda com o botao "homem-morto" pressionado;
  - soltou o botao, o controle desconectou ou parou de mandar /joy: para;
  - botao de emergencia: parada do robo_hardware (/emergency_stop = true). Para
    TUDO, inclusive o Nav2, e fica travada ate alguem liberar;
  - botao de liberar, segurado 2 s: libera a parada de emergencia;
  - se outro no (Nav2, mecanum_escape) estiver comandando /cmd_vel_nav, o
    controle cede e avisa, em vez de disputar o topico.

Extras para montar a arena (opcionais, desligam com string vazia):
  - salvar local: salva a pose atual como local no robo_locais (sem nome
    definido, o local ganha um nome automatico: C1, E1, Y1...);
  - salvar mapa: salva (serializa) o mapa do slam_toolbox;
  - finalizar, segurado 2 s: finaliza o mapeamento (salva mapa e locais e, se o
    objetivo tiver plano, libera a missao autonoma).
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from robo_controle.comando import BotaoSegurado, Limites, joystick_para_velocidade
from robo_controle.registro import PASTA_PADRAO, ArquivoJsonl, SessaoControle

Velocidade = Tuple[float, float, float]


class ControleNode(Node):
    def __init__(self) -> None:
        super().__init__("controle_mapeamento")

        p = self.declare_parameter
        p("topico_cmd_vel", "/cmd_vel_nav")
        p("frame_id", "base_link")
        p("topico_emergencia", "/emergency_stop")
        p("taxa_hz", 20.0)
        p("timeout_joy_s", 0.5)
        # Comando de outro no mais recente que isso: o controle cede.
        p("ceder_por_s", 0.5)
        # Indices do controle do notebook (DragonRise "PC TWIN SHOCK",
        # 0079:0006), iguais ao config/controle.yaml. Outro controle: rode
        # `ros2 run robo_controle mapear_controle` (cada modelo numera diferente).
        p("eixo_frente", 1)
        p("eixo_lateral", 0)
        p("eixo_giro", 3)
        p("inverter_frente", False)
        p("inverter_lateral", False)
        p("inverter_giro", False)
        p("botao_homem_morto", 8)
        p("botao_lento", 9)
        p("botao_emergencia", 0)
        p("botao_liberar", 2)         # segurar
        p("segurar_liberar_s", 2.0)
        p("botao_salvar_local", 1)
        p("botao_salvar_mapa", 3)
        p("botao_finalizar", 4)       # segurar
        p("segurar_finalizar_s", 2.0)
        p("servico_salvar_local", "/locais/salvar_pose_atual")
        p("servico_finalizar", "/locais/finalizar_mapeamento")
        p("arquivo_mapa", "~/robo_mapas/arena")
        # Registro do teste (JSONL por sessao). "" desliga.
        p("pasta_registro", PASTA_PADRAO)
        for nome, padrao in vars(Limites()).items():
            p(f"limites.{nome}", padrao)

        g = lambda nome: self.get_parameter(nome).value  # noqa: E731
        self.limites = Limites(**{n: float(g(f"limites.{n}")) for n in vars(Limites())})
        self.timeout_joy = float(g("timeout_joy_s"))
        self.ceder_por = float(g("ceder_por_s"))
        self.topico = str(g("topico_cmd_vel"))
        self.frame_id = str(g("frame_id"))

        self.pub = self.create_publisher(TwistStamped, self.topico, 10)
        # Mesmo QoS da assinatura do robo_hardware: o ultimo valor fica
        # guardado, entao a parada vale ate alguem publicar false.
        qos_emergencia = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topico_emergencia = str(g("topico_emergencia"))
        self.emergencia_pub = (
            self.create_publisher(Bool, topico_emergencia, qos_emergencia)
            if topico_emergencia else None
        )
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        # Escuta o proprio topico para perceber quando outro no esta comandando.
        self.create_subscription(TwistStamped, self.topico, self.on_cmd_vel, 10)
        self.create_timer(1.0 / float(g("taxa_hz")), self.on_timer)

        self.ultimo_joy: Optional[Joy] = None
        self.ultimo_joy_t = 0.0
        self.botoes_antes: List[int] = []
        self.travado = False
        self.andando = False
        self.zeros_pendentes = 0
        self.ultimo_enviado: Optional[Velocidade] = None
        self.ultimo_enviado_t = 0.0
        self.externo_t = 0.0

        servico_local = str(g("servico_salvar_local"))
        self.cliente_local = (
            self.create_client(Trigger, servico_local) if servico_local else None
        )
        self.cliente_mapa = None
        self.arquivo_mapa = str(g("arquivo_mapa"))
        servico_finalizar = str(g("servico_finalizar"))
        self.cliente_finalizar = (
            self.create_client(Trigger, servico_finalizar) if servico_finalizar else None
        )
        self.finalizar = BotaoSegurado(float(g("segurar_finalizar_s")))
        self.liberar = BotaoSegurado(float(g("segurar_liberar_s")))
        self.sessao = self._abrir_registro(str(g("pasta_registro")))

        self.get_logger().info(
            f"publicando TwistStamped em {self.topico}; segure o botao "
            f"{g('botao_homem_morto')} para andar. maximos vx={self.limites.vx_max} "
            f"vy={self.limites.vy_max} m/s, w={self.limites.w_max} rad/s"
        )

    def _abrir_registro(self, pasta: str) -> SessaoControle:
        arquivo = None
        if pasta:
            try:
                arquivo = ArquivoJsonl(pasta, "controle")
                self.get_logger().info(f"registro do teste: {arquivo.caminho}")
            except OSError as erro:
                self.get_logger().warn(f"registro desligado: {erro}")
        config = {
            "topico_cmd_vel": self.topico,
            "topico_emergencia": str(self.get_parameter("topico_emergencia").value),
            "limites": vars(self.limites),
            "eixos": {n: self._int(f"eixo_{n}") for n in ("frente", "lateral", "giro")},
        }
        return SessaoControle(arquivo, config)

    # ---------- Entrada ----------

    def on_joy(self, msg: Joy) -> None:
        self.ultimo_joy = msg
        self.ultimo_joy_t = time.monotonic()

        apertou = lambda idx: (  # noqa: E731
            0 <= idx < len(msg.buttons)
            and msg.buttons[idx] == 1
            and (idx >= len(self.botoes_antes) or self.botoes_antes[idx] == 0)
        )
        if apertou(self._int("botao_emergencia")):
            self.emergencia(True)
        if apertou(self._int("botao_liberar")) and self.travado:
            self.get_logger().info(
                f"segure {self.liberar.segundos:.0f} s para liberar a parada de emergencia"
            )
        if self.liberar.atualizar(
            self._botao(msg, self._int("botao_liberar")), time.monotonic()
        ):
            self.emergencia(False)
        if apertou(self._int("botao_salvar_local")):
            self.salvar_local()
        if apertou(self._int("botao_salvar_mapa")):
            self.salvar_mapa()
        if apertou(self._int("botao_finalizar")) and self.cliente_finalizar:
            self.get_logger().info(
                f"segure {self.finalizar.segundos:.0f} s para finalizar o mapeamento"
            )
        # O joy_node repete o /joy (autorepeat_rate), entao o tempo avanca
        # mesmo com o botao parado.
        if self.finalizar.atualizar(
            self._botao(msg, self._int("botao_finalizar")), time.monotonic()
        ):
            self.finalizar_mapeamento()
        self.botoes_antes = list(msg.buttons)

    def on_cmd_vel(self, msg: TwistStamped) -> None:
        t = msg.twist
        recebido = (t.linear.x, t.linear.y, t.angular.z)
        if not any(recebido):
            return  # zeros (parada) nao indicam que alguem esta dirigindo
        meu = (
            self.ultimo_enviado is not None
            and time.monotonic() - self.ultimo_enviado_t < 0.2
            and all(abs(a - b) < 1e-6 for a, b in zip(recebido, self.ultimo_enviado))
        )
        if not meu:
            self.externo_t = time.monotonic()

    def _int(self, nome: str) -> int:
        return int(self.get_parameter(nome).value)

    def _sinal(self, nome: str) -> float:
        return -1.0 if self.get_parameter(nome).value else 1.0

    @staticmethod
    def _botao(msg: Joy, idx: int) -> bool:
        return 0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1

    @staticmethod
    def _eixo(msg: Joy, idx: int) -> float:
        return float(msg.axes[idx]) if 0 <= idx < len(msg.axes) else 0.0

    # ---------- Saida ----------

    def comando_atual(self) -> Optional[Velocidade]:
        """(vx, vy, w) a publicar, ou None se o robo deve ficar parado."""
        msg = self.ultimo_joy
        if msg is None or time.monotonic() - self.ultimo_joy_t > self.timeout_joy:
            return None
        if self.travado or not self._botao(msg, self._int("botao_homem_morto")):
            return None
        return joystick_para_velocidade(
            self._eixo(msg, self._int("eixo_frente")) * self._sinal("inverter_frente"),
            self._eixo(msg, self._int("eixo_lateral")) * self._sinal("inverter_lateral"),
            self._eixo(msg, self._int("eixo_giro")) * self._sinal("inverter_giro"),
            self._botao(msg, self._int("botao_lento")),
            self.limites,
        )

    def _publicar(self, v: Velocidade) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = v
        self.ultimo_enviado, self.ultimo_enviado_t = v, time.monotonic()
        self.pub.publish(msg)

    def on_timer(self) -> None:
        joy = self.ultimo_joy
        self.sessao.joy(
            joy is not None and time.monotonic() - self.ultimo_joy_t <= self.timeout_joy
        )
        comando = self.comando_atual()
        cedendo = comando is not None and time.monotonic() - self.externo_t < self.ceder_por
        self.sessao.cedendo(cedendo)
        if cedendo:
            self.get_logger().warn(
                f"outro no esta comandando {self.topico} (Nav2?); controle ignorado. "
                f"Cancele o objetivo ou use o botao de emergencia "
                f"({self._int('botao_emergencia')}) para parar tudo.",
                throttle_duration_sec=2.0,
            )
            comando = None
        self.sessao.comando(
            comando, comando is not None and self._botao(joy, self._int("botao_lento"))
        )
        if comando is not None:
            self._publicar(comando)
            self.andando = True
            return
        if self.andando:
            # Alguns zeros garantem a parada mesmo se uma mensagem se perder;
            # depois fica em silencio para nao brigar com o Nav2.
            self.andando = False
            self.zeros_pendentes = 3
        if self.zeros_pendentes > 0:
            self._publicar((0.0, 0.0, 0.0))
            self.zeros_pendentes -= 1

    # ---------- Parada de emergencia ----------

    def emergencia(self, ativa: bool) -> None:
        self.travado = ativa
        self.sessao.emergencia(ativa)
        if self.emergencia_pub is None:
            self.get_logger().warn(
                "PARADA local do controle (topico de emergencia desligado)" if ativa
                else "controle destravado"
            )
            return
        self.emergencia_pub.publish(Bool(data=ativa))
        if ativa:
            self.get_logger().error(
                "PARADA DE EMERGENCIA enviada ao robo_hardware (para tambem o Nav2). "
                f"Segure o botao {self._int('botao_liberar')} por "
                f"{self.liberar.segundos:.0f} s para liberar."
            )
        else:
            self.get_logger().warn("parada de emergencia LIBERADA: o robo pode voltar a andar")

    # ---------- Extras ----------

    def _chamar(self, cliente, rotulo: str, nome: str) -> None:
        if cliente is None:
            return
        if not cliente.service_is_ready():
            self.get_logger().warn(
                f"{rotulo}: {cliente.srv_name} indisponivel (prototipo robo_locais rodando?)"
            )
            self.sessao.servico(nome, False, f"{cliente.srv_name} indisponivel")
            return

        def resposta(futuro) -> None:
            r = futuro.result()
            # O rclpy proibe mudar o nivel de log numa mesma linha do codigo
            # ("Logger severity cannot be changed between calls"): uma
            # chamada para cada nivel.
            self.sessao.servico(nome, r.success, r.message)
            if r.success:
                self.get_logger().info(f"{rotulo}: {r.message}")
            else:
                self.get_logger().warn(f"{rotulo}: {r.message}")

        cliente.call_async(Trigger.Request()).add_done_callback(resposta)

    def salvar_local(self) -> None:
        self._chamar(self.cliente_local, "salvar local", "salvar_local")

    def finalizar_mapeamento(self) -> None:
        self.get_logger().info("finalizando mapeamento...")
        self._chamar(self.cliente_finalizar, "finalizar mapeamento", "finalizar")

    def salvar_mapa(self) -> None:
        if not self.arquivo_mapa:
            return
        try:
            # Import tardio: o prototipo nao depende do slam_toolbox para rodar.
            from slam_toolbox.srv import SerializePoseGraph
        except ImportError:
            self.get_logger().error("slam_toolbox nao instalado; mapa nao salvo")
            self.sessao.servico("salvar_mapa", False, "slam_toolbox nao instalado")
            return
        if self.cliente_mapa is None:
            self.cliente_mapa = self.create_client(
                SerializePoseGraph, "/slam_toolbox/serialize_map"
            )
        if not self.cliente_mapa.service_is_ready():
            self.get_logger().warn("slam_toolbox nao esta rodando; mapa nao salvo")
            self.sessao.servico("salvar_mapa", False, "slam_toolbox nao esta rodando")
            return

        destino = Path(self.arquivo_mapa).expanduser()
        destino.parent.mkdir(parents=True, exist_ok=True)
        req = SerializePoseGraph.Request()
        req.filename = str(destino)
        def resposta(futuro) -> None:
            codigo = futuro.result().result
            ok = codigo == 0
            mensagem = f"mapa salvo em {destino}" if ok else f"falha ao salvar mapa (codigo {codigo})"
            self.sessao.servico("salvar_mapa", ok, mensagem)
            self.get_logger().info(mensagem)

        self.cliente_mapa.call_async(req).add_done_callback(resposta)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as erro:  # registra a queda antes de propagar
        node.sessao.erro(f"{type(erro).__name__}: {erro}")
        raise
    finally:
        resumo = node.sessao.fechar()
        print(f"[controle] sessao: {resumo['trechos']} trecho(s), "
              f"{resumo['tempo_dirigindo_s']:.0f} s dirigindo, "
              f"{resumo['emergencias']} emergencia(s)")
        try:
            node._publicar((0.0, 0.0, 0.0))
        except Exception:  # contexto ja finalizado pelo Ctrl+C
            pass
        node.destroy_node()
        rclpy.try_shutdown()
