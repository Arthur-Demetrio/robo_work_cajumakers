"""PROTOTIPO: dirigir pelo teclado, para testar sem controle conectado.

Usa a mesma conversao do controle (comando.py) e publica TwistStamped em
/cmd_vel_nav, passando pelo Collision Monitor. Teclas iguais as do teleop
da equipe (scripts/teleop_geometry_wasd_qe.py). Precisa rodar num terminal
de verdade (ros2 run), nao dentro de um launch.

    w / s  frente / tras        a / d  girar no lugar
    q / e  lateral esquerda / direita (mecanum)
    t      liga/desliga lento   espaco  parar      Ctrl+C  sair

Segurar a tecla mantem o comando (usa a repeticao do teclado); ao soltar,
o robo para depois de `tempo_tecla_s`.
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from robo_controle.comando import Limites, joystick_para_velocidade

# tecla -> (frente, lateral, giro) em [-1, 1], como se fosse o analogico
TECLAS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "q": (0.0, 1.0, 0.0),
    "e": (0.0, -1.0, 0.0),
    "a": (0.0, 0.0, 1.0),
    "d": (0.0, 0.0, -1.0),
}


class TecladoNode(Node):
    def __init__(self) -> None:
        super().__init__("teclado_mapeamento")
        self.declare_parameter("topico_cmd_vel", "/cmd_vel_nav")
        # Maior que o atraso inicial da repeticao do teclado (~0,5 s).
        self.declare_parameter("tempo_tecla_s", 0.6)
        self.pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("topico_cmd_vel").value), 10
        )
        self.tempo_tecla = float(self.get_parameter("tempo_tecla_s").value)
        self.limites = Limites()
        self.lento = False
        self.entrada = (0.0, 0.0, 0.0)
        self.entrada_t = 0.0
        self.andando = False

    def tecla(self, c: str) -> None:
        if c in TECLAS:
            self.entrada, self.entrada_t = TECLAS[c], time.monotonic()
        elif c == " ":
            self.entrada, self.entrada_t = (0.0, 0.0, 0.0), 0.0
        elif c == "t":
            self.lento = not self.lento
            print(f"\rmodo lento {'LIGADO' if self.lento else 'desligado'}   ", end="")

    def _enviar(self, vx: float, vy: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, w
        self.pub.publish(msg)

    def publicar(self) -> None:
        if time.monotonic() - self.entrada_t > self.tempo_tecla:
            if self.andando:
                self._enviar(0.0, 0.0, 0.0)
                self.andando = False
            return
        vx, vy, w = joystick_para_velocidade(*self.entrada, self.lento, self.limites)
        self._enviar(vx, vy, w)
        self.andando = True
        print(f"\rvx={vx:+.3f} vy={vy:+.3f} m/s  w={w:+.3f} rad/s   ", end="")


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print("teclado_node precisa de um terminal interativo (use ros2 run).")
        return
    rclpy.init(args=args)
    node = TecladoNode()
    print(__doc__)
    config_antiga = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        periodo = 0.05
        while rclpy.ok():
            pronto, _, _ = select.select([sys.stdin], [], [], periodo)
            if pronto:
                node.tecla(sys.stdin.read(1).lower())
            node.publicar()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, config_antiga)
        try:
            node._enviar(0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
        print()
