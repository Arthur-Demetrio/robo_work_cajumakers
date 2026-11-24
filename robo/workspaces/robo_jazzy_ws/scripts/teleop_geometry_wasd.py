#!/usr/bin/env python3
"""Teleop WASD de baixa velocidade para ROS 2 Jazzy.
Publica no /cmd_vel_nav (Twist ou TwistStamped), sem contornar
velocity_smoother e collision_monitor. Exige foco no terminal.
"""
import curses
import time
import sys

import rclpy
from geometry_msgs.msg import Twist, TwistStamped

TOPIC = '/cmd_vel_nav'
LINEAR = 0.04           # m/s, inferior ao limite atual do robô (0.08)
ANGULAR = 0.10         # rad/s, inferior ao limite atual (0.14)
DEADMAN_SECONDS = 0.60 # cessa envio de velocidades não nulas sem teclas recentes
HZ = 20.0


def detect_type(node):
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        for name, types in node.get_topic_names_and_types():
            if name == TOPIC:
                if 'geometry_msgs/msg/TwistStamped' in types:
                    return TwistStamped
                if 'geometry_msgs/msg/Twist' in types:
                    return Twist
                raise RuntimeError(f'Tópico {TOPIC} tem tipo incompatível: {types}')
        rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f'Não foi possível detectar o tipo de {TOPIC}. Confirme que o velocity_smoother está ativo.')


def main_ui(stdscr, node, message_type):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    pub = node.create_publisher(message_type, TOPIC, 10)
    last_key = -1e5
    linear = angular = 0.0
    last_pub = 0.0
    running = True

    def publish(vx, wz):
        msg = message_type()
        if message_type is TwistStamped:
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            twist = msg.twist
        else:
            twist = msg
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        pub.publish(msg)

    try:
        while rclpy.ok() and running:
            now = time.monotonic()
            key = stdscr.getch()
            if key != -1:
                if key in (ord('q'), ord('Q'), 27):
                    running = False
                    linear = angular = 0.0
                elif key in (ord('w'), ord('W')):
                    linear, angular = LINEAR, 0.0
                    last_key = now
                elif key in (ord('s'), ord('S')):
                    linear, angular = -LINEAR, 0.0
                    last_key = now
                elif key in (ord('a'), ord('A')):
                    linear, angular = 0.0, ANGULAR
                    last_key = now
                elif key in (ord('d'), ord('D')):
                    linear, angular = 0.0, -ANGULAR
                    last_key = now
                elif key == ord(' '):
                    linear = angular = 0.0
                    last_key = -1e5
            if now - last_key > DEADMAN_SECONDS:
                linear = angular = 0.0
            if now - last_pub >= 1.0 / HZ:
                publish(linear, angular)
                last_pub = now
            try:
                stdscr.erase()
                stdscr.addstr(0, 0, 'GEOMETRY — TELEOP WASD  |  velocidade baixa')
                stdscr.addstr(2, 0, 'W: frente | S: re | A: girar esquerda | D: girar direita')
                stdscr.addstr(3, 0, 'ESPACO: parar | Q: sair')
                stdscr.addstr(5, 0, f'Topico: {TOPIC}  Tipo: {message_type.__name__}')
                stdscr.addstr(6, 0, f'Comando: vx={linear:+.3f} m/s  wz={angular:+.3f} rad/s')
                stdscr.addstr(8, 0, 'Segure a tecla (repeticao do teclado). Soltar para em ate 0.6s.')
                stdscr.addstr(9, 0, 'Use so SEM objetivo autonomo ativo. E-stop fisico acessivel.')
                stdscr.refresh()
            except curses.error:
                pass
            time.sleep(0.01)
    finally:
        for _ in range(12):
            publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = rclpy.create_node('geometry_wasd_teleop')
    try:
        msg_type = detect_type(node)
        curses.wrapper(main_ui, node, msg_type)
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f'Teleop encerrado: {exc}', file=sys.stderr)
        sys.exit(1 if isinstance(exc, RuntimeError) else 0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
