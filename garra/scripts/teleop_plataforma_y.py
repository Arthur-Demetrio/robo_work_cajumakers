#!/usr/bin/env python3
"""Move a plataforma Y pelo teclado e mostra ao vivo o y do plano da garra (pela camera).

Para medir o curso real da plataforma (limites --y-min / --y-max do plataforma_y.py).

Precisa rodando: garra_real.launch.py e camera_cubo --tag-plataforma 256 (publica
/garra/plataforma_y/posicao). NAO rode junto com o plataforma_y.py: os dois mandam em
/garra/plataforma_y.

    python3 scripts/teleop_plataforma_y.py

Teclas (segure para andar; soltou, para):
    a / d      esquerda / direita
    A / D      esquerda / direita DEVAGAR (~7 mm/s)
    espaco     para
    m          marca o y atual (lista no fim)
    q          sai (para a plataforma)

Os fins de curso continuam valendo: o firmware para sozinho ao apertar a chave do lado.
y do plano = y do tag + DESLOCAMENTO (o mesmo --deslocamento do plataforma_y.py).
"""

import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Point
import rclpy
from std_msgs.msg import Int8

DESLOCAMENTO = -0.0291
TOPICO_CMD = '/garra/plataforma_y'
TOPICO_POS = '/garra/plataforma_y/posicao'
PERIODO = 0.1          # s entre comandos (o driver para com 300 ms sem mensagem)
SEGURAR_1 = 0.55       # s que um toque anda (cobre o atraso da repeticao do teclado)
SEGURAR_REP = 0.15     # s depois que a repeticao do teclado comecou
TECLAS = {'a': -1, 'd': 1, 'A': -2, 'D': 2}
NOMES = {-2: 'esquerda devagar', -1: 'esquerda', 0: 'parada', 1: 'direita', 2: 'direita devagar'}


def main():
    rclpy.init()
    no = rclpy.create_node('teleop_plataforma_y')
    pub = no.create_publisher(Int8, TOPICO_CMD, 10)
    estado = {'y': None, 't': 0.0}

    def ao_ver(p):
        estado['y'], estado['t'] = p.y + DESLOCAMENTO, time.monotonic()

    no.create_subscription(Point, TOPICO_POS, ao_ver, 10)
    marcas, cmd, ate, ultima_tecla = [], 0, 0.0, 0.0
    vmin = vmax = None
    antigo = termios.tcgetattr(sys.stdin)
    print(__doc__.split('Teclas')[1].split('Os fins')[0])
    try:
        tty.setcbreak(sys.stdin.fileno())
        prox = 0.0
        while rclpy.ok():
            rclpy.spin_once(no, timeout_sec=0.0)
            agora = time.monotonic()
            if select.select([sys.stdin], [], [], 0.02)[0]:
                c = sys.stdin.read(1)
                if c == 'q':
                    break
                if c in TECLAS:
                    rep = agora - ultima_tecla < 0.12 and TECLAS[c] == cmd
                    cmd, ate, ultima_tecla = TECLAS[c], agora + (SEGURAR_REP if rep else SEGURAR_1), agora
                elif c == ' ':
                    cmd, ate = 0, 0.0
                elif c == 'm':
                    if estado['y'] is not None:
                        marcas.append(estado['y'])
                        print(f'\n  marcado y = {estado["y"]:+.4f} m')
            if agora > ate:
                cmd = 0
            if agora >= prox:
                pub.publish(Int8(data=cmd))
                prox = agora + PERIODO
            y = estado['y']
            if y is not None and agora - estado['t'] < 0.5:
                vmin = y if vmin is None else min(vmin, y)
                vmax = y if vmax is None else max(vmax, y)
                txt = f'y plano = {y:+.4f} m   (min {vmin:+.4f}  max {vmax:+.4f})'
            else:
                txt = 'camera sem o tag da plataforma            '
            sys.stdout.write(f'\r{txt}   {NOMES[cmd]:16s}')
            sys.stdout.flush()
    finally:
        for _ in range(3):
            pub.publish(Int8(data=0))
            time.sleep(0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, antigo)
        print('\nPlataforma parada.')
        if vmin is not None:
            print(f'Faixa vista: y de {vmin:+.4f} a {vmax:+.4f} m (plano da garra, robo_base)')
        for i, m in enumerate(marcas, 1):
            print(f'  marca {i}: y = {m:+.4f} m')
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
