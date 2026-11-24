#!/usr/bin/env python3
"""Giro rapido nas viradas grandes (26/10): RotationShim a 0.40 rad/s.

Teste de 26/10 (teste_velocidade --giro): comando 0.10 -> ~16 graus/s (o que o shim usava
na prova1), 0.14 -> 18, 0.28 -> 22, 0.40 -> 25, 0.55 -> 29 graus/s.

Muda so o nav2_params.yaml:
  1) FollowPath (RotationShim): rotate_to_heading_angular_vel 0.10 -> 0.40,
     max_angular_accel 0.2 -> 0.8, closed_loop: false (usa o ultimo comando, nao a odom).
     O shim so gira quando o caminho esta a mais de 45 graus e devolve ao DWB a ~22 graus.
     O DWB continua com max_vel_theta 0.14 (ajuste fino e chegada nas mesas iguais).
  2) velocity_smoother: giro maximo 0.14 -> 0.40 (sem isso ele cortava o shim em 0.14),
     aceleracao do giro 0.2 -> 0.8.
  3) collision_monitor (VelocityPolygonStop):
     - forward/left/backward/right passam a cobrir giro ate +-0.60 (antes +-0.20);
     - novos poligonos giro_esq/giro_dir para giro parado acima de 0.20 rad/s, com 7 cm de
       margem em volta do robo (o 'stopped' tem 4 cm). Antes, um giro acima de 0.20 nao
       tinha poligono nenhum.

Uso:
    python3 aplicar_giro_rapido.py            # aplica (backup antes)
    python3 aplicar_giro_rapido.py --desfazer # volta o backup mais recente
Depois: compile o robo_navigation e reinicie o Nav2 (o script imprime os comandos).
"""
import glob
import os
import shutil
import sys
import time

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
MARCA = 'GIRO_RAPIDO'
SUF = '.bak_giro_'

TROCAS = [
    # (texto atual, texto novo, quantas vezes deve aparecer)
    ('      rotate_to_heading_angular_vel: 0.10  # comando (o robo real gira ~2x)\n',
     '      rotate_to_heading_angular_vel: 0.40  # GIRO_RAPIDO: ~25 graus/s real (teste 26/10)\n', 1),
    ('      max_angular_accel: 0.2\n',
     '      max_angular_accel: 0.8\n', 1),
    ('      simulate_ahead_time: 1.0             # confere colisao do giro\n',
     '      simulate_ahead_time: 1.0             # confere colisao do giro\n'
     '      closed_loop: false                   # usa o ultimo comando (a odom nao mede bem o giro)\n', 1),
    ('    max_velocity:\n    - 0.08\n    - 0.08\n    - 0.14\n',
     '    max_velocity:\n    - 0.08\n    - 0.08\n    - 0.40  # GIRO_RAPIDO (era 0.14; o DWB continua limitado a 0.14)\n', 1),
    ('    min_velocity:\n    - -0.08\n    - -0.08\n    - -0.14\n',
     '    min_velocity:\n    - -0.08\n    - -0.08\n    - -0.40\n', 1),
    ('    max_accel:\n    - 0.2\n    - 0.2\n    - 0.2\n',
     '    max_accel:\n    - 0.2\n    - 0.2\n    - 0.8\n', 1),
    ('        linear_max: 0.20\n'
     '        direction_start_angle: -0.785\n'
     '        direction_end_angle: 0.785\n'
     '        theta_min: -0.20\n'
     '        theta_max: 0.20\n',
     '        linear_max: 0.20\n'
     '        direction_start_angle: -0.785\n'
     '        direction_end_angle: 0.785\n'
     '        theta_min: -0.60\n'
     '        theta_max: 0.60\n', 1),
    ('        direction_start_angle: 0.785\n'
     '        direction_end_angle: 2.356\n'
     '        theta_min: -0.20\n'
     '        theta_max: 0.20\n',
     '        direction_start_angle: 0.785\n'
     '        direction_end_angle: 2.356\n'
     '        theta_min: -0.60\n'
     '        theta_max: 0.60\n', 1),
    ('        direction_start_angle: 2.356\n'
     '        direction_end_angle: -2.356\n'
     '        theta_min: -0.20\n'
     '        theta_max: 0.20\n',
     '        direction_start_angle: 2.356\n'
     '        direction_end_angle: -2.356\n'
     '        theta_min: -0.60\n'
     '        theta_max: 0.60\n', 1),
    ('        direction_start_angle: -2.356\n'
     '        direction_end_angle: -0.785\n'
     '        theta_min: -0.20\n'
     '        theta_max: 0.20\n',
     '        direction_start_angle: -2.356\n'
     '        direction_end_angle: -0.785\n'
     '        theta_min: -0.60\n'
     '        theta_max: 0.60\n', 1),
    ('      - right\n      - stopped\n',
     '      - right\n      - stopped\n      - giro_esq\n      - giro_dir\n', 1),
    ('        linear_min: 0.0\n'
     '        linear_max: 0.005\n'
     '        direction_start_angle: -3.1415\n'
     '        direction_end_angle: 3.1415\n'
     '        theta_min: -0.20\n'
     '        theta_max: 0.20\n',
     '        linear_min: 0.0\n'
     '        linear_max: 0.005\n'
     '        direction_start_angle: -3.1415\n'
     '        direction_end_angle: 3.1415\n'
     '        theta_min: -0.20\n'
     '        theta_max: 0.20\n'
     '\n'
     '      # GIRO_RAPIDO: giro parado acima de 0.20 rad/s (RotationShim a 0.40).\n'
     '      # 7 cm de margem em volta do footprint (o stopped tem 4 cm).\n'
     '      giro_esq:\n'
     '        points: "[[0.435, 0.385], [0.435, -0.385], [-0.275, -0.385], [-0.275, 0.385]]"\n'
     '        linear_min: 0.0\n'
     '        linear_max: 0.005\n'
     '        direction_start_angle: -3.1415\n'
     '        direction_end_angle: 3.1415\n'
     '        theta_min: 0.20\n'
     '        theta_max: 0.60\n'
     '\n'
     '      giro_dir:\n'
     '        points: "[[0.435, 0.385], [0.435, -0.385], [-0.275, -0.385], [-0.275, 0.385]]"\n'
     '        linear_min: 0.0\n'
     '        linear_max: 0.005\n'
     '        direction_start_angle: -3.1415\n'
     '        direction_end_angle: 3.1415\n'
     '        theta_min: -0.60\n'
     '        theta_max: -0.20\n', 1),
]


def desfazer():
    baks = sorted(glob.glob(ARQ + SUF + '*'))
    if not baks:
        print('Nenhum backup %s* encontrado.' % SUF)
        return 1
    shutil.copy2(baks[-1], ARQ)
    print('Restaurado de %s' % baks[-1])
    return 0


def aplicar_texto(txt):
    """Devolve o texto novo ou levanta ValueError dizendo o que nao bateu."""
    for velho, novo, n in TROCAS:
        achou = txt.count(velho)
        if achou != n:
            raise ValueError('esperava %d trecho(s) e achei %d:\n%s' % (n, achou, velho))
        txt = txt.replace(velho, novo)
    return txt


def main():
    if '--desfazer' in sys.argv:
        return desfazer()
    if not os.path.isfile(ARQ):
        print('Nao achei %s. Nada alterado.' % ARQ)
        return 1
    txt = open(ARQ).read()
    if MARCA in txt:
        print('Ja aplicado (achei a marca %s). Nada a fazer.' % MARCA)
        return 0
    try:
        novo = aplicar_texto(txt)
    except ValueError as exc:
        print('O nav2_params.yaml esta diferente do que eu esperava, nada alterado.\n%s' % exc)
        return 1
    bak = ARQ + SUF + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(ARQ, bak)
    open(ARQ, 'w').write(novo)
    print('Backup: %s' % bak)
    print('Aplicado: shim 0.40 rad/s, smoother 0.40, Collision Monitor com giro_esq/giro_dir.')
    print('\nAgora compile e reinicie o Nav2:')
    print('  cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash')
    print('Confira depois de subir:')
    print('  ros2 param get /controller_server FollowPath.rotate_to_heading_angular_vel   # 0.4')
    print('  ros2 param get /velocity_smoother max_velocity                               # [0.08, 0.08, 0.4]')
    print('  ros2 param get /collision_monitor VelocityPolygonStop.velocity_polygons      # ... giro_esq, giro_dir')
    return 0


if __name__ == '__main__':
    sys.exit(main())
