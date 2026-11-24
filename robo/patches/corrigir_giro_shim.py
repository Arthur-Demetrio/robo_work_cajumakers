#!/usr/bin/env python3
"""Correcao do giro rapido (prova2, 26/10): o RotationShim e o DWB brigavam no fim do caminho.

O que aconteceu (11:00:28-11:01:13): o robo chegou na pose de preparo (0.22, -0.21) e ficou
45 s girando para um lado e para o outro. O BT manda caminho novo 1x por segundo e, com
rotate_to_heading_once: false, o shim volta a valer a cada caminho novo. Perto do goal o
"rumo do caminho" do shim aponta para tras (o ponto final esta a 2 cm), entao ele girava a
0.40 para um lado; o DWB devolvia para o angulo do goal; 1 s depois o shim de novo...

Correcao (so no nav2_params.yaml):
  1) FollowPath: rotate_to_heading_once: true -> o shim gira so uma vez por goal (no comeco),
     e o resto e do DWB.
  2) velocity_smoother: freio do giro -0.5 -> -1.2 rad/s^2 (o 0.40 levava 0.8 s para parar,
     ~10 graus a mais de giro depois do comando zero).

Uso:
    python3 corrigir_giro_shim.py            # aplica (backup antes)
    python3 corrigir_giro_shim.py --desfazer # volta o backup mais recente
"""
import glob
import os
import shutil
import sys
import time

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
MARCA = 'GIRO_UMA_VEZ'
SUF = '.bak_giro2_'

TROCAS = [
    ('      closed_loop: false                   # usa o ultimo comando (a odom nao mede bem o giro)\n',
     '      closed_loop: false                   # usa o ultimo comando (a odom nao mede bem o giro)\n'
     '      rotate_to_heading_once: true         # GIRO_UMA_VEZ: so no comeco de cada goal\n', 1),
    ('    max_decel:\n    - -0.5\n    - -0.5\n    - -0.5\n',
     '    max_decel:\n    - -0.5\n    - -0.5\n    - -1.2  # GIRO_UMA_VEZ: para o giro rapido em ~0.3 s\n', 1),
]


def main():
    if '--desfazer' in sys.argv:
        baks = sorted(glob.glob(ARQ + SUF + '*'))
        if not baks:
            print('Nenhum backup %s* encontrado.' % SUF)
            return 1
        shutil.copy2(baks[-1], ARQ)
        print('Restaurado de %s' % baks[-1])
        return 0
    txt = open(ARQ).read()
    if MARCA in txt:
        print('Ja aplicado (achei a marca %s). Nada a fazer.' % MARCA)
        return 0
    if 'GIRO_RAPIDO' not in txt:
        print('O aplicar_giro_rapido.py nao foi aplicado neste arquivo. Nada alterado.')
        return 1
    for velho, novo, n in TROCAS:
        if txt.count(velho) != n:
            print('Trecho esperado nao encontrado (%d vezes), nada alterado:\n%s' % (txt.count(velho), velho))
            return 1
        txt = txt.replace(velho, novo)
    bak = ARQ + SUF + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(ARQ, bak)
    open(ARQ, 'w').write(txt)
    print('Backup: %s' % bak)
    print('Aplicado: rotate_to_heading_once: true, freio do giro -1.2.')
    print('\nAgora compile e reinicie o Nav2:')
    print('  cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash')
    print('Confira depois de subir:')
    print('  ros2 param get /controller_server FollowPath.rotate_to_heading_once   # True')
    print('  ros2 param get /velocity_smoother max_decel                          # [-0.5, -0.5, -1.2]')
    return 0


if __name__ == '__main__':
    sys.exit(main())
