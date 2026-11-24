#!/usr/bin/env python3
"""Faz o Collision Monitor e o costmap local usarem /scan_nav (scan sem os pontos de dentro do robo).

Troca as 3 linhas 'topic: /scan_front' do nav2_params.yaml (costmap local, obstacle_layer do
global, que esta desligado, e Collision Monitor) por '/scan_nav'. O SLAM e o alinhador nao mudam.
Precisa do filtro_corpo_scan.py rodando.

Uso:
    python3 usar_scan_nav.py            # aplica (backup antes)
    python3 usar_scan_nav.py --desfazer # volta o backup mais recente
"""
import glob
import os
import shutil
import sys
import time

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
SUF = '.bak_scannav_'
VELHO = 'topic: /scan_front\n'
NOVO = 'topic: /scan_nav  # FILTRO_CORPO: sem os pontos de dentro do robo\n'


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
    if 'FILTRO_CORPO' in txt:
        print('Ja aplicado. Nada a fazer.')
        return 0
    n = txt.count(VELHO)
    if n != 3:
        print('Esperava 3 linhas "topic: /scan_front", achei %d. Nada alterado.' % n)
        return 1
    bak = ARQ + SUF + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(ARQ, bak)
    open(ARQ, 'w').write(txt.replace(VELHO, NOVO))
    print('Backup: %s' % bak)
    print('Collision Monitor e costmap agora usam /scan_nav.')
    print('\nCompile e reinicie o Nav2 (com o filtro_corpo_scan.py rodando):')
    print('  cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash')
    print('Confira:  ros2 param get /collision_monitor scan.topic   # /scan_nav')
    return 0


if __name__ == '__main__':
    sys.exit(main())
