#!/usr/bin/env python3
"""Coloca o RotationShimController na frente do DWB (FollowPath) no nav2_params.yaml.

O shim gira o robo no lugar ate ficar virado para o caminho (quando o erro passa de 45 graus)
e so entao entrega para o DWB. Objetivo: acabar com o "vai-e-vem" do DWB parado num ponto
(teste de 26/10 08:39-08:40: 85 s tremendo no mesmo lugar com o caminho a ~60 graus).

Uso:
    python3 ativar_rotation_shim.py            # aplica (faz backup antes)
    python3 ativar_rotation_shim.py --desfazer # volta o backup mais recente
"""
import glob
import os
import re
import shutil
import sys
import time

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
MARCA = '# ROTATION_SHIM'


def main():
    if '--desfazer' in sys.argv:
        baks = sorted(glob.glob(ARQ + '.bak_shim_*'))
        if not baks:
            print('Nenhum backup .bak_shim_* encontrado.')
            return 1
        shutil.copy2(baks[-1], ARQ)
        print('Restaurado de %s' % baks[-1])
        return 0
    txt = open(ARQ).read()
    if MARCA in txt:
        print('O RotationShim ja esta aplicado (achei a marca %s). Nada a fazer.' % MARCA)
        return 0
    linhas = txt.split('\n')
    idx = [i for i, l in enumerate(linhas)
           if re.match(r'^\s*plugin:\s*["\']?dwb_core::DWBLocalPlanner["\']?\s*(#.*)?$', l)]
    if len(idx) != 1:
        print('Esperava 1 linha "plugin: dwb_core::DWBLocalPlanner", achei %d. Nao mexi em nada.' % len(idx))
        return 1
    i = idx[0]
    ind = re.match(r'^(\s*)', linhas[i]).group(1)
    novo = [
        ind + 'plugin: "nav2_rotation_shim_controller::RotationShimController"  ' + MARCA,
        ind + 'primary_controller: "dwb_core::DWBLocalPlanner"',
        ind + 'angular_dist_threshold: 0.785        # gira antes se o caminho estiver a mais de 45 graus',
        ind + 'forward_sampling_distance: 0.5       # direcao do caminho medida 0.5 m a frente',
        ind + 'rotate_to_heading_angular_vel: 0.10  # comando (o robo real gira ~2x)',
        ind + 'max_angular_accel: 0.2',
        ind + 'simulate_ahead_time: 1.0             # confere colisao do giro',
    ]
    bak = ARQ + '.bak_shim_' + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(ARQ, bak)
    linhas[i:i + 1] = novo
    open(ARQ, 'w').write('\n'.join(linhas))
    print('Backup: %s' % bak)
    print('Aplicado. Trecho novo:')
    print('\n'.join(novo))
    print('\nReinicie o Nav2 para valer (e confira: ros2 param get /controller_server FollowPath.plugin).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
