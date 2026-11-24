#!/usr/bin/env python3
"""Area aberta sem zigue-zague (teste curva5, 28/10): o DWB passa a andar so para frente e girar.

O que aconteceu (10:50:50-10:51:15, 1 m em 25 s; 10:54:03-10:54:09): o DWB escolhia a cada
0,5 s um vx/vy pequeno diferente (+0.027, -0.053, vy +-0.053...). Com o PWM minimo da ponte,
QUALQUER comando desses anda ~0,15-0,2 m/s de verdade, entao cada troca de sinal virava um
tranco para o outro lado: o robo ia e voltava em vez de seguir o caminho.

Mudanca (so o FollowPath do controller_server no nav2_params.yaml):
  - sem lateral:  min_vel_y 0, max_vel_y 0, vy_samples 1
  - sem re:       min_vel_x 0 (o RotationShim ja gira para o rumo do caminho antes)
O andar de lado continua para o alinhador, o mecanum_escape e o velocity_smoother: nao mudam.

Uso:
    python3 dwb_sem_lateral.py            # aplica (backup antes)
    python3 dwb_sem_lateral.py --desfazer # volta o backup mais recente
"""
import glob
import os
import re
import shutil
import sys
import time

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
MARCA = 'DWB_SEM_LATERAL'
SUF = '.bak_dwb_'

# (parametro, valor novo) dentro do bloco FollowPath
NOVOS = [('min_vel_x', '0.0'), ('min_vel_y', '0.0'), ('max_vel_y', '0.0'), ('vy_samples', '1')]


def bloco_followpath(txt):
    """Inicio e fim (indices) do bloco FollowPath do controller_server."""
    m = re.search(r'^controller_server:\s*$', txt, re.M)
    if not m:
        return None
    f = re.search(r'^(\s+)FollowPath:\s*$', txt[m.end():], re.M)
    if not f:
        return None
    ini = m.end() + f.end()
    ind = len(f.group(1))
    # o bloco termina na proxima linha com indentacao <= a do "FollowPath:"
    fim = len(txt)
    for linha in re.finditer(r'^( *)\S.*$', txt[ini:], re.M):
        if len(linha.group(1)) <= ind:
            fim = ini + linha.start()
            break
    return ini, fim


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
    b = bloco_followpath(txt)
    if b is None:
        print('Nao achei controller_server -> FollowPath no arquivo. Nada alterado.')
        return 1
    ini, fim = b
    bloco = txt[ini:fim]
    for nome, valor in NOVOS:
        padrao = re.compile(r'^(\s+)%s:\s*(\S+)[^\n]*$' % nome, re.M)
        achados = padrao.findall(bloco)
        if len(achados) != 1:
            print('Esperava 1 linha "%s:" no FollowPath, achei %d. Nada alterado.' % (nome, len(achados)))
            return 1
        antigo = achados[0][1]
        bloco = padrao.sub(lambda m: '%s%s: %s  # %s (era %s)' % (m.group(1), nome, valor, MARCA, antigo), bloco)
        print('  %-11s %s -> %s' % (nome, antigo, valor))
    bak = ARQ + SUF + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(ARQ, bak)
    open(ARQ, 'w').write(txt[:ini] + bloco + txt[fim:])
    print('Backup: %s' % bak)
    print('\nAgora compile e reinicie o Nav2:')
    print('  cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash')
    print('Confira depois de subir:')
    print('  ros2 param get /controller_server FollowPath.max_vel_y    # 0.0')
    print('  ros2 param get /controller_server FollowPath.min_vel_x    # 0.0')
    return 0


if __name__ == '__main__':
    sys.exit(main())
