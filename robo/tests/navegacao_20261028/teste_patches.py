#!/usr/bin/env python3
"""Testa os scripts de patches/ numa copia falsa do workspace (HOME temporario).

Aplica, em ordem, recuperacao rapida -> giro rapido -> correcao do giro; confere os valores
no YAML, confere que rodar de novo nao muda nada e que --desfazer volta ao original.
Uso: python3 testes/teste_patches.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
PATCHES = os.path.join(AQUI, '..', 'patches')


def rodar(home, script, *args):
    r = subprocess.run([sys.executable, os.path.join(PATCHES, script)] + list(args),
                       env=dict(os.environ, HOME=home), capture_output=True, text=True)
    return r.returncode, r.stdout


def main():
    home = tempfile.mkdtemp()
    ws = os.path.join(home, 'robo_jazzy_ws', 'src', 'robo_navigation')
    os.makedirs(os.path.join(ws, 'config'))
    os.makedirs(os.path.join(ws, 'behavior_trees'))
    par = os.path.join(ws, 'config', 'nav2_params.yaml')
    bt = os.path.join(ws, 'behavior_trees', 'navigate_to_pose_smooth.xml')
    shutil.copy(os.path.join(AQUI, 'dados', 'nav2_params_antes.yaml'), par)
    shutil.copy(os.path.join(AQUI, 'dados', 'navigate_to_pose_smooth_antes.xml'), bt)
    original = open(par).read()

    for script in ('aplicar_recuperacao_rapida.py', 'aplicar_giro_rapido.py', 'corrigir_giro_shim.py'):
        rc, out = rodar(home, script)
        assert rc == 0, (script, out)
        rc, out = rodar(home, script)
        assert rc == 0 and 'Ja aplicado' in out, (script, out)
        print('ok  ', script)

    d = yaml.safe_load(open(par))
    cs = d['controller_server']['ros__parameters']
    assert cs['progress_checker']['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert cs['progress_checker']['movement_time_allowance'] == 6.0
    fp = cs['FollowPath']
    assert fp['rotate_to_heading_angular_vel'] == 0.40 and fp['rotate_to_heading_once'] is True
    assert fp['max_vel_theta'] == 0.14
    vs = d['velocity_smoother']['ros__parameters']
    assert vs['max_velocity'][2] == 0.40 and vs['max_decel'][2] == -1.2
    cm = d['collision_monitor']['ros__parameters']['VelocityPolygonStop']
    assert cm['velocity_polygons'][-2:] == ['giro_esq', 'giro_dir']
    assert 'RECUPERACAO_RAPIDA' in open(bt).read()
    print('ok   valores conferidos no YAML e na arvore')

    for script in ('corrigir_giro_shim.py', 'aplicar_giro_rapido.py', 'aplicar_recuperacao_rapida.py'):
        rc, out = rodar(home, script, '--desfazer')
        assert rc == 0, (script, out)
    assert open(par).read() == original
    assert open(bt).read() == open(os.path.join(AQUI, 'dados', 'navigate_to_pose_smooth_antes.xml')).read()
    print('ok   --desfazer voltou os dois arquivos ao original')
    shutil.rmtree(home)
    print('TUDO OK')


if __name__ == '__main__':
    main()
