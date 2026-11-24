"""Integra shell real com executaveis ROS simulados. Nunca inicia ROS/hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import pytest
ROOT=Path('/home/marcos/robo_jazzy_ws')


def preparar(tmp_path):
    home=tmp_path
    ws=home/'robo_jazzy_ws';bin=home/'bin';bin.mkdir()
    for f in ('ferramentas/iniciar_supervisor_validado.sh','src/robo_navigation/launch/navigation.launch.py','src/robo_navigation/launch/mapping.launch.py'):
        p=ws/f;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/f,p)
    prefix=ws/'install_nav2_principal/robo_navigation'
    for f in ('config/nav2_params_principal.yaml','behavior_trees/navigate_to_pose_principal.xml'):
        p=prefix/'share/robo_navigation'/f;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/'src/robo_navigation'/f,p)
    for f in ('install/setup.bash','install_nav2_principal/setup.bash'):
        p=ws/f;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('export PATH="'+str(bin)+':$PATH"\n')
    mapas=home/'mapas';mapas.mkdir();(mapas/'arena.posegraph').write_bytes(b'teste')
    (mapas/'arena_pontos.yaml').write_text('pontos: {START: {x: 0.2, y: 0.0, yaw: 0.0}}')
    startup=home/'subir_robo.sh';shutil.copy2('/home/marcos/subir_robo.sh',startup)
    ros=bin/'ros2';ros.write_text('''#!/usr/bin/python3
import os,sys,time
from pathlib import Path
a=sys.argv[1:]
with open(os.environ['CHAMADAS'],'a') as f:f.write(' '.join(a)+'\\n')
if a[:3]==['pkg','prefix','robo_navigation']:print(os.environ['PREFIXO'])
elif a[:2]==['lifecycle','get']:
 for _ in range(150):
  if 'nav2_principal:=' in Path(os.environ['CHAMADAS']).read_text():break
  time.sleep(.05)
 print('active [3]')
elif a[:2]==['service','list']:
 if 'mapping.launch.py' in Path(os.environ['CHAMADAS']).read_text():print('/slam_toolbox/deserialize_map')
elif a[:3]==['param','get','/alinhador_passagem']:
 print(('Boolean value is: '+('False' if os.environ.get('BAD_PARAM') else 'True')) if a[3]=='nav2_principal' else 'String value is: observar')
elif a[:2]==['node','list']:
 if not os.environ.get('MISSING_ARBITER') and 'nav2_principal:=true' in Path(os.environ['CHAMADAS']).read_text():print('/arbitro_nav_principal')
elif a[:2]==['run','tf2_ros']:print('Translation: [0, 0, 0]')
# list de services vazio: mapping deve subir, sem instancia preexistente
''');ros.chmod(0o755)
    for name in ('sleep','tail'):
        p=bin/name;p.write_text('#!/bin/sh\n'+('echo alinhador_teste\n' if name=='tail' else '')+'exit 0\n');p.chmod(0o755)
    env=dict(os.environ,HOME=str(home),CHAMADAS=str(home/'calls'),PREFIXO=str(prefix))
    return startup,env,home


@pytest.mark.parametrize('flag,erro,esperado', [('',{},0),('',{'BAD_PARAM':'1'},1),('',{'MISSING_ARBITER':'1'},1),('--legado',{},0)])
def test_subida_real_com_ros_simulado(tmp_path,flag,erro,esperado):
    startup,env,home=preparar(tmp_path);env.update(erro)
    args=['bash',str(startup),'--mapeando','arena']+([flag] if flag else [])
    r=subprocess.run(args,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=25)
    assert r.returncode==esperado,r.stdout
    calls=(home/'calls').read_text()
    if flag=='--legado':
        assert 'nav2_principal:=false' in calls
    elif esperado==0:
        assert 'nav2_principal:=true supervisor_modo:=observar' in calls
        assert 'params_file:='+str(home/'robo_jazzy_ws/install_nav2_principal/robo_navigation/share/robo_navigation/config/nav2_params_principal.yaml') in calls
        assert 'CONFIRMADO:' in r.stdout
    else:
        assert 'ROBO PRONTO' not in r.stdout
    assert 'action send_goal' not in calls and 'topic pub' not in calls
