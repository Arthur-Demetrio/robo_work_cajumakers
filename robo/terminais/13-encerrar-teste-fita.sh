#!/usr/bin/env bash
# Encerramento dos testes de fita/ROS deste robo; preserva a janela que o executa.
set -e
python3 - <<'PY'
import os,signal,time,subprocess
from pathlib import Path
uid=os.getuid()
protected=set()
pid=os.getpid()
while pid > 1:
    protected.add(pid)
    try:
        pid=int(next(line.split()[1] for line in Path(f'/proc/{pid}/status').read_text().splitlines() if line.startswith('PPid:')))
    except (OSError,StopIteration):
        break
nodes={'virtual_wall','tape_keepout','rviz2','tf2_echo','hardware_bridge_node',
       'scan_filter_node','command_imu_odometry_node','async_slam_toolbox_node',
       'static_transform_publisher','controller_server','planner_server','smoother_server',
       'behavior_server','bt_navigator','waypoint_follower','velocity_smoother',
       'collision_monitor','lifecycle_manager','progressive_spin_node','mecanum_escape_node'}
packages={'robo_virtual_wall','robo_hardware','robo_lidar','robo_localization','robo_navigation'}
helpers={'filtro_corpo_scan.py','destravar_node.py','alinhador_passagem.py','suavizador_nav.py'}
targets=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit() or int(entry.name) in protected:
        continue
    try:
        if entry.stat().st_uid!=uid:
            continue
        args=entry.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
        args=[a for a in args if a]
        if not args:
            continue
        executable=Path(args[0]).name
        # Match executable/script positions, never arbitrary command text.
        program=Path(args[1]).name if executable.startswith('python') and len(args)>1 else executable
        launch=False
        if program=='ros2' and 'launch' in args:
            index=args.index('launch')
            launch=index+1<len(args) and args[index+1] in packages
        helper=(program in helpers and any('/robo_navigation/scripts/' in a for a in args[1:2]))
        if program in nodes or helper or launch:
            targets.append((int(entry.name),program))
    except (OSError,ValueError):
        continue
for target,program in targets:
    if program=='hardware_bridge_node':
        continue
    try:
        os.kill(target,signal.SIGINT)
        print(f'Encerrando {program} ({target})')
    except ProcessLookupError:
        pass
time.sleep(2)
for target,program in targets:
    if program=='hardware_bridge_node':
        try:
            os.kill(target,signal.SIGINT)
            print(f'Encerrando {program} ({target})')
        except ProcessLookupError:
            pass
# Close only windows created by the test scripts, excluding this terminal.
try:
    windows=subprocess.run(['wmctrl','-l'],capture_output=True,text=True,check=True).stdout
    own_window=os.environ.get('WINDOWID','')
    prefixes=('Fita - ','Base fita - ','Validacao fita - ','Fita Nav2 - ')
    for line in windows.splitlines():
        fields=line.split(None,3)
        if len(fields)<4 or not fields[3].startswith(prefixes):
            continue
        if own_window and int(fields[0],16)==int(own_window,0):
            continue
        subprocess.run(['wmctrl','-ic',fields[0]],check=False)
except (OSError,subprocess.CalledProcessError,ValueError):
    print('Nao foi possivel acessar as janelas. Os sinais aos processos foram enviados.')
print('Encerramento solicitado. Nenhum objetivo ou comando de deslocamento foi enviado.')
PY
