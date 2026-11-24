"""Grava a saida dos nos de cada `ros2 launch` em <repositorio>/logs/<data>_<nome>/ (fora do git).

Usado pelos launchers da garra e da coleta:
    from garra_coleta.logs_launch import salvar_logs
    return LaunchDescription(salvar_logs('coleta') + [...])

Na pasta ficam:
  <no>.log   tudo o que o no escreveu no terminal (stdout e stderr), com a hora de cada linha;
  *.log      os logs do rcl/rclpy dos nos (ROS_LOG_DIR aponta para a mesma pasta).
A tela continua mostrando tudo como antes. GARRA_LOGS=<pasta> troca a pasta base.
Um launch incluido por outro (coleta.launch.py -> pegar_cubo.launch.py) usa a pasta de quem incluiu.
"""

import os
import time
from pathlib import Path

from launch.actions import LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessIO

_pasta = None


def pasta_base():
    env = os.environ.get('GARRA_LOGS')
    if env:
        return Path(env).expanduser()
    for d in Path(__file__).resolve().parents:
        if (d / '.git').exists():
            return d / 'logs'
    return Path.home() / '.ros' / 'log' / 'garra'


def salvar_logs(nome):
    """Acoes para colocar no inicio do LaunchDescription. Na segunda chamada (launch incluido): []."""
    global _pasta
    if _pasta is not None:
        return []
    _pasta = pasta_base() / time.strftime(f'%Y-%m-%d_%H-%M-%S_{nome}')
    _pasta.mkdir(parents=True, exist_ok=True)
    arquivos = {}

    def escrever(evento):
        arq = arquivos.get(evento.process_name)
        if arq is None:
            arq = open(_pasta / f'{evento.process_name}.log', 'a', encoding='utf-8', buffering=1)
            arquivos[evento.process_name] = arq
        hora = time.strftime('%H:%M:%S')
        for linha in evento.text.decode(errors='replace').splitlines():
            arq.write(f'{hora} {linha}\n')

    return [
        SetEnvironmentVariable('ROS_LOG_DIR', str(_pasta)),
        RegisterEventHandler(OnProcessIO(on_stdout=escrever, on_stderr=escrever)),
        LogInfo(msg=f'Logs desta execucao em {_pasta}'),
    ]
