"""camera_cubo: webcam + AprilTag (abre uma janela). Publica em /cubo_detectado.

    ros2 launch garra_coleta camera_cubo.launch.py
    ros2 launch garra_coleta camera_cubo.launch.py auto:=false topico:=/pegar_cubo  # direto na garra
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction

sys.path.insert(0, os.path.dirname(__file__))
from coleta_comum import SIM, argumentos, no, valores  # noqa: E402

ARGS = [
    ('topico', '/cubo_detectado', 'onde publicar (o alcance_cubo.py escuta /cubo_detectado)'),
    ('auto', 'true', 'manda o cubo sozinho quando ele fica parado (sem apertar P)'),
    ('indice', '', 'indice da webcam /dev/videoN (vazio = acha a "Web Camera")'),
    ('id', '', 'ID do AprilTag do cubo (vazio = o mais perto)'),
    ('apoio', 'auto', 'altura (m) da superficie dos cubos: auto = descobre sozinha; 0 = chao; mesa = tampo'),
    ('fator_pnp', '1.11', 'com apoio:=auto: distancia do solvePnP / real (confira o pnp/pixel na imagem)'),
    ('alturas', '0,0.05,0.10,0.15', 'com apoio:=auto: alturas possiveis (m) da superficie dos cubos'),
    ('tag_plataforma', '256', 'ID do tag da plataforma Y (nao e cubo; vazio = desliga)'),
    ('altura_plataforma', '0.174', 'altura (m) do centro do tag da plataforma ate o chao'),
]


from garra_coleta.logs_launch import salvar_logs  # noqa: E402

def montar(context):
    a = valores(context, ARGS)
    extra = ['--topico', a['topico'], '--apoio', a['apoio'],
             '--fator-pnp', a['fator_pnp'], '--alturas', a['alturas']]
    if a['auto'] in SIM:
        extra.append('--auto')
    if a['indice']:
        extra += ['--camera', a['indice']]
    if a['id']:
        extra += ['--id', a['id']]
    if a['tag_plataforma']:
        extra += ['--tag-plataforma', a['tag_plataforma'], '--altura-plataforma', a['altura_plataforma']]
    return [no('camera_cubo', *extra)]


def generate_launch_description():
    return LaunchDescription(salvar_logs('camera_cubo') + argumentos(ARGS) + [OpaqueFunction(function=montar)])
