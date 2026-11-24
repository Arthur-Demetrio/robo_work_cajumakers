import math
import bisect
from pathlib import Path
from datetime import datetime

import yaml
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

home = Path.home()

bag = home / (
    "robo_jazzy_ws/logs/"
    "area_aberta_deadband_20261019_090414/rosbag"
)

config = home / (
    "robo_jazzy_ws/src/robo_navigation/"
    "config/nav2_params.yaml"
)

params = yaml.safe_load(config.read_text())
cm = params["collision_monitor"]["ros__parameters"]
vp = cm["VelocityPolygonStop"]

ordem = vp["velocity_polygons"]

def selecionar(v):
    x = v.linear.x
    y = v.linear.y
    theta = v.angular.z

    magnitude = math.hypot(x, y)
    direcao = math.atan2(y, x) if magnitude > 0 else 0.0

    for nome in ordem:
        p = vp[nome]

        if not (
            p["linear_min"] <= magnitude <= p["linear_max"]
            and p["theta_min"] <= theta <= p["theta_max"]
        ):
            continue

        inicio = p.get("direction_start_angle", -math.pi)
        fim = p.get("direction_end_angle", math.pi)

        if inicio <= fim:
            valido = inicio <= direcao <= fim
        else:
            valido = direcao >= inicio or direcao <= fim

        if valido:
            return nome

    return "SEM_CORRESPONDENCIA"

reader = rosbag2_py.SequentialReader()

reader.open(
    rosbag2_py.StorageOptions(
        uri=str(bag),
        storage_id="mcap"
    ),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
)

tipos = {
    t.name: get_message(t.type)
    for t in reader.get_all_topics_and_types()
}

comandos = []
eventos = []

while reader.has_next():
    topico, bruto, ns = reader.read_next()

    if topico not in (
        "/cmd_vel_smoothed",
        "/rosout"
    ):
        continue

    msg = deserialize_message(bruto, tipos[topico])
    t = ns / 1e9

    if topico == "/cmd_vel_smoothed":
        comandos.append((t, msg.twist))

    elif (
        "Robot to stop due to VelocityPolygonStop"
        in msg.msg
    ):
        eventos.append(t)

tempos = [x[0] for x in comandos]

print("===== POLIGONOS PROVAVEIS NOS STOPS =====")
print("Eventos:", len(eventos))

for t in eventos:
    i = bisect.bisect_right(tempos, t) - 1
    hora = datetime.fromtimestamp(t).strftime("%H:%M:%S")

    if i < 0 or t-tempos[i] > 0.5:
        print(hora, "SEM COMANDO RECENTE")
        continue

    v = comandos[i][1]
    nome = selecionar(v)

    print(
        f"{hora} | {nome:18s} "
        f"| vx={v.linear.x:+.4f} "
        f"vy={v.linear.y:+.4f} "
        f"wz={v.angular.z:+.4f}"
    )

print("\nAnalise concluida. Nenhum movimento enviado.")
