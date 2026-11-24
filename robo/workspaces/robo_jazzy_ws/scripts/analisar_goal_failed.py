#!/usr/bin/env python3
import sys
import math
from datetime import datetime

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

pasta = sys.argv[1]

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=pasta,
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

topicos = [
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/cmd_vel"
]

vel = {t: [] for t in topicos}
odom = []
falhas = []
eventos = []

def angulo(q):
    return math.atan2(
        2*(q.w*q.z + q.x*q.y),
        1-2*(q.y*q.y + q.z*q.z)
    )

def delta_angular(a, b):
    return math.atan2(
        math.sin(b-a),
        math.cos(b-a)
    )

while reader.has_next():
    topico, bruto, ns = reader.read_next()

    if topico not in tipos:
        continue

    if topico not in topicos + ["/odom", "/rosout"]:
        continue

    msg = deserialize_message(bruto, tipos[topico])
    t = ns / 1e9

    if topico in topicos:
        v = msg.twist
        vel[topico].append((
            t,
            v.linear.x,
            v.linear.y,
            v.angular.z
        ))

    elif topico == "/odom":
        p = msg.pose.pose
        odom.append((
            t,
            p.position.x,
            p.position.y,
            angulo(p.orientation)
        ))

    elif topico == "/rosout":
        texto = msg.msg

        if "Failed to make progress" in texto:
            falhas.append(t)

        if any(s in texto for s in [
            "Failed to make progress",
            "Goal failed",
            "Robot to stop",
            "No valid trajectories",
            "Aborting handle"
        ]):
            eventos.append((t, texto))

print("\n===== FALHAS ENCONTRADAS =====")

for t in falhas:
    print(datetime.fromtimestamp(t).strftime("%H:%M:%S"))

for t in falhas:
    print("\n" + "="*52)
    print(
        "FALHA:",
        datetime.fromtimestamp(t).strftime("%H:%M:%S")
    )
    print("Janela: 8 segundos anteriores, excluindo os ultimos 0,3s")

    inicio = t - 8
    fim = t - 0.3

    for nome in topicos:
        dados = [
            v for v in vel[nome]
            if inicio <= v[0] < fim
        ]

        print("\nTOPICO:", nome)
        print("Mensagens:", len(dados))

        if not dados:
            continue

        lin = [
            math.hypot(v[1], v[2])
            for v in dados
        ]

        wz = [v[3] for v in dados]

        print("Linear media:", round(sum(lin)/len(lin), 5))
        print("Linear maxima:", round(max(lin), 5))
        print("Angular media abs:", round(
            sum(abs(x) for x in wz)/len(wz), 5
        ))
        print("Giros positivos:", sum(x > 0.01 for x in wz))
        print("Giros negativos:", sum(x < -0.01 for x in wz))
        print("Comandos quase zero:", sum(
            a < 0.003 and abs(b) < 0.003
            for a, b in zip(lin, wz)
        ))

    poses = [
        p for p in odom
        if inicio <= p[0] < fim
    ]

    print("\n===== ODOMETRIA ESTIMADA =====")

    if len(poses) >= 2:
        a, b = poses[0], poses[-1]

        d = math.hypot(
            b[1]-a[1],
            b[2]-a[2]
        )

        giro = math.degrees(
            delta_angular(a[3], b[3])
        )

        print("Deslocamento liquido:", round(d*100, 2), "cm")
        print("Giro liquido:", round(giro, 2), "graus")
    else:
        print("Amostras insuficientes")

    print("\n===== EVENTOS =====")

    for te, texto in eventos:
        if t-8 <= te <= t+2:
            print(
                datetime.fromtimestamp(te).strftime("%H:%M:%S"),
                texto[:180]
            )

print("\nANALISE CONCLUIDA")
print("Odometria estimada nao comprova movimento fisico.")
