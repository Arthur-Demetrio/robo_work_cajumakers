import math
import time
import statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node("diagnostico_geometrico_traseiro")

# Geometria em base_link
LIDAR_X = 0.330
LIDAR_Y = 0.005

FRENTE = 0.365
TRASEIRA = -0.205
MEIA_LARGURA = 0.315

dados = {
    "/scan_raw": [],
    "/scan_front": []
}

def callback(topico):
    def receber(msg):
        if msg.header.frame_id != "laser_raw":
            print("Frame inesperado:", msg.header.frame_id)
            return

        dentro = []
        atras = []
        outros = []

        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment

            diferenca = abs(
                abs(math.atan2(math.sin(ang), math.cos(ang)))
                - math.pi
            )

            if diferenca > math.radians(35):
                continue

            if not (
                math.isfinite(r)
                and msg.range_min <= r <= msg.range_max
            ):
                continue

            x = LIDAR_X + r * math.cos(ang)
            y = LIDAR_Y + r * math.sin(ang)

            ponto = (
                round(x, 3),
                round(y, 3),
                round(math.degrees(ang), 1),
                round(r, 3)
            )

            if (
                TRASEIRA <= x <= FRENTE
                and abs(y) <= MEIA_LARGURA
            ):
                dentro.append(ponto)

            elif (
                x < TRASEIRA
                and abs(y) <= MEIA_LARGURA + 0.05
            ):
                atras.append(ponto)

            else:
                outros.append(ponto)

        dados[topico].append((dentro, atras, outros))

    return receber

subs = [
    n.create_subscription(
        LaserScan, topico, callback(topico),
        qos_profile_sensor_data
    )
    for topico in dados
]

print("Analisando retornos traseiros por 5 segundos...")

inicio = time.monotonic()

while time.monotonic() - inicio < 5:
    rclpy.spin_once(n, timeout_sec=0.2)

for topico, scans in dados.items():
    print("\n========", topico, "========")
    print("Scans:", len(scans))

    if not scans:
        continue

    for indice, nome in enumerate([
        "Dentro do footprint",
        "Atras da traseira",
        "Outros pontos"
    ]):
        contagens = [len(s[indice]) for s in scans]
        print(
            nome,
            "| mediana por scan:",
            statistics.median(contagens)
        )

    ultimo = scans[-1]
    externos = ultimo[1]

    if externos:
        print("\nPontos externos mais proximos:")
        for x, y, ang, r in sorted(
            externos,
            key=lambda p: -p[0]
        )[:5]:
            folga_x = TRASEIRA - x
            print(
                f"x={x:+.3f} y={y:+.3f} "
                f"angulo={ang:+.1f} "
                f"folga_longitudinal={folga_x:.3f}m"
            )
    else:
        print(
            "Nenhum retorno externo atras do "
            "footprint no ultimo scan."
        )
        print(
            "Isso NAO confirma espaco livre."
        )

n.destroy_node()
rclpy.shutdown()
