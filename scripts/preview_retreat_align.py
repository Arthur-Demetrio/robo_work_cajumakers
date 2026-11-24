import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

LX, LY = 0.330, 0.005

XMIN, XMAX = -0.205, 0.365
YMIN, YMAX = -0.315, 0.315

PAD = 0.03

rclpy.init()
node = Node("geometry_retreat_preview")

scans = {
    "/scan_raw": None,
    "/scan_safety": None
}

contagens = dict.fromkeys(scans, 0)

def callback(topic):
    def receber(msg):
        scans[topic] = msg
        contagens[topic] += 1
    return receber

subs = [
    node.create_subscription(
        LaserScan, topic, callback(topic),
        qos_profile_sensor_data
    )
    for topic in scans
]

print("Coletando leituras por 5 segundos...")

fim = time.monotonic() + 5

while rclpy.ok() and time.monotonic() < fim:
    rclpy.spin_once(node, timeout_sec=0.2)

print("\n===== PREVIA SEM MOVIMENTO =====")
print("Scans:", contagens)

msg = scans["/scan_safety"]

if msg is None or msg.header.frame_id != "laser_raw":
    print("BLOQUEADO: scan_safety indisponivel")
else:
    idade = (
        node.get_clock().now() -
        Time.from_msg(msg.header.stamp)
    ).nanoseconds / 1e9

    print("Idade do scan:", round(idade, 3), "s")

    if idade < -0.25 or idade > 0.7:
        print("BLOQUEADO: scan desatualizado")

    else:
        pontos = []

        for i, r in enumerate(msg.ranges):
            if not (
                math.isfinite(r)
                and msg.range_min <= r <= msg.range_max
            ):
                continue

            ang = msg.angle_min + i * msg.angle_increment

            x = LX + r * math.cos(ang)
            y = LY + r * math.sin(ang)

            pontos.append((x, y))

        internos = [
            p for p in pontos
            if XMIN < p[0] < XMAX
            and YMIN < p[1] < YMAX
        ]

        externos = [
            p for p in pontos
            if p not in internos
        ]

        print("\nPontos validos:", len(pontos))
        print("Pontos dentro do footprint:", len(internos))

        for x, y in internos[:5]:
            print(
                f"  PONTO INTERNO "
                f"x={x:+.3f} y={y:+.3f}"
            )

        traseiros = [
            (XMIN-x, x, y)
            for x, y in externos
            if x < XMIN and abs(y) <= YMAX+0.05
        ]

        if traseiros:
            d, x, y = min(traseiros)
            print(
                f"\nObstaculo traseiro: "
                f"folga longitudinal={d:.3f}m "
                f"x={x:+.3f} y={y:+.3f}"
            )

        def verificar(recuo, giro):
            # Primeiro recuar; depois girar.
            poses = [
                (recuo*k/4, 0.0)
                for k in range(5)
            ]

            etapas = max(1, math.ceil(abs(giro)/0.5))

            poses += [
                (recuo, giro*k/etapas)
                for k in range(1, etapas+1)
            ]

            conflito = None

            for dx, graus in poses:
                a = math.radians(graus)
                c = math.cos(a)
                s = math.sin(a)

                for x, y in externos:
                    px = x - dx

                    bx = c*px + s*y
                    by = -s*px + c*y

                    if (
                        XMIN-PAD <= bx <= XMAX+PAD
                        and YMIN-PAD <= by <= YMAX+PAD
                    ):
                        conflito = (dx, graus, x, y)
                        break

                if conflito:
                    break

            print(
                f"\nRECUO={-recuo*100:.0f}cm "
                f"GIRO={giro:+.0f} graus"
            )

            if conflito:
                dx, graus, x, y = conflito
                print("INTERSECAO PREVISTA")
                print(
                    f"  No recuo={-dx*100:.1f}cm "
                    f"giro={graus:+.1f} graus"
                )
                print(
                    f"  Ponto x={x:+.3f} "
                    f"y={y:+.3f}"
                )
            else:
                print(
                    "Sem intersecao nos pontos "
                    "externos observados"
                )

        verificar(0.0, +3)
        verificar(0.0, -3)

        verificar(-0.02, 0)

        verificar(-0.02, +3)
        verificar(-0.02, -3)

        print("\nPREVIA CONCLUIDA")
        print("NENHUM MOVIMENTO AUTORIZADO")
        print(
            "Ainda faltam costmap, oclusoes "
            "e distancia de frenagem."
        )

node.destroy_node()
rclpy.shutdown()
