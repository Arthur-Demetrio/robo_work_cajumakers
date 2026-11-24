#!/usr/bin/env python3
import sys
import math
import time
import rclpy
from pathlib import Path

sys.path.insert(0, str(Path.home() / "robo_jazzy_ws/scripts"))
from geometry_approach_align import Align, yaw

# Footprint real + 3 cm de margem
CORPO = [
    (0.395, 0.345),
    (0.395, -0.345),
    (-0.235, -0.345),
    (-0.235, 0.345)
]

def interseta(poly, cell):
    eixos = [(1, 0), (0, 1)]

    for a, b in zip(poly, poly[1:] + poly[:1]):
        eixos.append((-(b[1]-a[1]), b[0]-a[0]))

    for ax, ay in eixos:
        p = [x*ax + y*ay for x, y in poly]
        q = [x*ax + y*ay for x, y in cell]

        if max(p) < min(q) or max(q) < min(p):
            return False
    return True

rclpy.init()
n = Align()

try:
    limite = time.monotonic() + 12
    ultimo = "Sem sensores"

    while time.monotonic() < limite:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            n.guard()
            break
        except Exception as e:
            ultimo = str(e)
    else:
        raise RuntimeError(ultimo)

    cm = n.getLocalCostmap()
    if cm is None:
        raise RuntimeError("Costmap local indisponivel")

    m = cm.metadata
    w, h, res = m.size_x, m.size_y, m.resolution

    if res <= 0 or len(cm.data) != w*h:
        raise RuntimeError("Costmap invalido")

    t = n.tf(cm.header.frame_id)
    cx = t.transform.translation.x
    cy = t.transform.translation.y
    yaw0 = yaw(t.transform.rotation)

    ox = m.origin.position.x
    oy = m.origin.position.y
    origem = yaw(m.origin.orientation)

    co = math.cos(origem)
    so = math.sin(origem)

    def grade(wx, wy):
        dx, dy = wx-ox, wy-oy
        return co*dx+so*dy, -so*dx+co*dy

    print("===== COSTMAP LOCAL =====")
    print("Frame:", cm.header.frame_id)
    print("Dimensoes:", w, "x", h)
    print("Resolucao:", res, "m")
    print("Orientacao:", round(math.degrees(yaw0), 2), "graus")

    def avaliar(recuo, giro):
        poses = [(recuo*k/4, 0.0) for k in range(5)]
        passos = max(1, math.ceil(abs(giro)/0.5))

        for k in range(1, passos+1):
            poses.append((recuo, giro*k/passos))

        encontrados = {253: set(), 254: set(), 255: set()}
        inflados = set()
        primeiro = {}
        fora = False

        for dx, ang_delta in poses:
            a = yaw0 + math.radians(ang_delta)

            px = cx + math.cos(yaw0)*dx
            py = cy + math.sin(yaw0)*dx

            poly = []

            for bx, by in CORPO:
                wx = px + math.cos(a)*bx - math.sin(a)*by
                wy = py + math.sin(a)*bx + math.cos(a)*by
                poly.append(grade(wx, wy))

            xmin = min(p[0] for p in poly)
            xmax = max(p[0] for p in poly)
            ymin = min(p[1] for p in poly)
            ymax = max(p[1] for p in poly)

            if xmin < 0 or ymin < 0 or xmax >= w*res or ymax >= h*res:
                fora = True
                break

            i0 = max(0, math.floor(xmin/res)-1)
            i1 = min(w-1, math.floor(xmax/res)+1)
            j0 = max(0, math.floor(ymin/res)-1)
            j1 = min(h-1, math.floor(ymax/res)+1)

            for j in range(j0, j1+1):
                for i in range(i0, i1+1):
                    quadro = [
                        (i*res, j*res),
                        ((i+1)*res, j*res),
                        ((i+1)*res, (j+1)*res),
                        (i*res, (j+1)*res)
                    ]

                    if not interseta(poly, quadro):
                        continue

                    custo = int(cm.data[j*w+i])

                    if custo in encontrados:
                        encontrados[custo].add((i, j))
                        if custo not in primeiro:
                            primeiro[custo] = (dx, ang_delta, i, j)
                    elif 0 < custo < 253:
                        inflados.add((i, j))

        print(f"\nRECUO={abs(recuo)*100:.0f} cm | GIRO={giro:+.0f} graus")
        print("Custo 253:", len(encontrados[253]), "celulas")
        print("Custo 254:", len(encontrados[254]), "celulas")
        print("Desconhecidas 255:", len(encontrados[255]), "celulas")
        print("Inflacao 1 a 252:", len(inflados), "celulas")

        if fora:
            print("BLOQUEIO: footprint sai do costmap")

        for custo in (253, 254, 255):
            if custo in primeiro:
                dx, ang, i, j = primeiro[custo]
                print(
                    f"Primeiro custo {custo}: "
                    f"recuo={abs(dx)*100:.1f}cm "
                    f"giro={ang:+.1f}graus "
                    f"celula=({i},{j})"
                )

    avaliar(0.0, +3)
    avaliar(0.0, -3)
    avaliar(-0.02, 0)
    avaliar(-0.02, +3)
    avaliar(-0.02, -3)

    print("\nDIAGNOSTICO CONCLUIDO — SEM COMANDOS AOS MOTORES")

except Exception as e:
    print("DIAGNOSTICO BLOQUEADO:", e)

finally:
    n.destroyNode()
    rclpy.shutdown()
