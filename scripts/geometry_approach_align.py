#!/usr/bin/env python3
import argparse, math, time, sys
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

rad = math.radians
deg = math.degrees

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw(q):
    return math.atan2(
        2*(q.w*q.z+q.x*q.y),
        1-2*(q.y*q.y+q.z*q.z)
    )

class Align(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='geometry_align')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.scan_age = 999.
        self.last_scan = 0.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_front',
            self.on_scan, qos_profile_sensor_data
        )
        self.moving = False

    def on_scan(self, msg):
        self.last_scan = time.monotonic()
        self.scan_age = (
            self.get_clock().now() -
            Time.from_msg(msg.header.stamp)
        ).nanoseconds/1e9

    def tf(self, target='map'):
        t = self.buf.lookup_transform(
            target, 'base_link', Time()
        )
        dt = (
            self.get_clock().now() -
            Time.from_msg(t.header.stamp)
        ).nanoseconds/1e9

        if not -0.25 <= dt <= 0.6:
            raise RuntimeError(f'TF antigo: {dt:.2f}s')
        return t

    def active(self, node):
        c = self.create_client(
            GetState, '/'+node+'/get_state'
        )
        if not c.wait_for_service(timeout_sec=2.):
            raise RuntimeError(
                'Lifecycle indisponivel: '+node
            )
        f = c.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            self, f, timeout_sec=2.
        )
        if (not f.done() or f.result() is None
                or f.result().current_state.id != 3):
            raise RuntimeError('No nao ativo: '+node)

    def guard(self):
        if (time.monotonic()-self.last_scan > 0.7
                or not -0.25 <= self.scan_age <= 0.7):
            raise RuntimeError(
                'LiDAR ausente ou timestamp divergente'
            )

        for n in (
            'collision_monitor', 'velocity_smoother',
            'bt_navigator', 'controller_server'
        ):
            self.active(n)

        self.tf()


    def clearance_spin(self, step_rad):
        self.guard()

        if abs(step_rad) > math.radians(5.01):
            raise RuntimeError("PASSO_MAIOR_QUE_5_GRAUS")

        cm = self.getLocalCostmap()

        if cm is None:
            raise RuntimeError("COSTMAP_INDISPONIVEL")

        idade = (
            self.get_clock().now() -
            Time.from_msg(cm.header.stamp)
        ).nanoseconds / 1e9

        if not -0.25 <= idade <= 1.5:
            raise RuntimeError(
                f"COSTMAP_DESATUALIZADO: {idade:.2f}s"
            )

        m = cm.metadata
        res = m.resolution
        w = m.size_x
        h = m.size_y

        if res <= 0 or w <= 0 or h <= 0:
            raise RuntimeError("DIMENSOES_INVALIDAS")

        if len(cm.data) != w*h:
            raise RuntimeError("COSTMAP_INCOMPLETO")

        t = self.tf(cm.header.frame_id)

        cx = t.transform.translation.x
        cy = t.transform.translation.y
        yaw0 = yaw(t.transform.rotation)

        ox = m.origin.position.x
        oy = m.origin.position.y
        origem_yaw = yaw(m.origin.orientation)

        co = math.cos(origem_yaw)
        so = math.sin(origem_yaw)

        def para_grade(wx, wy):
            dx = wx - ox
            dy = wy - oy
            return (
                co*dx + so*dy,
                -so*dx + co*dy
            )

        # Footprint real + margem de 3 cm
        pad = 0.03
        corpo = [
            (0.365+pad, 0.315+pad),
            (0.365+pad, -0.315-pad),
            (-0.205-pad, -0.315-pad),
            (-0.205-pad, 0.315+pad)
        ]

        def intersecta(poly, quadro):
            eixos = [(1.0, 0.0), (0.0, 1.0)]

            for k in range(len(poly)):
                a = poly[k]
                b = poly[(k+1) % len(poly)]
                ex = b[0] - a[0]
                ey = b[1] - a[1]
                eixos.append((-ey, ex))

            for ax, ay in eixos:
                pp = [x*ax+y*ay for x,y in poly]
                qq = [x*ax+y*ay for x,y in quadro]

                if max(pp) < min(qq) or max(qq) < min(pp):
                    return False

            return True

        etapas = max(
            1,
            math.ceil(
                abs(step_rad)/math.radians(0.5)
            )
        )

        print(
            f"Simulando microgiro: "
            f"{math.degrees(step_rad):+.2f} graus"
        )
        print(
            f"Costmap: {w}x{h}, "
            f"resolucao={res:.3f}m"
        )

        for k in range(etapas+1):
            delta = step_rad*k/etapas
            ang = yaw0 + delta

            ca = math.cos(ang)
            sa = math.sin(ang)

            poligono = []

            for px, py in corpo:
                wx = cx + ca*px - sa*py
                wy = cy + sa*px + ca*py
                poligono.append(para_grade(wx, wy))

            xmin = min(x for x,y in poligono)
            xmax = max(x for x,y in poligono)
            ymin = min(y for x,y in poligono)
            ymax = max(y for x,y in poligono)

            if (
                xmin < 0 or ymin < 0
                or xmax >= w*res or ymax >= h*res
            ):
                raise RuntimeError(
                    f"FOOTPRINT_FORA_DO_COSTMAP "
                    f"em {math.degrees(delta):+.1f} graus"
                )

            i0 = max(0, math.floor(xmin/res)-1)
            i1 = min(w-1, math.floor(xmax/res)+1)
            j0 = max(0, math.floor(ymin/res)-1)
            j1 = min(h-1, math.floor(ymax/res)+1)

            for j in range(j0, j1+1):
                for i in range(i0, i1+1):
                    valor = cm.data[j*w+i]

                    # 253: inscribed
                    # 254: lethal
                    # 255: desconhecido
                    if valor < 253:
                        continue

                    quadro = [
                        (i*res, j*res),
                        ((i+1)*res, j*res),
                        ((i+1)*res, (j+1)*res),
                        (i*res, (j+1)*res)
                    ]

                    if intersecta(poligono, quadro):
                        raise RuntimeError(
                            "MICROGIRO_BLOQUEADO: "
                            f"angulo={math.degrees(delta):+.1f}graus, "
                            f"custo={valor}, celula=({i},{j})"
                        )

        print(
            "MICROGIRO_POSSIVEL_NO_COSTMAP: "
            "footprint livre em todas as etapas"
        )
        return True


def pose(nav, x, y, angle):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(angle/2)
    p.pose.orientation.w = math.cos(angle/2)
    return p

def finish(nav, name, timeout):
    t0 = time.monotonic()

    while rclpy.ok() and not nav.isTaskComplete():
        if (time.monotonic()-t0 > timeout
                or time.monotonic()-nav.last_scan > 0.8):
            nav.cancelTask()
            nav.isTaskComplete()
            nav.moving = False
            raise RuntimeError(
                f'{name}: timeout ou LiDAR sem dados'
            )

    nav.moving = False

    if nav.getResult() != TaskResult.SUCCEEDED:
        raise RuntimeError(f'{name}: acao falhou')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage-x', type=float)
    ap.add_argument('--stage-y', type=float)
    ap.add_argument('--axis-deg', type=float, required=True)
    ap.add_argument('--only-align', action='store_true')
    ap.add_argument('--execute', action='store_true')
    args = ap.parse_args()

    if not args.only_align and (
        args.stage_x is None or args.stage_y is None
    ):
        ap.error(
            'Informe stage-x e stage-y, '
            'ou use --only-align'
        )

    rclpy.init()
    n = Align()

    try:
        print('Aguardando scan valido do LiDAR...')

        prazo = time.monotonic() + 10.0

        while time.monotonic() < prazo:
            rclpy.spin_once(n, timeout_sec=0.2)

            idade_recebimento = time.monotonic() - n.last_scan

            if (
                n.last_scan > 0
                and idade_recebimento <= 0.7
                and -0.25 <= n.scan_age <= 0.7
            ):
                print(
                    'LiDAR OK | idade do scan: '
                    f'{n.scan_age:.3f}s'
                )
                break
        else:
            raise RuntimeError(
                'SEM_SCAN_VALIDO_APOS_10_SEGUNDOS'
            )

        print('Aguardando TF map -> base_link...')

        prazo_tf = time.monotonic() + 12.0
        ultimo_erro = ''

        while time.monotonic() < prazo_tf:
            rclpy.spin_once(n, timeout_sec=0.2)

            try:
                n.tf('map')
                print('TF map -> base_link OK')
                break
            except Exception as exc:
                ultimo_erro = str(exc)
        else:
            raise RuntimeError(
                'TF_MAP_INDISPONIVEL_APOS_12S: ' +
                ultimo_erro
            )

        n.guard()

        t = n.tf()
        x = t.transform.translation.x
        y = t.transform.translation.y
        a = yaw(t.transform.rotation)
        target = wrap(rad(args.axis_deg))

        print(
            f'Atual ({x:.2f}, {y:.2f}), '
            f'yaw={deg(a):.1f} graus, '
            f'alvo={deg(target):.1f} graus'
        )

        if not args.only_align:
            p = pose(n, args.stage_x, args.stage_y, a)
            path = n.getPath(
                pose(n, x, y, a),
                p,
                use_start=True
            )

            if path is None or not path.poses:
                raise RuntimeError(
                    'Sem caminho planejado para ponto de espera'
                )

            print(
                f'ApproachAlign: {len(path.poses)} '
                'poses no caminho'
            )

        if not args.execute:
            if args.only_align:
                erro = wrap(target - a)

                if abs(erro) <= rad(3):
                    print('ALINHAMENTO JA SUFICIENTE')
                else:
                    passo = max(
                        -rad(5), min(rad(5), erro)
                    )
                    n.clearance_spin(passo)
            else:
                print(
                    'PREVIA APPROACH: caminho consultado; '
                    'movimento nao autorizado'
                )

            print(
                'MODO PREVIA: nenhum movimento enviado'
            )
            return

        if not args.only_align:
            n.moving = True
            n.goToPose(p)
            finish(n, 'ApproachAlign', 90)
            n.guard()

            t = n.tf()

            if math.hypot(
                t.transform.translation.x-args.stage_x,
                t.transform.translation.y-args.stage_y
            ) > .10:
                raise RuntimeError(
                    'Nao chegou ao ponto de espera '
                    'com tolerancia de 10cm'
                )

        for i in range(12):
            n.guard()
            error = wrap(
                target-yaw(n.tf().transform.rotation)
            )

            print(
                f'MicroAlign {i+1}: '
                f'erro {deg(error):+.1f} graus'
            )

            if abs(error) <= rad(3):
                print(
                    'ALINHADO. Nenhum comando de '
                    'atravessar corredor enviado.'
                )
                return

            n.clearance_spin(step)

            step = max(
                -rad(5), min(rad(5), error)
            )

            n.moving = True
            n.spin(
                spin_dist=step,
                time_allowance=12.
            )
            finish(n, 'MicroAlign', 15)

        raise RuntimeError(
            'Nao alinhou em 12 microgiros'
        )

    finally:
        if n.moving:
            try:
                n.cancelTask()
                n.isTaskComplete()
            except Exception:
                print(
                    'ATENCAO: nao foi possivel '
                    'confirmar cancelamento',
                    file=sys.stderr
                )
        n.destroyNode()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print('BLOQUEADO:', e, file=sys.stderr)
        sys.exit(2)
