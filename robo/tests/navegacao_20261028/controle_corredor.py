"""Travessia de corredor guiada pelo LiDAR (sem ROS, testavel fora do robo)."""
import math


def _ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def _sat(v, vmin, vmax):
    """Satura |v| entre vmin e vmax mantendo o sinal (vmin vence a zona morta do smoother)."""
    if v == 0.0:
        return 0.0
    return math.copysign(min(max(abs(v), vmin), vmax), v)


class Travessia:
    """Maquina de estados: aproximar -> medir -> girar/deslocar -> medir ... -> atravessar -> fim.

    Coordenadas: pose do odom (x, y, yaw); a medicao do LiDAR vem em base_link.
    """

    def __init__(self, v_aprox=0.03, v_travessia=0.035, giro_min=0.02, giro_max=0.05,
                 lateral_cmd=0.02, tol_giro=math.radians(1.5), tol_lateral=0.015,
                 largura_min=0.66, largura_max=1.00, aprox_max=0.40, ciclos_max=6,
                 tempo_max=60.0, log=print):
        self.v_aprox = v_aprox
        self.v_trav = v_travessia
        self.giro_min, self.giro_max = giro_min, giro_max
        self.lat_cmd = lateral_cmd
        self.tol_giro, self.tol_lat = tol_giro, tol_lateral
        self.larg_min, self.larg_max = largura_min, largura_max
        self.aprox_max = aprox_max
        self.ciclos_max = ciclos_max
        self.tempo_max = tempo_max
        self.log = log
        self.estado = 'aproximar'
        self.motivo = ''
        self.t0 = None
        self.t_estado = None
        self.inicio_estado = None  # pose do odom quando o estado comecou
        self.meds = []             # (t, med, pose_no_scan)
        self.ult = None            # ultima medicao boa de 2 paredes
        self.largura = None
        self.ciclos = 0
        self.n_lat = 0
        self.yaw_ref = None
        self.alvo = None

    # ------------------------------------------------------------------
    def _boa(self, med):
        return (med is not None and med['lados'] == 2 and med['n'] >= 20 and
                self.larg_min < med['largura'] < self.larg_max)

    def nova_medicao(self, t, med, pose):
        if med is None:
            return
        if self._boa(med):
            self.meds.append((t, med, pose))
            self.meds = self.meds[-10:]
            self.ult = (t, med, pose)
            self.largura = med['largura'] if self.largura is None else (
                0.8 * self.largura + 0.2 * med['largura'])
        elif (self.estado == 'atravessar' and med['n'] >= 10 and med.get('desvio') is not None
              and self.ult is not None):
            # uma parede so (dentro do corredor): aceita se concordar com o angulo atual
            yaw_eixo = pose[2] + med['angulo']
            if abs(_ang(yaw_eixo - self.yaw_ref)) < math.radians(4):
                self.ult = (t, med, pose)

    def _mudar(self, estado, t, pose):
        self.estado, self.t_estado, self.inicio_estado = estado, t, pose

    def _desloc_robo(self, p0, p1):
        """Deslocamento de p0 ate p1 no referencial do robo em p0."""
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(p0[2]), math.sin(p0[2])
        return c * dx + s * dy, -s * dx + c * dy

    def _erro_lateral(self, pose):
        """Centro do corredor (m, + = a esquerda), corrigido pelo movimento desde o scan."""
        t, med, p_scan = self.ult
        dx, dy = self._desloc_robo(p_scan, pose)
        return med['desvio'] + math.tan(med['angulo']) * dx - dy

    def abortar(self, motivo):
        self.estado, self.motivo = 'abortar', motivo
        self.log('Travessia LiDAR abortada: %s' % motivo)

    # ------------------------------------------------------------------
    def passo(self, t, pose, restante):
        """Devolve (vx, vy, wz). restante: metros ate o centro do robo passar da saida."""
        if self.t0 is None:
            self.t0 = t
            self.p_aprox = pose
            self._mudar('aproximar', t, pose)
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if t - self.t0 > self.tempo_max:
            self.abortar('tempo maximo')
            return 0.0, 0.0, 0.0
        e = self.estado

        if e == 'aproximar':
            vistas = [m for m in self.meds if m[0] >= self.t_estado]
            if len(vistas) >= 2:  # duas leituras boas seguidas: nao e ruido
                self.log('Paredes vistas (largura %.2f m): parando na boca para alinhar' %
                         self.ult[1]['largura'])
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            andou = math.hypot(pose[0] - self.p_aprox[0], pose[1] - self.p_aprox[1])
            if andou > self.aprox_max:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return self.v_aprox, 0.0, 0.0

        if e == 'assentar':  # espera o robo parar e o LiDAR atualizar
            if t - self.t_estado >= 0.5:
                self._mudar('medir', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'medir':
            novas = [m for m in self.meds if m[0] >= self.t_estado]
            if len(novas) < 5 and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if len(novas) < 2:
                # ainda longe: avanca mais um pouco (dentro do limite) antes de desistir
                andou = math.hypot(pose[0] - self.p_aprox[0], pose[1] - self.p_aprox[1])
                if self.ciclos == 0 and andou < self.aprox_max:
                    self.estado, self.t_estado, self.inicio_estado = 'aproximar', t, pose
                    return 0.0, 0.0, 0.0
                self.abortar('nao vejo as duas paredes da passagem')
                return 0.0, 0.0, 0.0
            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            self.ciclos += 1
            self.log('Medicao %d: girar %+.1f graus, centro a %+.1f cm, largura %.2f m' % (
                self.ciclos, math.degrees(ang), des * 100, self.largura))
            if self.largura < self.larg_min + 0.02:
                self.abortar('passagem estreita demais (%.2f m)' % self.largura)
                return 0.0, 0.0, 0.0
            # giro grande primeiro (atrapalha a medida lateral); depois no maximo 2 deslizes
            # (cada deslize da um tranco de yaw); o giro fino fica por ultimo
            pode = self.ciclos <= self.ciclos_max
            if pode and abs(ang) > 2 * self.tol_giro:
                self.alvo = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif pode and abs(des) > self.tol_lat and self.n_lat < 2:
                self.n_lat += 1
                self.alvo = des
                self._mudar('deslocar', t, pose)
            elif pode and abs(ang) > self.tol_giro:
                self.alvo = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif abs(ang) < 2.5 * self.tol_giro and (
                    abs(des) < 2 * self.tol_lat or (self.n_lat >= 2 and abs(des) < 0.045)):
                # o resto do erro lateral e corrigido andando
                self.yaw_ref = _ang(yaw_scan + ang)
                self.log('Alinhado: atravessando')
                self._mudar('atravessar', t, pose)
            else:
                self.abortar('nao consegui alinhar em %d tentativas' % self.ciclos_max)
            return 0.0, 0.0, 0.0

        if e == 'girar':
            err = _ang(self.alvo - pose[2])
            if abs(err) < math.radians(0.6) or t - self.t_estado > 8.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, _sat(0.5 * err, self.giro_min, self.giro_max)

        if e == 'deslocar':
            _, dy = self._desloc_robo(self.inicio_estado, pose)
            falta = self.alvo - dy
            if abs(falta) < 0.004 or falta * self.alvo < 0 or t - self.t_estado > 6.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, math.copysign(self.lat_cmd, falta), 0.0

        if e == 'atravessar':
            if restante <= 0.0:
                self.estado = 'fim'
                self.log('Travessia LiDAR concluida')
                return 0.0, 0.0, 0.0
            # eixo do corredor: corrige devagar com as medicoes de 2 paredes
            tm, med, p_scan = self.ult
            if med['lados'] == 2 and t - tm < 0.5:
                eixo = _ang(p_scan[2] + med['angulo'])
                self.yaw_ref = _ang(self.yaw_ref + max(-0.01, min(0.01, 0.3 * _ang(eixo - self.yaw_ref))))
            err_yaw = _ang(self.yaw_ref - pose[2])
            err_lat = self._erro_lateral(pose) if t - tm < 1.5 else 0.0
            wz = _sat(0.4 * err_yaw, 0.016, 0.03) if abs(err_yaw) > math.radians(1.2) else 0.0
            vy = _sat(0.5 * err_lat, 0.016, 0.025) if abs(err_lat) > self.tol_lat else 0.0
            vx = self.v_trav
            if abs(err_yaw) > math.radians(4) or abs(err_lat) > 0.035:
                vx = 0.0  # corrige parado antes de seguir
            return vx, vy, wz
        return 0.0, 0.0, 0.0
