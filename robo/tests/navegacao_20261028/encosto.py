"""Encostar de frente numa mesa (Service Area) usando o LiDAR. Sem ROS (testavel fora do robo).

A partir da pose de aproximacao (~60 cm da face, olhando para a mesa):
medir -> girar (IMU) / deslizar (LiDAR) ate ficar de frente e centrado -> avancar devagar
ate o para-choque ficar a `distancia_alvo` da face -> medir de novo -> fim.
"""
import math


def _ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def _sat(v, vmin, vmax):
    if v == 0.0:
        return 0.0
    return math.copysign(min(max(abs(v), vmin), vmax), v)


def medir_face(pts, x_min=0.30, x_max=1.30, y_lim=0.70, tol=0.02):
    """Face da mesa a frente do robo (pontos em base_link).

    Devolve dict(dist, ang, centro, span) ou None:
      dist   = distancia (m) da origem do base_link ate a face, no eixo x do robo
      ang    = quanto girar (rad, + = anti-horario) para ficar de frente (perpendicular)
      centro = y do centro da face (m, + = a esquerda) ou None se uma ponta nao aparece
    """
    sel = [p for p in pts if x_min <= p[0] <= x_max and abs(p[1]) <= y_lim]
    if len(sel) < 6:
        return None
    # reta x = a + b*y (face quase perpendicular ao robo): varre angulos de -25 a +25 graus
    melhor = None
    for k in range(-50, 51):
        th = math.radians(k * 0.5)
        c, s = math.cos(th), math.sin(th)
        offs = sorted((p[0] * c - p[1] * s, p) for p in sel)
        j = 0
        for i in range(len(offs)):
            while offs[i][0] - offs[j][0] > 0.03:
                j += 1
            n = i - j + 1
            # empate: prefere a reta mais proxima (a face, nao a parede atras da mesa)
            if melhor is None or n > melhor[0] or (n == melhor[0] and offs[j][0] < melhor[2]):
                melhor = (n, th, offs[j][0], (offs[i][0] + offs[j][0]) / 2.0)
    n, th, _, centro_off = melhor
    c, s = math.cos(th), math.sin(th)
    inl = [p for p in sel if abs(p[0] * c - p[1] * s - centro_off) <= 0.02]
    for _ in range(2):
        m = len(inl)
        if m < 6:
            return None
        my = sum(p[1] for p in inl) / m
        mx = sum(p[0] for p in inl) / m
        syy = sum((p[1] - my) ** 2 for p in inl)
        if syy < 1e-4:
            return None
        b = sum((p[1] - my) * (p[0] - mx) for p in inl) / syy
        a = mx - b * my
        inl = [p for p in sel if abs(p[0] - (a + b * p[1])) < tol]
    if len(inl) < 6:
        return None
    ys = sorted(p[1] for p in inl)
    span = ys[-1] - ys[0]
    if span < 0.25:
        return None
    # centro so vale se as duas pontas da face aparecem (nao foram cortadas pela janela)
    centro = None
    if 0.60 <= span <= 0.98 and ys[0] > -y_lim + 0.05 and ys[-1] < y_lim - 0.05:
        yc = (ys[0] + ys[-1]) / 2.0
        xc = a + b * yc
        # deslocamento do robo ao longo da face (nao muda quando o robo gira no lugar;
        # o y puro do centro mudava ~1 cm por grau de giro: teste de 22/10 09:36)
        centro = (yc + xc * b) / math.sqrt(1.0 + b * b)
    # normal da face no robo: (1, -b) -> girar -atan(b) para ficar perpendicular
    return dict(dist=a, ang=-math.atan(b), centro=centro, span=span, n=len(inl))


class Encosto:
    def __init__(self, distancia_alvo=0.08, para_choque=0.365, v_aprox=0.02, v_final=0.016,
                 giro_min=0.02, giro_max=0.03, tol_giro=math.radians(2.0),
                 antecipacao=math.radians(2.0), tol_lat=0.015, lateral_cmd=0.02,
                 tempo_max=60.0, log=print):
        self.alvo = distancia_alvo
        self.pc = para_choque
        self.v_aprox, self.v_final = v_aprox, v_final
        self.giro_min, self.giro_max = giro_min, giro_max
        self.tol_giro, self.antecip = tol_giro, antecipacao
        self.tol_lat, self.lat_cmd = tol_lat, lateral_cmd
        self.tempo_max = tempo_max
        self.log = log
        self.estado = 'assentar'
        self.motivo = ''
        self.t0 = self.t_estado = None
        self.ini = None
        self.meds = []
        self.ciclos = 0
        self.n_lat = 0
        self.avancos = 0
        self.alvo_giro = self.alvo_lat = self.yaw_ref = None
        self.resultado = None
        self.p_dur = 0.15      # duracao do pulso de avanco final (s), ajustada sozinha
        self.p_ult = None
        self.n_pulsos = 0

    def nova_medicao(self, t, med, pose):
        if med is not None:
            self.meds.append((t, med, pose))
            self.meds = self.meds[-12:]

    def abortar(self, motivo):
        self.estado, self.motivo = 'abortar', motivo
        self.log('Encosto abortado: %s' % motivo)

    def bloqueio(self, t, pose):
        """Collision Monitor segurando: perto da mesa e esperado no avanco final."""
        if self.estado in ('avancar', 'pulso'):
            self.log('Collision Monitor parou o avanco: medindo onde ficou')
            self._mudar('assentar_final', t, pose)
        else:
            self.abortar('Collision Monitor bloqueando em "%s"' % self.estado)

    def _mudar(self, e, t, pose):
        self.estado, self.t_estado, self.ini = e, t, pose

    def _novas(self, desde):
        return [m for m in self.meds if m[0] >= desde]

    @staticmethod
    def _desloc(p0, p1):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(p0[2]), math.sin(p0[2])
        return c * dx + s * dy, -s * dx + c * dy

    def passo(self, t, pose):
        if self.t0 is None:
            self.t0 = t
            self._mudar('assentar', t, pose)
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if t - self.t0 > self.tempo_max:
            self.abortar('tempo maximo')
            return 0.0, 0.0, 0.0
        e = self.estado

        if e in ('assentar', 'assentar_final'):
            if t - self.t_estado >= 0.4:
                self._mudar('medir' if e == 'assentar' else 'medir_final', t, pose)
            return 0.0, 0.0, 0.0

        if e in ('medir', 'medir_final'):
            novas = self._novas(self.t_estado)
            if len(novas) < 4 and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if len(novas) < 2:
                self.abortar('nao vejo a face da mesa a frente')
                return 0.0, 0.0, 0.0
            med = lambda k: sorted(m[1][k] for m in novas)[len(novas) // 2]
            dist, ang = med('dist'), med('ang')
            centros = [m[1]['centro'] for m in novas if m[1]['centro'] is not None]
            centro = sorted(centros)[len(centros) // 2] if len(centros) >= 2 else None
            folga = dist - self.pc
            yaw_scan = novas[-1][2][2]
            if e == 'medir_final':
                self.resultado = dict(folga=folga, ang=ang, centro=centro)
                self.log('Encostado: frente a %.1f cm da mesa, %.1f graus, centro %s' % (
                    folga * 100, math.degrees(ang),
                    ('%+.1f cm' % (centro * 100)) if centro is not None else 'nao medido'))
                if folga > self.alvo + 0.025 and self.avancos < 3:
                    self.avancos += 1
                    self.yaw_ref = _ang(yaw_scan + ang)
                    self._mudar('avancar', t, pose)
                    return 0.0, 0.0, 0.0
                self.estado = 'fim'
                return 0.0, 0.0, 0.0
            self.ciclos += 1
            self.log('Medicao %d: face a %.0f cm (para-choque), girar %+.1f graus, centro %s' % (
                self.ciclos, folga * 100, math.degrees(ang),
                ('%+.1f cm' % (centro * 100)) if centro is not None else 'nao visto'))
            if folga < 0.03:
                self.abortar('ja esta perto demais da mesa (%.0f cm)' % (folga * 100))
                return 0.0, 0.0, 0.0
            pode = self.ciclos <= 10
            if pode and abs(ang) > self.tol_giro:
                self.alvo_giro = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif pode and centro is not None and abs(centro) > self.tol_lat and self.n_lat < 5:
                self.n_lat += 1
                self.alvo_lat = centro
                self._mudar('deslocar', t, pose)
            elif abs(ang) > math.radians(3.0) or (centro is not None and abs(centro) > 0.04):
                self.abortar('nao consegui alinhar com a mesa (%.1f graus, centro %+.1f cm)' % (
                    math.degrees(ang), (centro or 0.0) * 100))
            else:
                self.yaw_ref = _ang(yaw_scan + ang)
                self.log('Alinhado com a mesa: avancando ate %.0f cm' % (self.alvo * 100))
                self._mudar('avancar', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'girar':
            err = _ang(self.alvo_giro - pose[2])
            if abs(err) < max(math.radians(0.6), self.antecip) or t - self.t_estado > 8.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, _sat(0.5 * err, self.giro_min, self.giro_max)

        if e == 'deslocar':
            # Lateral em pulsos medidos pelo LiDAR: com comando pequeno o robo desliza ~15 cm/s
            # e passava do centro (teste de 22/10 09:36: -5 cm virou +5 cm num deslize so).
            self.l_ult = None
            self.l_pulsos = 0
            self.l_dur = 0.15
            self.l_falta = self.alvo_lat
            self.l_yaw = pose[2]
            self._mudar('lat_pulso', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'lat_pulso':
            if t - self.t_estado >= self.l_dur:
                self._mudar('lat_esperar', t, pose)
                return 0.0, 0.0, 0.0
            err = _ang(self.l_yaw - pose[2])
            wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(0.8) else 0.0
            return 0.0, math.copysign(self.lat_cmd, self.l_falta), wz

        if e == 'lat_esperar':
            novas = [m for m in self._novas(self.t_estado + 0.2) if m[1]['centro'] is not None]
            if (len(novas) < 2 or t - self.t_estado < 0.5) and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            self.l_pulsos += 1
            if not novas or self.l_pulsos >= 8:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            c = sorted(m[1]['centro'] for m in novas[-3:])[len(novas[-3:]) // 2]
            if abs(c) < self.tol_lat:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            if c * self.l_falta < 0:
                self.l_dur = max(0.10, self.l_dur * 0.5)  # passou do centro: volta com pulso menor
            else:
                andou = abs(self.l_falta) - abs(c)
                if andou > 0.003:
                    novo = 0.5 * abs(c) / (andou / self.l_dur)
                else:
                    novo = self.l_dur * 1.4
                self.l_dur = min(max(novo, 0.10), self.l_dur * 1.4, 0.30)
            self.l_falta = c
            self._mudar('lat_pulso', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'avancar':
            # Avanco continuo ate ~6 cm antes do alvo, freando pelo LiDAR (o odom nao serve:
            # com comando pequeno o robo anda ate 2x o que o odom diz - teste de 22/10 08:45).
            novas = self._novas(self.t_estado)
            if novas:
                tl, med, _ = novas[-1]
                folga = med['dist'] - self.pc
                v = 0.0
                if len(novas) >= 2:
                    t0_, m0, _ = novas[-2]
                    if tl - t0_ > 0.05:
                        v = max(0.0, ((m0['dist'] - self.pc) - folga) / (tl - t0_))
                previsto = folga - v * (t - tl + 0.35)  # 0.35 s = atraso + parada
            else:
                folga = previsto = 9.0
            if previsto <= self.alvo + 0.08 or t - self.t_estado > 15.0:
                self.p_ult = None
                self.n_pulsos = 0
                self._mudar('esperar', t, pose)
                return 0.0, 0.0, 0.0
            err = _ang(self.yaw_ref - pose[2])
            wz = _sat(0.4 * err, 0.016, 0.03) if abs(err) > math.radians(1.0) else 0.0
            return self.v_aprox, 0.0, wz

        if e == 'esperar':
            # parado: mede a folga com 2 leituras novas e decide o proximo pulso
            novas = self._novas(self.t_estado + 0.2)
            if (len(novas) < 2 or t - self.t_estado < 0.5) and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if not novas:
                self.abortar('perdi a face da mesa')
                return 0.0, 0.0, 0.0
            folga = sorted(m[1]['dist'] for m in novas[-3:])[len(novas[-3:]) // 2] - self.pc
            falta = folga - self.alvo
            if falta <= 0.012 or self.n_pulsos >= 12:
                self._mudar('assentar_final', t, pose)
                return 0.0, 0.0, 0.0
            if self.p_ult is not None:
                andou = self.p_ult - folga
                # o robo quase nao anda abaixo de ~0.15 s e depois dispara (pulso de 0.40 s
                # andou 6.6 cm no teste de 09:36): cresce no maximo 40% por vez, ate 0.30 s
                if andou > 0.003:
                    taxa = andou / self.p_dur
                    novo = 0.5 * falta / taxa
                else:
                    novo = self.p_dur * 1.4
                self.p_dur = min(max(novo, 0.10), self.p_dur * 1.4, 0.30)
            self.p_ult = folga
            self.n_pulsos += 1
            self.log('Folga %.1f cm: pulso %d de %.2f s' % (folga * 100, self.n_pulsos, self.p_dur))
            self._mudar('pulso', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'pulso':
            if t - self.t_estado >= self.p_dur:
                self._mudar('esperar', t, pose)
                return 0.0, 0.0, 0.0
            return self.v_final, 0.0, 0.0
        return 0.0, 0.0, 0.0
