"""Funcoes comuns da analise (11/11): pose map->base_link pelo /tf extraido, leitura de JSONL."""
import json, math, bisect, os, datetime

def ler(pasta, nome):
    p = os.path.join(pasta, nome)
    return [json.loads(l) for l in open(p)] if os.path.exists(p) else []

def hora(t):
    return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4]

def ang(a):
    return math.atan2(math.sin(a), math.cos(a))

class Poses:
    """Compoe map->odom (ultimo recebido) com odom->base_link."""
    def __init__(self, pasta):
        self.t, self.p = [], []
        mo = None
        for d in ler(pasta, 'tf.jsonl'):
            for tr in d['tf']:
                if tr['p'] == 'map':
                    mo = tr
                elif mo is not None:
                    c, s = math.cos(mo['yaw']), math.sin(mo['yaw'])
                    x = mo['x'] + c*tr['x'] - s*tr['y']
                    y = mo['y'] + s*tr['x'] + c*tr['y']
                    self.t.append(d['t']); self.p.append((x, y, ang(mo['yaw'] + tr['yaw']), tr['x'], tr['y'], tr['yaw']))
    def em(self, t):
        i = bisect.bisect_left(self.t, t)
        i = min(max(i, 0), len(self.t) - 1)
        return self.p[i]
