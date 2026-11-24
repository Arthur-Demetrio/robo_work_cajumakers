"""Utilidades das analises de 28/10: horario local (Recife, UTC-3), pose no mapa e o alinhador."""
import bisect, importlib.util, math, os, pickle
from datetime import datetime, timezone, timedelta

TZ = timezone(timedelta(hours=-3))
AQUI = os.path.dirname(os.path.abspath(__file__))


def alinhador():
    caminho = os.environ.get('AL', os.path.join(AQUI, '..', 'robo_navigation', 'scripts', 'alinhador_passagem.py'))
    sp = importlib.util.spec_from_file_location('al', caminho)
    al = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(al)
    return al


def carregar(pkl):
    D = pickle.load(open(pkl, 'rb'))
    D['_ts'] = {k: [r[0] for r in v] for k, v in D.items() if isinstance(v, list)}
    return D


def hms(t):
    return datetime.fromtimestamp(t, TZ).strftime('%H:%M:%S.%f')[:-4]


def ep(D, h):
    """'HH:MM:SS.s' no dia do bag -> epoch."""
    d = datetime.fromtimestamp(D['scan'][0][0], TZ)
    a, b, c = h.split(':')
    return datetime(d.year, d.month, d.day, int(a), int(b), 0, tzinfo=TZ).timestamp() + float(c)


def em(D, chave, t):
    i = bisect.bisect_right(D['_ts'][chave], t) - 1
    return D[chave][max(i, 0)]


def pose_mapa(D, t):
    a, b = em(D, 'mo', t), em(D, 'ob', t)
    c, s = math.cos(a[3]), math.sin(a[3])
    return a[1] + c * b[1] - s * b[2], a[2] + s * b[1] + c * b[2], a[3] + b[3]


def pontos(scan, laser=(0.33, 0.005)):
    _, a0, da, R = scan
    return [(laser[0] + r * math.cos(a0 + i * da), laser[1] + r * math.sin(a0 + i * da))
            for i, r in enumerate(R) if 0.05 < r < 8]
