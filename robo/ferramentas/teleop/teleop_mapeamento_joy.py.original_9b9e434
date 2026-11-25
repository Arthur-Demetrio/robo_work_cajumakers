#!/usr/bin/env python3
"""Teleop manual por controle (Xbox Series X ou PS5) para mapear a arena e marcar pontos.

Le o controle direto em /dev/input/jsN (API joystick do Linux, sem joy_node), detecta o modelo
pelo nome do dispositivo, publica TwistStamped em /cmd_vel_nav (o Collision Monitor continua no
pipeline) e grava poses map->base_link nos arquivos que o executor_prova.py ja le:

  START / FINISH         -> --pontos         (padrao ~/mapas/pontos.yaml, chave 'pontos')
  WS_n / SH_n / PP_n     -> --service-areas  (padrao ~/mapas/service_areas.yaml)
     type: workstation | shelf | precision
     aproximacao = pose do robo no momento da marcacao (robo DE FRENTE para a mesa)
     face        = estimada a --dist-face m a frente do robo (origem: teleop)

Comandos (Xbox / PS5):
  LB / L1 (segurar)     homem-morto: so anda segurando
  analogico esquerdo    frente/re e lateral (mecanum)
  analogico direito X   girar
  RB / R1 (segurar)     velocidade rapida
  B / Circulo           PARADA (zera e trava ate soltar tudo)
  D-pad cima/baixo      troca o tipo: START, FINISH, WS (workstation), SH (shelf), PP (precision)
  D-pad esq/dir         troca o numero (WS_1, WS_2, ...)
  A / Cruz              MARCA a pose atual no item selecionado
  Y / Triangulo         apaga o item selecionado (pressionar 2x em 2 s)
  Menu / Options        GRAVA os arquivos (com backup .bak_teleop_<hora>)
  View / Create         lista o que ja foi marcado
Teclado (terminal): Ctrl+C sai (pergunta se grava o que nao foi gravado).

Nao usar junto de goal Nav2 ativo.
"""
import argparse
import array
import fcntl
import glob
import math
import os
import select
import shutil
import struct
import sys
import time

import yaml

# ---------------------------------------------------------------------------
# Mapeamentos de botoes/eixos (API js do Linux)
# ---------------------------------------------------------------------------
# Xbox Series X/S pelo driver xpad (USB) ou xpadneo (Bluetooth): mesma numeracao.
# PS5 DualSense pelo hid-playstation (USB e Bluetooth).
# Se o seu controle numerar diferente, rode com --mostrar-eventos e ajuste/use --mapa.
PERFIS = {
    'xbox': {
        'botoes': {'marcar': 0, 'parar': 1, 'apagar': 3, 'deadman': 4, 'rapido': 5,
                   'listar': 6, 'gravar': 7},
        'eixos': {'lx': 0, 'ly': 1, 'rx': 3, 'hat_x': 6, 'hat_y': 7},
        'nomes': {'marcar': 'A', 'parar': 'B', 'apagar': 'Y', 'deadman': 'LB', 'rapido': 'RB',
                  'listar': 'View', 'gravar': 'Menu'},
    },
    # Xbox por Bluetooth sem xpadneo (hid-generic): X/Y deslocados e LB/RB em 6/7.
    'xbox_bt': {
        'botoes': {'marcar': 0, 'parar': 1, 'apagar': 4, 'deadman': 6, 'rapido': 7,
                   'listar': 10, 'gravar': 11},
        'eixos': {'lx': 0, 'ly': 1, 'rx': 2, 'hat_x': 6, 'hat_y': 7},
        'nomes': {'marcar': 'A', 'parar': 'B', 'apagar': 'Y', 'deadman': 'LB', 'rapido': 'RB',
                  'listar': 'View', 'gravar': 'Menu'},
    },
    'ps5': {
        'botoes': {'marcar': 0, 'parar': 1, 'apagar': 2, 'deadman': 4, 'rapido': 5,
                   'listar': 8, 'gravar': 9},
        'eixos': {'lx': 0, 'ly': 1, 'rx': 3, 'hat_x': 6, 'hat_y': 7},
        'nomes': {'marcar': 'Cruz', 'parar': 'Circulo', 'apagar': 'Triangulo', 'deadman': 'L1',
                  'rapido': 'R1', 'listar': 'Create', 'gravar': 'Options'},
    },
}

TIPOS = ['START', 'FINISH', 'WS', 'SH', 'PP']
TIPO_AREA = {'WS': 'workstation', 'SH': 'shelf', 'PP': 'precision'}

JS_EVENT_FMT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80
JSIOCGNAME = lambda n: 0x80006A13 + (0x10000 * n)  # noqa: E731
JSIOCGAXES, JSIOCGBUTTONS = 0x80016A11, 0x80016A12


# ---------------------------------------------------------------------------
# Controle
# ---------------------------------------------------------------------------

class Joystick:
    def __init__(self, dev):
        self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        buf = array.array('B', [0] * 128)
        fcntl.ioctl(self.fd, JSIOCGNAME(len(buf)), buf)
        self.nome = buf.tobytes().split(b'\0')[0].decode(errors='replace')
        n = array.array('B', [0])
        fcntl.ioctl(self.fd, JSIOCGAXES, n)
        self.eixos = [0.0] * n[0]
        fcntl.ioctl(self.fd, JSIOCGBUTTONS, n)
        self.botoes = [0] * n[0]
        self.ultimo_evento = time.monotonic()

    def ler(self):
        """Le todos os eventos pendentes; devolve lista de (tipo, numero, valor) de bordas."""
        bordas = []
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            if len(data) < JS_EVENT_SIZE:
                break
            _, valor, tipo, num = struct.unpack(JS_EVENT_FMT, data)
            init = bool(tipo & JS_EVENT_INIT)
            tipo &= ~JS_EVENT_INIT
            self.ultimo_evento = time.monotonic()
            if tipo == JS_EVENT_BUTTON and num < len(self.botoes):
                self.botoes[num] = valor
                if not init:
                    bordas.append(('b', num, valor))
            elif tipo == JS_EVENT_AXIS and num < len(self.eixos):
                antigo = self.eixos[num]
                self.eixos[num] = valor / 32767.0
                if not init:
                    bordas.append(('a', num, self.eixos[num], antigo))
        return bordas

    def fechar(self):
        os.close(self.fd)


def achar_controle(dev_arg):
    devs = [dev_arg] if dev_arg else sorted(glob.glob('/dev/input/js*'))
    for d in devs:
        try:
            return Joystick(d), d
        except OSError as e:
            print('Nao abriu %s: %s' % (d, e))
    raise SystemExit('Nenhum controle em /dev/input/js*. Conecte o controle (USB ou Bluetooth) '
                     'e confira permissao (grupo input).')


def detectar_perfil(nome):
    n = nome.lower()
    if any(k in n for k in ('dualsense', 'playstation', 'sony', 'wireless controller')):
        return 'ps5'
    if 'xbox' in n or 'x-box' in n or 'microsoft' in n:
        return 'xbox'
    return None


# ---------------------------------------------------------------------------
# Arquivos
# ---------------------------------------------------------------------------

def ler_yaml(caminho):
    if os.path.exists(caminho):
        with open(caminho) as f:
            return yaml.safe_load(f) or {}
    return {}


def backup(caminho):
    if os.path.exists(caminho):
        bak = caminho + '.bak_teleop_' + time.strftime('%Y%m%d_%H%M%S')
        shutil.copy2(caminho, bak)
        return bak
    return None


def gravar_yaml(caminho, doc, cabecalho):
    os.makedirs(os.path.dirname(os.path.abspath(caminho)), exist_ok=True)
    bak = backup(caminho)
    tmp = caminho + '.tmp'
    with open(tmp, 'w') as f:
        f.write(cabecalho)
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
    os.replace(tmp, caminho)
    return bak


def pose_dict(x, y, yaw):
    return {'x': round(float(x), 3), 'y': round(float(y), 3), 'yaw': round(float(yaw), 4)}


# ---------------------------------------------------------------------------
# Programa
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description='Teleop por controle Xbox/PS5 para mapeamento e marcacao '
                                            'de START/FINISH/WS/SH/PP')
    p.add_argument('--dev', help='dispositivo do controle (padrao: primeiro /dev/input/js*)')
    p.add_argument('--controle', choices=sorted(PERFIS), help='forca o perfil (padrao: autodetecta)')
    p.add_argument('--linear', type=float, default=0.10, help='m/s maximo frente/re (normal)')
    p.add_argument('--lateral', type=float, default=0.08, help='m/s maximo lateral (normal)')
    p.add_argument('--angular', type=float, default=0.30, help='rad/s maximo giro (normal)')
    p.add_argument('--fator-rapido', type=float, default=2.0, help='multiplicador com RB/R1')
    p.add_argument('--zona-morta', type=float, default=0.12)
    p.add_argument('--rampa', type=float, default=0.30, help='aceleracao max em m/s^2 (giro: x3)')
    p.add_argument('--topic', default='/cmd_vel_nav')
    p.add_argument('--taxa', type=float, default=20.0, help='Hz de publicacao')
    p.add_argument('--pontos', default=os.path.expanduser('~/mapas/pontos.yaml'))
    p.add_argument('--service-areas', default=os.path.expanduser('~/mapas/service_areas.yaml'))
    p.add_argument('--dist-face', type=float, default=0.60,
                   help='distancia (m) da pose do robo ate a face estimada da mesa')
    p.add_argument('--max-indice', type=int, default=9)
    p.add_argument('--mostrar-eventos', action='store_true',
                   help='so imprime botoes/eixos do controle (para conferir o mapeamento)')
    a = p.parse_args()
    if not (0 < a.linear <= 0.25 and 0 < a.lateral <= 0.25 and 0 < a.angular <= 0.8):
        p.error('limites: 0 < linear,lateral <= 0.25 m/s; 0 < angular <= 0.8 rad/s')
    if not 1.0 <= a.fator_rapido <= 3.0:
        p.error('fator-rapido entre 1 e 3')
    return a


def mostrar_eventos(js):
    print('Controle: "%s" (%d eixos, %d botoes). Ctrl+C sai.' % (js.nome, len(js.eixos), len(js.botoes)))
    try:
        while True:
            select.select([js.fd], [], [], 0.2)
            for ev in js.ler():
                if ev[0] == 'b':
                    print('botao %2d -> %d' % (ev[1], ev[2]))
                elif abs(ev[2] - ev[3]) > 0.25 or ev[2] in (-1.0, 1.0):
                    print('eixo  %2d -> %+.2f' % (ev[1], ev[2]))
    except KeyboardInterrupt:
        pass


def main():
    a = parse_args()
    js, dev = achar_controle(a.dev)
    if a.mostrar_eventos:
        mostrar_eventos(js)
        return

    perfil_nome = a.controle or detectar_perfil(js.nome)
    if perfil_nome is None:
        raise SystemExit('Controle "%s" nao reconhecido; use --controle xbox|xbox_bt|ps5.' % js.nome)
    perfil = PERFIS[perfil_nome]
    B, E, N = perfil['botoes'], perfil['eixos'], perfil['nomes']
    precisa = max(B.values()) + 1
    if len(js.botoes) < precisa or len(js.eixos) <= max(E.values()):
        print('AVISO: controle tem %d botoes/%d eixos; o perfil %s espera mais. '
              'Confira com --mostrar-eventos.' % (len(js.botoes), len(js.eixos), perfil_nome))

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init(args=None)
    node = Node('teleop_mapeamento_joy')
    pub = node.create_publisher(TwistStamped, a.topic, 10)
    tfb = Buffer()
    TransformListener(tfb, node)

    # Estado das marcacoes (comeca com o que ja existe nos arquivos)
    doc_pts = ler_yaml(a.pontos)
    pontos = dict(doc_pts.get('pontos') or {})
    doc_sa = ler_yaml(a.service_areas)
    areas = dict(doc_sa.get('service_areas') or {})
    alterado = False

    sel_tipo, sel_idx = 0, 1
    v = [0.0, 0.0, 0.0]
    travado = False
    apagar_pedido = 0.0
    dt = 1.0 / a.taxa

    def chave():
        t = TIPOS[sel_tipo]
        return t if t in ('START', 'FINISH') else '%s_%d' % (t, sel_idx)

    def status(msg=''):
        k = chave()
        tem = k in pontos or k in areas
        sys.stdout.write('\r\033[K[%s] selecionado: %-7s %s%s %s' % (
            'TRAVADO' if travado else ('ANDANDO' if any(v) else 'parado '),
            k, '(marcado)' if tem else '(vazio)  ', ' *nao gravado*' if alterado else '', msg))
        sys.stdout.flush()

    def log(msg):
        sys.stdout.write('\r\033[K' + msg + '\n')
        status()

    def pose_atual():
        try:
            tr = tfb.lookup_transform('map', 'base_link', Time())
        except Exception as e:  # noqa: BLE001
            log('ERRO: sem TF map->base_link (%s). SLAM/localizacao no ar?' % type(e).__name__)
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def marcar():
        nonlocal alterado
        p = pose_atual()
        if p is None:
            return
        x, y, yaw = p
        k = chave()
        if k in ('START', 'FINISH'):
            pontos[k] = pose_dict(x, y, yaw)
        else:
            antigo = areas.get(k) or {}
            fx, fy = x + a.dist_face * math.cos(yaw), y + a.dist_face * math.sin(yaw)
            areas[k] = {
                'type': TIPO_AREA[k.split('_')[0]],
                'table': antigo.get('table') or {'width': 0.8, 'depth': 0.5, 'height': 0.1},
                'face': dict(pose_dict(fx, fy, yaw), origem='teleop'),
                'aproximacao': pose_dict(x, y, yaw),
                'marcado_em': time.strftime('%Y-%m-%d %H:%M:%S'),
            }
        alterado = True
        log('MARCADO %-7s x=%.3f y=%.3f yaw=%.1f graus' % (k, x, y, math.degrees(yaw)))

    def apagar():
        nonlocal alterado, apagar_pedido
        k = chave()
        alvo = pontos if k in pontos else areas if k in areas else None
        if alvo is None:
            log('%s ja esta vazio.' % k)
            return
        agora = time.monotonic()
        if agora - apagar_pedido > 2.0:
            apagar_pedido = agora
            log('Apertar %s de novo em 2 s para apagar %s.' % (N['apagar'], k))
            return
        apagar_pedido = 0.0
        del alvo[k]
        alterado = True
        log('APAGADO %s' % k)

    def gravar():
        nonlocal alterado
        hora = time.strftime('%Y-%m-%d %H:%M:%S')
        doc_pts['pontos'] = pontos
        b1 = gravar_yaml(a.pontos, doc_pts,
                         '# START e FINISH (frame map). Gravado por teleop_mapeamento_joy.py em %s\n' % hora)
        doc_sa['frame'] = doc_sa.get('frame', 'map')
        doc_sa['service_areas'] = areas
        doc_sa['gerado_em'] = hora
        b2 = gravar_yaml(a.service_areas, doc_sa,
                         '# Areas marcadas (WS/SH/PP). Frame map. aproximacao = pose do robo; face estimada '
                         'a %.2f m quando origem: teleop\n' % a.dist_face)
        alterado = False
        log('GRAVADO %s e %s%s' % (a.pontos, a.service_areas,
                                    ' (backups .bak_teleop_*)' if (b1 or b2) else ''))

    def listar():
        linhas = ['--- marcados ---']
        for k in ('START', 'FINISH'):
            if k in pontos:
                p = pontos[k]
                linhas.append('  %-7s x=%7.3f y=%7.3f yaw=%6.1f°' % (k, p['x'], p['y'], math.degrees(p['yaw'])))
        for k in sorted(areas):
            p = areas[k].get('aproximacao') or areas[k].get('face') or {}
            linhas.append('  %-7s %-11s x=%7.3f y=%7.3f yaw=%6.1f°' % (
                k, areas[k].get('type', '?'), p.get('x', 0), p.get('y', 0), math.degrees(p.get('yaw', 0))))
        log('\n'.join(linhas))

    def publicar(vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = float(vx), float(vy), float(wz)
        pub.publish(msg)

    def eixo(i):
        val = js.eixos[i] if i < len(js.eixos) else 0.0
        if abs(val) < a.zona_morta:
            return 0.0
        return math.copysign((abs(val) - a.zona_morta) / (1 - a.zona_morta), val)

    def rampa(atual, alvo, passo):
        return atual + max(-passo, min(passo, alvo - atual))

    print('TELEOP MAPEAMENTO — controle "%s" em %s, perfil %s' % (js.nome, dev, perfil_nome))
    print('  segure %s e use os analogicos | %s rapido | %s PARADA' % (N['deadman'], N['rapido'], N['parar']))
    print('  D-pad cima/baixo: tipo (START/FINISH/WS=workstation/SH=shelf/PP=precision) | esq/dir: numero')
    print('  %s marca | %s apaga (2x) | %s grava | %s lista' % (N['marcar'], N['apagar'], N['gravar'], N['listar']))
    print('  Publicando %s (TwistStamped). Collision Monitor continua ativo. Sem goal Nav2 ativo!' % a.topic)
    print('  Arquivos: %s | %s' % (a.pontos, a.service_areas))
    status()

    try:
        proximo = time.monotonic()
        while rclpy.ok():
            select.select([js.fd], [], [], max(0.0, proximo - time.monotonic()))
            for ev in js.ler():
                if ev[0] == 'b' and ev[2] == 1:
                    n = ev[1]
                    if n == B['parar']:
                        travado = True
                        v = [0.0, 0.0, 0.0]
                        log('PARADA acionada: solte tudo para destravar.')
                    elif n == B['marcar']:
                        marcar()
                    elif n == B['apagar']:
                        apagar()
                    elif n == B['gravar']:
                        gravar()
                    elif n == B['listar']:
                        listar()
                elif ev[0] == 'a' and ev[1] in (E['hat_x'], E['hat_y']):
                    novo, antigo = round(ev[2]), round(ev[3])
                    if novo == 0 or novo == antigo:
                        continue
                    if ev[1] == E['hat_y']:   # cima = -1
                        sel_tipo = (sel_tipo + (1 if novo > 0 else -1)) % len(TIPOS)
                    else:
                        sel_idx = max(1, min(a.max_indice, sel_idx + (1 if novo > 0 else -1)))
                    apagar_pedido = 0.0
                    status()
            agora = time.monotonic()
            if agora < proximo:
                continue
            proximo = agora + dt

            deadman = bool(js.botoes[B['deadman']]) if B['deadman'] < len(js.botoes) else False
            if travado and not any(js.botoes) and abs(eixo(E['lx'])) + abs(eixo(E['ly'])) + abs(eixo(E['rx'])) == 0:
                travado = False
                log('Destravado.')
            if deadman and not travado and os.path.exists(dev):
                f = a.fator_rapido if js.botoes[B['rapido']] else 1.0
                alvo = [-eixo(E['ly']) * a.linear * f, -eixo(E['lx']) * a.lateral * f,
                        -eixo(E['rx']) * a.angular * f]
            else:
                alvo = [0.0, 0.0, 0.0]
            if not os.path.exists(dev):
                if any(v):
                    log('CONTROLE DESCONECTADO: parando.')
                v = [0.0, 0.0, 0.0]
            elif not deadman or travado:
                v = [0.0, 0.0, 0.0]  # soltar o deadman para na hora, sem rampa
            else:
                passo = a.rampa * dt
                v = [rampa(v[0], alvo[0], passo), rampa(v[1], alvo[1], passo), rampa(v[2], alvo[2], 3 * passo)]
            publicar(*v)
            status()
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print('\nErro lendo o controle: %s' % e)
    finally:
        for _ in range(5):
            publicar(0.0, 0.0, 0.0)
            time.sleep(0.04)
        print('\nComando zero publicado.')
        if alterado:
            try:
                r = input('Ha marcacoes nao gravadas. Gravar agora? [s/N] ').strip().lower()
            except EOFError:
                r = ''
            if r == 's':
                gravar()
                print()
        js.fechar()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
