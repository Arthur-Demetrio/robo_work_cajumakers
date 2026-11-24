#!/usr/bin/env python3
"""Calibra as poses de config/slots.yaml movendo o braco junta a junta pelo MoveIt.

Precisa do MoveIt real rodando (garra_real.launch.py). O robo nao anda.

    source ~/robo_work_teste/garra/install/setup.bash
    python3 ~/robo_work_teste/garra/scripts/calibrar_slots.py

Comandos:
  1+ / 1- / 2+ / 2- / 3+ / 3-   move a junta 1..3 um passo (confere colisao antes)
  passo <graus>                 tamanho do passo (padrao 3 graus, maximo 15)
  abrir / fechar                dedos
  ir <nome>                     vai a uma pose ja gravada (pronta, mostrar_apriltag, acima_slots, s0..s3)
  gravar <nome>                 grava as juntas atuais em <nome>
  yplat <slot> <m|null>         y_plataforma do slot
  estado / sair
"""
import math
import os
import re
import shutil
import sys
import time

import rclpy
import yaml
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

from garra_coleta.cliente_garra import LIMITES, SUCESSO, ClienteGarra

ARQ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'garra_coleta', 'config', 'slots.yaml')
PONTOS = ('pronta', 'mostrar_apriltag', 'acima_slots')
ABERTA, FECHADA = 1.0471975511965976, 1.5707963267948966


def ler():
    return yaml.safe_load(open(ARQ)) or {}


def pose_gravada(nome):
    d = ler()
    if nome in PONTOS:
        return (d.get('pontos') or {}).get(nome)
    m = re.fullmatch(r's(\d)', nome)
    if m:
        return ((d.get('slots') or {}).get(int(m.group(1))) or {}).get('juntas')
    return None


def gravar_linha(padrao, novo):
    """Troca uma linha do yaml mantendo os comentarios; backup a cada gravacao."""
    txt = open(ARQ).read()
    novo_txt, n = re.subn(padrao, novo, txt, count=1, flags=re.M)
    if n != 1:
        raise ValueError('linha nao encontrada: %s' % padrao)
    yaml.safe_load(novo_txt)
    shutil.copy2(ARQ, '%s.bak_calib_%s' % (ARQ, time.strftime('%Y%m%d_%H%M%S')))
    open(ARQ, 'w').write(novo_txt)


def fmt(q):
    return '[%s]' % ', '.join('%.4f' % v for v in q)


def gravar(nome, q):
    if nome in PONTOS:
        gravar_linha(r'^(  %s:).*$' % nome, r'\1 %s' % fmt(q))
        return
    m = re.fullmatch(r's(\d)', nome)
    if not m:
        raise ValueError('nome invalido: %s' % nome)
    gravar_linha(r'^(  %s: \{juntas: )[^,]*(, .*)$' % m.group(1), r'\g<1>%s\2' % fmt(q))


def main():
    rclpy.init()
    c = ClienteGarra('calibrar_slots')
    c.aguardar()
    dedos = ActionClient(c, GripperCommand, '/garra_dedos_controller/gripper_cmd')
    passo = math.radians(3)

    def mover(alvo):
        if not all(lo <= v <= hi for v, (lo, hi) in zip(alvo, LIMITES)):
            print('fora dos limites das juntas %s' % LIMITES)
            return
        if not c.valido(alvo):
            print('pose em colisao (MoveIt): nao movo')
            return
        traj, cod = c.planejar_juntas(c.juntas_atuais(), alvo, velocidade=0.1)
        if cod != SUCESSO:
            print('planejamento falhou (codigo %d)' % cod)
            return
        cod = c.executar(traj)
        print('ok' if cod == SUCESSO else 'execucao falhou (codigo %d)' % cod)

    def garra(pos):
        if not dedos.wait_for_server(timeout_sec=5.0):
            print('sem /garra_dedos_controller/gripper_cmd')
            return
        g = GripperCommand.Goal()
        g.command.position = pos
        g.command.max_effort = 1.0
        f = dedos.send_goal_async(g)
        rclpy.spin_until_future_complete(c, f)
        if f.result() and f.result().accepted:
            r = f.result().get_result_async()
            rclpy.spin_until_future_complete(c, r, timeout_sec=10.0)
        print('dedos ok')

    print(__doc__.split('Comandos:')[1])
    while True:
        q = c.juntas_atuais()
        try:
            cmd = input('juntas %s (graus %s) > ' % (fmt(q), [round(math.degrees(v), 1) for v in q])).split()
        except EOFError:
            break
        if not cmd:
            continue
        try:
            m = re.fullmatch(r'([123])([+-])', cmd[0])
            if m:
                alvo = list(q)
                alvo[int(m.group(1)) - 1] += passo if m.group(2) == '+' else -passo
                mover(alvo)
            elif cmd[0] == 'passo':
                passo = math.radians(min(15.0, abs(float(cmd[1]))))
                print('passo %.1f graus' % math.degrees(passo))
            elif cmd[0] == 'abrir':
                garra(ABERTA)
            elif cmd[0] == 'fechar':
                garra(FECHADA)
            elif cmd[0] == 'ir':
                alvo = pose_gravada(cmd[1])
                if alvo is None:
                    print('%s ainda nao gravada' % cmd[1])
                else:
                    mover([float(v) for v in alvo])
            elif cmd[0] == 'gravar':
                gravar(cmd[1], q)
                print('%s = %s gravado em %s' % (cmd[1], fmt(q), os.path.normpath(ARQ)))
            elif cmd[0] == 'yplat':
                v = 'null' if cmd[2] == 'null' else '%.3f' % float(cmd[2])
                gravar_linha(r'^(  %d: \{juntas: (?:null|\[[^\]]*\]), y_plataforma: )[^}]*(\}.*)$' % int(cmd[1]),
                             r'\g<1>%s\2' % v)
                print('ok')
            elif cmd[0] == 'estado':
                print(open(ARQ).read())
            elif cmd[0] == 'sair':
                break
            else:
                print('comando desconhecido')
        except (ValueError, IndexError) as e:
            print('erro: %s' % e)
    c.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
