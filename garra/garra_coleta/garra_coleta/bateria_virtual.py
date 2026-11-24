#!/usr/bin/env python3
"""Bateria de testes virtuais da pegada e da entrega, sem a garra conectada.

Roda contra o MoveIt simulado (garra_moveit_config demo.launch.py). Para cada estrutura de
config/bateria_virtual.yaml (chao, workstations, estantes) e cada posicao de cubo ao longo da
profundidade, monta o plano do pegar_cubo e confere:

  (a) como o pegar_cubo esta hoje, SEM conhecer a estrutura: o plano e feito com a cena vazia e
      cada ponto de cada trajetoria e conferido contra a estrutura (colisao = falha);
  (b) se o planejador conhecesse a estrutura: o mesmo plano com a estrutura na cena do MoveIt.

Contato dos DEDOS com a superficie onde o cubo esta, com a ponta a menos de RAIO_PEGADA do cubo, vira
AVISO, nao falha: no modelo os dedos passam ~2 cm abaixo do tool0, mas a garra real pega no chao com a
ponta no centro do cubo sem encostar (2026-11-24). Contato longe do cubo (borda da mesa, teto da
estante) ou de outras partes do braco e falha. O chao e o proprio robo_base (placa do URDF em z <= 0).
Os x NAO passam pela CORRECAO_X do pegar_cubo: testa o modelo, nao a calibracao da garra real.

Uso:
    ros2 launch garra_moveit_config demo.launch.py use_rviz:=false     # outro terminal
    ros2 run garra_coleta bateria_virtual                              # tudo
    ros2 run garra_coleta bateria_virtual --estrutura mesa_10cm --acao pegar
    ros2 run garra_coleta bateria_virtual --config minhas_medidas.yaml --csv saida.csv
"""

import argparse
from contextlib import redirect_stdout
import csv
import io
import math
import os
import sys
import time
from collections import Counter

import rclpy
import yaml

from garra_coleta import estruturas, pegar_cubo as pc
from garra_coleta.estruturas import (  # noqa: F401 (reexportados para os testes)
    PREFIXO, Cena, caixa, caixas_da_estrutura, config_padrao)
from garra_coleta.cliente_garra import ClienteGarra, PRONTA, SUCESSO

# Links das pontas dos dedos e raio (m) em torno do cubo onde o contato deles com a superficie e aviso.
DEDOS = ('P_1', 'P_1_1')
RAIO_PEGADA = 0.03


class Bateria:

    def __init__(self, c):
        self.c = c
        self.coleta = pc.Coleta(c, 0.2)
        self._cena = Cena(c)

    def cena(self, objetos, superficie=None):
        """Troca as caixas; com a superficie, libera o contato das pontas dos dedos com ela (ACM)."""
        self._cena.trocar(objetos, estruturas.dedos_na_superficie(superficie) if superficie else ())

    # --- plano completo, como no pegar_cubo ---

    def fases(self, acao, x, z, espera=PRONTA):
        """Lista [(nome, trajetoria)] do ciclo inteiro, ou (None, motivo)."""
        c, v = self.c, 0.2
        descer_mm = round(pc.interpolar(pc.DESCER_POR_X, x), 1) if acao == 'pegar' else pc.DESCER_ENTREGA_MM
        with redirect_stdout(io.StringIO()):   # o pegar_cubo imprime cada inclinacao testada
            plano = self.coleta.plano_de_pegada(x, z, PRONTA, descer_mm / 1000.0)
            descer = None if plano is None else pc.descida_usada(plano, descer_mm)
        if plano is None:
            return None, 'fora do alcance', None
        if descer is None:
            return None, 'descida nao cabe', plano['graus']
        a = math.radians(plano['graus'])
        fases = []
        traj, cod = c.planejar_juntas(espera, plano['q_pre'], v)
        if cod != SUCESSO:
            return None, f'transporte nao planeja ({cod})', plano['graus']
        fases.append(('transporte', traj))
        q = fim(traj)
        ida = pc.para_garra_base((pc.RECUO * math.cos(a), 0.0, pc.RECUO * math.sin(a)))
        entrada = c.linha_cartesiana(q, ida, velocidade=v)
        if entrada.fracao < 0.99:
            return None, f'entrada incompleta ({entrada.fracao:.0%})', plano['graus']
        fases.append(('entrada', entrada.trajetoria))
        q = fim(entrada.trajetoria)
        if descer > 0.0:
            d = c.linha_cartesiana(q, (0.0, 0.0, -descer), evitar_colisao=False, velocidade=v)
            fases.append(('descida extra', d.trajetoria))
            q = fim(d.trajetoria)
        if acao == 'pegar':
            saida = c.linha_cartesiana(q, (0.0, 0.0, self.coleta.subida), evitar_colisao=False, velocidade=v)
            nome_saida = 'subida'
        else:
            if descer > 0.0:
                s = c.linha_cartesiana(q, (0.0, 0.0, descer), evitar_colisao=False, velocidade=v)
                fases.append(('saida da descida', s.trajetoria))
                q = fim(s.trajetoria)
            saida = c.linha_cartesiana(q, tuple(-k for k in ida), velocidade=v)
            nome_saida = 'recuo'
        if saida.fracao < 0.99:
            return None, f'{nome_saida} incompleta ({saida.fracao:.0%})', plano['graus']
        fases.append((nome_saida, saida.trajetoria))
        q = fim(saida.trajetoria)
        if acao == 'pegar' and espera != PRONTA:   # estante: o pegar_cubo recua antes de voltar
            r = c.linha_cartesiana(q, tuple(-k for k in ida), velocidade=v)
            if r.fracao < 0.99:
                return None, f'recuo da estante incompleto ({r.fracao:.0%})', plano['graus']
            fases.append(('recuo da estante', r.trajetoria))
            q = fim(r.trajetoria)
        traj, cod = c.planejar_juntas(q, espera, v)
        if cod != SUCESSO:
            return None, f'volta nao planeja ({cod})', plano['graus']
        fases.append(('volta', traj))
        return fases, '', plano['graus']

    def colisoes(self, fases, cubo, superficie):
        """(falhas, avisos): listas de 'fase: corpo x corpo' encontradas nas trajetorias."""
        falhas, avisos = [], []
        for nome, traj in fases:
            for ponto in traj.joint_trajectory.points:
                q = list(ponto.positions)
                ok, contatos = self.c.valido(q)
                if ok:
                    continue
                perto = None
                for par in contatos:
                    txt = f'{nome}: {par[0]} x {par[1]}'
                    aviso = False
                    if superficie in par and any(d in par for d in DEDOS):
                        if perto is None:
                            p = self.c.fk(q, pc.FRAME).position
                            perto = math.hypot(p.x - cubo[0], p.z - cubo[1]) < RAIO_PEGADA
                        aviso = perto
                    alvo = avisos if aviso else falhas
                    if txt not in alvo:
                        alvo.append(txt)
        return falhas, avisos

    def caso(self, objetos, superficie, acao, x, z, estante=''):
        """Resultado de um cubo: dict com o que deu em (a) e em (b).

        Estante (nome): o pegar_cubo ja recebe a estante (--estante / parametro), poe ela na cena e
        espera na pose que usar_estante escolher; entao (a) e (b) sao o mesmo teste."""
        r = dict(acao=acao, x=round(x, 3), z=round(z, 3))
        if estante:
            self.cena([])
            try:
                with redirect_stdout(io.StringIO()):
                    self.coleta.usar_estante(estante, mover=False)
                espera = self.coleta.espera
            except RuntimeError as erro:
                espera, r['inclinacao'], r['atual'], r['detalhe_atual'] = None, None, 'falha', str(erro)
            if espera is not None:
                fases, motivo, r['inclinacao'] = self.fases(acao, x, z, espera)
                if fases is None:
                    r['atual'], r['detalhe_atual'] = 'falha', motivo
                else:
                    falhas, avisos = self.colisoes(fases, (x, z), superficie)
                    r['atual'] = 'colide' if falhas else ('aviso' if avisos else 'ok')
                    r['detalhe_atual'] = '; '.join(falhas or avisos)
            r['com_cena'], r['detalhe_com_cena'] = r['atual'], 'estante: sempre com a cena'
            with redirect_stdout(io.StringIO()):
                self.coleta.usar_estante('', mover=False)
            return r
        # (a) pegar_cubo de hoje: planeja sem a estrutura, confere contra ela.
        self.cena([])
        fases, motivo, graus = self.fases(acao, x, z)
        r['inclinacao'] = graus
        if fases is None:
            r['atual'], r['detalhe_atual'] = 'falha', motivo
        else:
            self.cena(objetos, superficie)
            falhas, avisos = self.colisoes(fases, (x, z), superficie)
            r['atual'] = 'colide' if falhas else ('aviso' if avisos else 'ok')
            r['detalhe_atual'] = '; '.join(falhas or avisos)
        # (b) com a estrutura na cena do planejador.
        if r['atual'] in ('ok', 'aviso'):
            r['com_cena'], r['detalhe_com_cena'] = r['atual'], 'igual ao atual'
            return r
        self.cena(objetos, superficie)
        fases, motivo, _ = self.fases(acao, x, z)
        if fases is None:
            r['com_cena'], r['detalhe_com_cena'] = 'falha', motivo
        else:
            falhas, avisos = self.colisoes(fases, (x, z), superficie)
            r['com_cena'] = 'colide' if falhas else ('aviso' if avisos else 'ok')
            r['detalhe_com_cena'] = '; '.join(falhas or avisos)
        return r


def fim(traj):
    return list(traj.joint_trajectory.points[-1].positions)


def faixas(xs, passo):
    """[0.31, 0.32, 0.33, 0.36] -> '31-33, 36 cm'."""
    if not xs:
        return '-'
    grupos, ini, ant = [], xs[0], xs[0]
    for x in xs[1:]:
        if x - ant > passo * 1.5:
            grupos.append((ini, ant))
            ini = x
        ant = x
    grupos.append((ini, ant))
    return ', '.join(f'{a * 100:.0f}' if a == b else f'{a * 100:.0f}-{b * 100:.0f}' for a, b in grupos) + ' cm'


def resumo(nome, borda, linhas, passo):
    print(f'\n=== {nome} ===')
    for acao in ('pegar', 'entregar'):
        rs = [r for r in linhas if r['acao'] == acao]
        if not rs:
            continue
        ok = [r for r in rs if r['atual'] in ('ok', 'aviso')]
        cena = [r for r in rs if r['com_cena'] in ('ok', 'aviso')]
        dist = lambda grupo: faixas([round(r['x'] - borda, 3) for r in grupo], passo)
        print(f'  {acao:8s} hoje: {len(ok):3d}/{len(rs)}  (da borda: {dist(ok)})')
        print(f'  {"":8s} com cena: {len(cena):3d}/{len(rs)}  (da borda: {dist(cena)})')
        avisos = sum(r['atual'] == 'aviso' for r in rs)
        if avisos:
            print(f'  {"":8s} {avisos} so com os dedos na superficie junto do cubo (modelo dos dedos, ver doc)')
        motivos = Counter(r['detalhe_atual'].split(';')[0] for r in rs if r['atual'] in ('falha', 'colide'))
        for m, n in motivos.most_common(4):
            print(f'  {"":8s} - {n}x {m}')


def argumentos_sem_ros():
    try:
        from rclpy.utilities import remove_ros_args
        return remove_ros_args(sys.argv)[1:]
    except ImportError:
        return sys.argv[1:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=config_padrao(), help='YAML das estruturas')
    ap.add_argument('--estrutura', action='append', help='so esta(s) estrutura(s) (repetivel)')
    ap.add_argument('--acao', choices=['pegar', 'entregar', 'ambas'], default='ambas')
    ap.add_argument('--passo', type=float, help='passo em x (m); padrao do YAML')
    ap.add_argument('--csv', default=None, help='arquivo de saida (padrao: logs/bateria_<hora>.csv)')
    args = ap.parse_args(argumentos_sem_ros())

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    passo = args.passo or cfg['passo']
    estruturas = [e for e in cfg['estruturas'] if not args.estrutura or e['nome'] in args.estrutura]
    if not estruturas:
        ap.error('nenhuma estrutura com esse nome no YAML')
    acoes = ['pegar', 'entregar'] if args.acao == 'ambas' else [args.acao]
    saida = args.csv or os.path.join(os.path.expanduser('~/work/logs'),
                                     time.strftime('bateria_%Y%m%d_%H%M%S.csv'))
    os.makedirs(os.path.dirname(os.path.abspath(saida)), exist_ok=True)

    rclpy.init()
    c = ClienteGarra('bateria_virtual')
    todas = []
    try:
        c.aguardar()
        b = Bateria(c)
        y = c.fk(PRONTA, pc.FRAME).position.y
        meio = cfg['cubo'] / 2
        for e in estruturas:
            objetos, sup, corpo_sup = caixas_da_estrutura(e, y, cfg['largura_y'])
            n = int(round((cfg['x_max'] - cfg['x_min']) / passo)) + 1
            xs = [e['borda_x'] + cfg['x_min'] + i * passo for i in range(n)]
            if e['tipo'] != 'chao':
                xs = [x for x in xs if x + meio <= e['borda_x'] + e['profundidade']]
            linhas = []
            for acao in acoes:
                for x in xs:
                    print(f"[{e['nome']}] {acao} x={x:.3f} ...", flush=True)
                    r = dict(estrutura=e['nome'], **b.caso(objetos, corpo_sup, acao, x, sup + meio, e['nome'] if e['tipo'] == 'estante' else ''))
                    linhas.append(r)
            todas += linhas
            resumo(e['nome'], e['borda_x'], linhas, passo)
        b.cena([])
    except RuntimeError as erro:
        print(f'ERRO: {erro}')
        return 1
    except KeyboardInterrupt:
        print('Interrompido; salvando o que ja foi testado.')
    finally:
        if todas:
            with open(saida, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(todas[0].keys()))
                w.writeheader()
                w.writerows(todas)
            print(f'\nResultados em {saida}')
        c.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
