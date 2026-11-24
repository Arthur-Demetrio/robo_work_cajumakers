#!/usr/bin/env python3
"""Para cada 'Reached the goal!' real do bag: o goal checker reproduzido (sonda goal_checker_replay, POR_CICLO=1)
diz 'chegou' no ciclo do controller daquele instante (+-0,15 s)? Modo atual = fidelidade do modelo;
modo arvore = sem regressao do GOAL_TF_ATUAL. Uso: checar_reached_reais.py <dados_bag> <saida_por_ciclo.txt>"""
import sys, re, json, datetime
H = lambda t: datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4]
reais = [json.loads(l)['t'] for l in open(sys.argv[1] + '/rosout.jsonl') if 'Reached the goal' in l]
cic = {}
for m in re.finditer(r'ciclo modo=(\w+) t=([\d.]+) chegou=(\d)', open(sys.argv[2]).read()):
    cic.setdefault(m.group(1), []).append((float(m.group(2)), m.group(3) == '1'))
ok = {'atual': 0, 'arvore': 0}
for t in reais:
    linha = [H(t)]
    for modo in ('atual', 'arvore'):
        v = [c for (tc, c) in cic.get(modo, []) if abs(tc - t) <= 0.15]
        r = any(v)
        ok[modo] += r
        linha.append('%s=%s' % (modo, 'chegou' if r else ('NAO' if v else 'sem_ciclo')))
    print(' '.join(linha))
print('Reached reais: %d | reproduzidos no modo atual: %d | na arvore GOAL_TF_ATUAL: %d' % (len(reais), ok['atual'], ok['arvore']))
