#!/usr/bin/env python3
"""Compara, por caminho do FollowPath, o 'chegou' do goal checker real no modo atual x arvore (GOAL_TF_ATUAL)
com os 'Reached the goal!' reais do bag. Uso: comparar_replay_bag_inteiro.py <dados_bag> <replay_por_caminho.txt>"""
import sys, re, json, datetime
H = lambda t: datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4]
reais = [json.loads(l)['t'] for l in open(sys.argv[1] + '/rosout.jsonl') if 'Reached the goal' in l]
por = {}
for m in re.finditer(r'caminho modo=(\w+) sf=([\d.]+) chegou_t=([-\d.]+)', open(sys.argv[2]).read()):
    por.setdefault(float(m.group(2)), {})[m.group(1)] = float(m.group(3))
res = {'real_e_arvore': 0, 'real_sem_arvore': 0, 'arvore_a_mais': 0, 'atual_igual_real': 0, 'atual_sem_real': 0}
print('Reached reais:', [H(t) for t in reais])
for sf in sorted(por):
    d = por[sf]
    at, av = d.get('atual', -1), d.get('arvore', -1)
    real = [t for t in reais if sf <= t <= sf + 600 and (not [s for s in por if sf < s <= t])]
    if at > 0 or av > 0 or real:
        print('caminho %s | real=%s | replay atual=%s | arvore=%s' % (H(sf), H(real[0]) if real else '-',
              H(at) if at > 0 else '-', H(av) if av > 0 else '-'))
    if real and av > 0: res['real_e_arvore'] += 1
    if real and av < 0: res['real_sem_arvore'] += 1
    if not real and av > 0: res['arvore_a_mais'] += 1
    if real and at > 0 and abs(at - real[0]) < 0.25: res['atual_igual_real'] += 1
    if at > 0 and not real: res['atual_sem_real'] += 1
print(res)
