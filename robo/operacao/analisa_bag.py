#!/usr/bin/env python3
import sys, statistics
from datetime import datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def hora(t): return datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
def seg(st): return st.sec + st.nanosec * 1e-9

leitor = rosbag2_py.SequentialReader()
leitor.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id=''),
            rosbag2_py.ConverterOptions('', ''))
tipos = {t.name: get_message(t.type) for t in leitor.get_all_topics_and_types()}

series = {}
while leitor.has_next():
    topico, dados, t_rec = leitor.read_next()
    if topico in ('/tf_static', '/rosout'):
        continue
    msg = deserialize_message(dados, tipos[topico])
    rec = t_rec * 1e-9
    if topico == '/tf':
        for tr in msg.transforms:
            nome = f'TF {tr.header.frame_id} -> {tr.child_frame_id}'
            series.setdefault(nome, []).append((rec, seg(tr.header.stamp)))
    elif hasattr(msg, 'header'):
        series.setdefault(topico, []).append((rec, seg(msg.header.stamp)))

congelou_em = None
print('===== RESUMO POR FONTE (atraso = recebido - carimbo; negativo = carimbo no futuro) =====')
for nome, pts in sorted(series.items()):
    if len(pts) < 2:
        print(f'{nome}: apenas {len(pts)} mensagem'); continue
    dur = max(pts[-1][0] - pts[0][0], 1e-6)
    atrasos = [r - s for r, s in pts]
    voltas = [(r, s_ant - s) for (r, s), (_, s_ant) in zip(pts[1:], pts[:-1]) if s < s_ant]
    lacuna = max(b[0] - a[0] for a, b in zip(pts[:-1], pts[1:]))
    maior, ini_maior, ini, s_ini = 0.0, None, pts[0][0], pts[0][1]
    for r, s in pts[1:]:
        if s != s_ini:
            ini, s_ini = r, s
        elif r - ini > maior:
            maior, ini_maior = r - ini, ini
    print(f'\n{nome}: {len(pts)} msgs, {len(pts)/dur:.1f} Hz')
    print(f'  atraso: min {min(atrasos):.3f} | mediana {statistics.median(atrasos):.3f} | max {max(atrasos):.3f} s')
    print(f'  maior intervalo sem mensagens: {lacuna:.2f} s')
    print(f'  carimbos voltando no tempo: {len(voltas)}')
    for r, d in voltas[:5]:
        print(f'    em {hora(r)}: voltou {d:.3f} s')
    if ini_maior is not None:
        print(f'  maior período com carimbo PARADO: {maior:.1f} s, a partir de {hora(ini_maior)}')
    if nome == 'TF map -> odom' and maior > 2.0:
        congelou_em = ini_maior

if congelou_em:
    print(f'\n===== JANELA DO CONGELAMENTO DO map->odom ({hora(congelou_em)}, +/- 3 s) =====')
    for nome, pts in sorted(series.items()):
        jan = [(r, s) for r, s in pts if congelou_em - 3 <= r <= congelou_em + 3]
        if len(jan) < 2:
            print(f'{nome}: {len(jan)} msgs na janela'); continue
        atr = [r - s for r, s in jan]
        lac = max(b[0] - a[0] for a, b in zip(jan[:-1], jan[1:]))
        print(f'{nome}: {len(jan)} msgs | atraso min {min(atr):.3f} max {max(atr):.3f} s | maior lacuna {lac:.2f} s')
else:
    print('\nmap->odom NÃO congelou nesta gravação (ou não foi gravado).')
