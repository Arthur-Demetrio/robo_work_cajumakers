"""Resumo objetivo das missoes gravadas (somente /rosout e CM): resultados por area, destravar, paradas CM, duracao."""
import sys, json, re, os, subprocess, tempfile, glob
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def ler(bagdir):
    comp = 'compression_format: zstd' in (Path(bagdir)/'metadata.yaml').read_text() or any(Path(bagdir).glob('*.zstd'))
    r = rosbag2_py.SequentialCompressionReader() if comp else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bagdir), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in ('/rosout', '/collision_monitor_state') if t in T]))
    while r.has_next():
        top, d, t = r.read_next(); yield top, deserialize_message(d, T[top]), t*1e-9
out = []
for m in sorted(glob.glob(os.path.expanduser('~/bags/missao*/')), key=lambda p: (int(re.search(r'missao(\d+)', p).group(1)), p)):
    b = Path(m)/'bag'
    if not (b/'metadata.yaml').exists(): continue
    nome = Path(m).name; notas = (Path(m)/'notas.txt').read_text(errors='ignore') if (Path(m)/'notas.txt').exists() else ''
    desc = re.search(r'descricao: (.*)', notas); desc = desc.group(1)[:90] if desc else ''
    res, versao, destr, cm, erros, t0, t1, goals = [], set(), 0, 0, 0, None, None, 0
    cm_ant = False
    try:
        for top, msg, t in ler(b):
            if top == '/collision_monitor_state':
                st = msg.action_type in (1,)  # stop
                if st and not cm_ant: cm += 1
                cm_ant = st; continue
            txt = msg.msg
            if msg.name == 'alinhador_passagem':
                for v in re.findall(r'v3\.\d+', txt): versao.add(v)
                if txt.startswith('RESULTADO:'):
                    res.append((round(t, 1), txt[11:80])); t0 = t0 or t; t1 = t
                if txt.startswith('Novo goal') or 'TRABALHADOR RECEBEU' in txt: goals += 'TRABALHADOR' in txt
            if msg.name == 'destravar' and ('DESTRAVAR: DECISAO' in txt or 'destravando' in txt.lower()): destr += 1
            if msg.level >= 40: erros += 1
    except Exception as e:
        out.append(dict(missao=nome, erro=str(e)[:120])); print(nome, 'ERRO', e); continue
    suc = sum('SUCESSO' in x[1] for x in res); fal = sum('FALHOU' in x[1] for x in res)
    d = dict(missao=nome, desc=desc, versoes=sorted(versao), goals=goals, sucesso=suc, falhou=fal, destravar=destr, paradas_cm=cm,
             erros=erros, duracao_resultados_s=round(t1-t0, 1) if t0 else None, resultados=res)
    out.append(d); print(nome, '|', desc[:60], '| v', ','.join(sorted(versao))[:20], '| goals', goals, 'ok', suc, 'falha', fal, 'destr', destr, 'cm', cm, 'err', erros, 'dur', d['duracao_resultados_s'])
json.dump(out, open(sys.argv[1], 'w'), indent=1, ensure_ascii=False)
