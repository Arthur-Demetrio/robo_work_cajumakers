"""Read-only live terminal monitor for the paired PPO battery."""
import csv,json,os,time
from pathlib import Path
R=Path(__file__).resolve().parent
while True:
 try:
  print('\033[2J\033[H',end='')
  print('ROBO-WORK — TREINAMENTO PPO EM ANDAMENTO\n')
  p=R/'process.json';state=json.loads(p.read_text()) if p.exists() else {}
  pid=state.get('pid');alive=bool(pid and Path(f'/proc/{pid}/cmdline').exists() and b'experiment.py' in Path(f'/proc/{pid}/cmdline').read_bytes())
  print(f'Processo: {pid or "aguardando"} | '+('ATIVO' if alive else 'SEM PROCESSO ATIVO'))
  print('3 braços × 3 sementes | 250.000 passos/réplica | avaliações a cada 25.000\n')
  for seed in [4201,4202,4203]:
   for arm in ['controle','entropia_zero','variancia_angular']:
    run=R/'runs'/f'{arm}_s{seed}'
    if not run.exists():print(f'{arm:20} s{seed}  aguardando');continue
    status=json.loads((run/'status.json').read_text()) if (run/'status.json').exists() else {}
    progress=run/'optimizer/progress.csv';rows=[]
    if progress.exists():
     with progress.open() as f:rows=list(csv.DictReader(f))
    count=len(rows);steps=min(250000,(count//49)*25000+(count%49)*512)
    print(f'{arm:20} s{seed}  {steps:>7,}/250,000 passos  '+('CONCLUÍDO' if status.get('completed') else status.get('reason') or 'em execução'))
    if rows:
     last=rows[-1];print('   '+' | '.join(f'{k.removeprefix("train/")}={last[k]}' for k in ['train/approx_kl','train/clip_fraction','train/entropy_loss','train/explained_variance'] if k in last))
    records=sorted((run/'evaluations').glob('*.json'))
    if records:
     ev=json.loads(records[-1].read_text());m=ev['metrics']['fixed'];h=ev['metrics']['holdout']
     print(f"   Avaliação {ev['new_steps']:,}: sucesso={m['success']:.1%} colisões={m['collision']:.1%} retorno={m['return_']:.2f} | holdout sucesso={h['success']:.1%} | std angular={ev['std'][2]:.3f}")
  print('\nA janela gráfica reproduz checkpoints; este painel acompanha as atualizações reais.')
  print('Durante avaliações, o contador de passos permanece parado. Ctrl+C fecha só este painel.')
  print('\nÚltimos registros:')
  log=R/'training.log'
  if log.exists():
   for line in log.read_text().splitlines()[-2:]:print(line[:400])
  time.sleep(2)
 except KeyboardInterrupt:break
 except (OSError,ValueError,KeyError):time.sleep(2)
