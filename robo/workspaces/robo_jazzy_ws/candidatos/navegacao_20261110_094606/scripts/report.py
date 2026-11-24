"""Produce a reproducible evidence bundle, including hashes of candidate inputs."""
from pathlib import Path
import json,hashlib,difflib
ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT/'src/robo_navigation'
rows=json.loads((ROOT/'review/antecipacao/native_comparison.json').read_text())
lines=['# Comparação offline', '', 'Plugins Navfn e DWB reais; integração ideal dos comandos, sem física, BT, Collision Monitor ou recuperação neste laço. Tempos não são medições no robô.', '',
'| Perfil | Cenário | Replaneja | Chegou | Duração s | Sem translação s | Yaw intermediário rad | Primeiro lateral x m | Interseções |',
'|---|---|---|---|---:|---:|---:|---:|---:|']
for r in rows:
 if r['profile'] in ('shim','dwb','early','predictive'):
  lines.append('| {profile} | {scenario} | {replanning} | {complete} | {duration_s} | {no_translation_s} | {intermediate_rotation_rad} | {first_lateral_x} | {collisions} |'.format(**r))
lines+=['','Caixa inicia em x=0,85 m; frente do footprint em x=0,365 m. x=-1 significa nenhum comando lateral. Primeiro lateral é indicador, não prova isolada de desvio útil. Yaw intermediário não classifica automaticamente todo giro como desnecessário.', '',
'Recuperações não medidas neste laço. Os contornos concluídos não executaram recuperação. A árvore integrada ainda precisa de ensaios fechados com todos os nós e sensores. As interseções da referência são geométricas neste modelo, não colisões físicas comprovadas.']
(ROOT/'review/COMPARATIVO.md').write_text('\n'.join(lines)+'\n')
a=(NAV/'config/nav2_params_shim_reference.yaml').read_text().splitlines(True)
b=(NAV/'config/nav2_params.yaml').read_text().splitlines(True)
(ROOT/'review/nav2_params.diff').write_text(''.join(difflib.unified_diff(a,b,fromfile='snapshot_local/nav2_params.yaml',tofile='candidata/nav2_params.yaml')))
paths=[NAV/'config/nav2_params.yaml',NAV/'behavior_trees/navigate_to_pose_candidate.xml',NAV/'launch/navigation_candidate.launch.py',ROOT/'src/robo_recovery/robo_recovery/supervised.py',ROOT/'src/robo_recovery/robo_recovery/supervised_node.py']
(ROOT/'review/sha256_candidata.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},indent=2))
