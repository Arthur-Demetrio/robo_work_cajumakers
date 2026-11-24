import argparse,csv,json
from pathlib import Path
import numpy as np
from .visual import plt

def report(run):
    run=Path(run);rows=list(csv.DictReader((run/'episodes.csv').open()))
    if not rows:raise ValueError('Sem episódios completos ainda')
    fig,axes=plt.subplots(3,2,figsize=(11,10));x=[int(r['steps_total']) for r in rows]
    for ax,key in zip(axes.flat,['success','collision','path_length','stopped_time','rotations','oscillations']):
        values=np.array([float(r[key]) if r[key] not in ('True','False') else float(r[key]=='True') for r in rows])
        window=min(20,len(rows));smooth=np.convolve(values,np.ones(window)/window,'valid')
        ax.plot(x,values,alpha=.3);ax.plot(x[window-1:],smooth);ax.set_title(key);ax.set_xlabel('Transições')
    fig.tight_layout();fig.savefig(run/'training_metrics.png');plt.close(fig)
    summaries=[]
    for path in sorted((run/'evaluations').glob('*/summary.json'),key=lambda p:int(p.parent.name)):
        summary=json.loads(path.read_text());summaries.append((int(path.parent.name),summary))
    fig,ax=plt.subplots(figsize=(9,4))
    for scenario in (summaries[0][1] if summaries else []):
        ax.plot([s for s,_ in summaries],[v[scenario]['success'] for _,v in summaries],marker='o',label=scenario)
    ax.set_xlabel('Transições');ax.set_ylabel('Taxa de sucesso em avaliação');ax.set_ylim(-.02,1.02)
    if summaries:ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(run/'evaluation_history.png');plt.close(fig)
    text=f'# Relatório de execução\n\nEpisódios completos registrados: {len(rows)}.\n\n'
    text+='Estes resultados medem simulação; um treino curto valida a infraestrutura e não comprova navegação aprendida. Não houve comparação com DWB.\n\n'
    if summaries:
        text+='Última avaliação: '+str(summaries[-1][0])+' transições.\n\n| Cenário | Sucesso | Colisão | Tempo médio (s; inclui falhas) |\n|---|---:|---:|---:|\n'
        for scenario,v in summaries[-1][1].items():text+=f"| {scenario} | {v['success']:.1%} | {v['collision']:.1%} | {v['elapsed']:.2f} |\n"
    text+='\nO tempo de conclusão apenas dos sucessos está em summary.json; null significa nenhum sucesso. Recuperação = saída de janela sem progresso; oscilação = inversão de sinal de comando acima do limiar.\n'
    (run/'REPORT.md').write_text(text)
    print(run/'REPORT.md')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',default='runs/principal');a=p.parse_args();report(a.run)
