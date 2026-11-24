"""Rebuild comparative artifacts from immutable evaluation records."""
import json,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent

def generate():
    rows=[];runs={}
    for run in sorted((ROOT/'runs').glob('*')):
        if not run.is_dir():continue
        records=[json.loads(p.read_text()) for p in sorted((run/'evaluations').glob('*.json'))]
        runs[run.name]=records
        for rec in records:
            for group,metrics in rec['metrics'].items():
                rows.append(dict(run=run.name,new_steps=rec['new_steps'],group=group,**metrics,angular_variance=rec['angular_variance'],angular_saturation=rec['telemetry']['saturation'][2],kl=rec['optimizer'].get('train/approx_kl'),clip_fraction=rec['optimizer'].get('train/clip_fraction')))
    if rows:
        with (ROOT/'comparacao.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    fig,axes=plt.subplots(3,3,figsize=(15,11))
    labels=[('success','Sucesso determinístico'),('collision','Colisões determinísticas'),('return_','Retorno do ambiente'),('angular_variance','Variância Gaussiana angular'),('angular_saturation','Saturação angular no treino'),('kl','KL aproximado latente'),('clip_fraction','Fração clipping PPO'),('timeout','Timeouts'),('distance','Distância final (m)')]
    colors={'controle':'tab:blue','entropia_zero':'tab:orange','variancia_angular':'tab:green'}
    for ax,(metric,title) in zip(axes.flat,labels):
        for name in runs:
            data=[r for r in rows if r['run']==name and r['group']=='fixed' and r.get(metric) is not None and (r['new_steps'] or metric not in ['angular_saturation'])]
            if not data:continue
            arm=name.rsplit('_s',1)[0];ax.plot([r['new_steps'] for r in data],[r[metric] for r in data],label=name,color=colors.get(arm),alpha=.75)
        ax.set_title(title);ax.set_xlabel('Novos passos');ax.grid(alpha=.3)
        if metric=='angular_variance':ax.set_yscale('log')
    handles,legend=axes[0,0].get_legend_handles_labels()
    if handles:fig.legend(handles,legend,loc='lower center',ncol=3,fontsize=8)
    fig.tight_layout(rect=[0,.07,1,1]);fig.savefig(ROOT/'comparacao.png',dpi=140);plt.close(fig)
    fig2,axes2=plt.subplots(2,3,figsize=(14,8))
    for row_index,group in enumerate(['holdout','stochastic']):
        for ax,metric in zip(axes2[row_index],['success','collision','return_']):
            for name in runs:
                data=[r for r in rows if r['run']==name and r['group']==group]
                if data:ax.plot([r['new_steps'] for r in data],[r[metric] for r in data],color=colors.get(name.rsplit('_s',1)[0]),alpha=.75,label=name)
            ax.set_title(group+' / '+metric);ax.grid(alpha=.3);ax.set_xlabel('Novos passos')
    fig2.tight_layout();fig2.savefig(ROOT/'validacao.png',dpi=140);plt.close(fig2)
    finished=(ROOT/'complete.json').exists()
    text=['# Piloto PPO Robo-WORK','',f'Estado: {"concluído" if finished else "EM ANDAMENTO — resultados parciais"}.',
          '','Três braços × três sementes (4201–4203); teto de 250.000 novos passos por réplica, avaliações a cada 25.000. Orçamento máximo total: 2.250.000 passos. Réplicas interrompidas são censuradas; não extrapolar ao orçamento completo.',
          '','![Comparação](comparacao.png)','','| Réplica | Novos passos avaliados | Sucesso fixo | Colisão fixa | Sucesso holdout | Estado |','|---|---:|---:|---:|---:|---|']
    for name,recs in runs.items():
        if not recs:continue
        r=recs[-1];s=ROOT/'runs'/name/'status.json';state=json.loads(s.read_text()) if s.exists() else {}
        text.append(f"| {name} | {r['new_steps']} | {r['metrics']['fixed']['success']:.1%} | {r['metrics']['fixed']['collision']:.1%} | {r['metrics']['holdout']['success']:.1%} | {state.get('reason') or ('completo' if state.get('completed') else 'em execução')} |")
    text+=['','![Holdout e execução estocástica](validacao.png)','','## Metodologia e limites','','Checkpoint comum: origem.zip (125.000 no contador da linhagem), com pesos e Adam restaurados. Supervisor novo em estágio 0; gate original (100 episódios, sucesso ≥90%, zero colisões). Física, recompensa, sampler, mapas e observações são cópias congeladas. O desvio angular inicial é cerca de 2,533; este desenho mede recuperação a partir de um checkpoint histórico anterior à explosão, e não prova resgate direto da política de 6 milhões de passos. Retomada não replica a trajetória RNG da execução histórica; entre braços a semente é pareada.',
           '','A proposta original era fatorial 2×2 e 500 mil passos. Este piloto aplica a solicitação atual de três braços e 250 mil passos por réplica. Sem o quarto braço, não se pode estimar a interação entre entropia zero e limite angular.',
           '','SB3 amostra u~Normal, executa clip(u), armazena u e log p(u). O estimador PPO sobre u é consistente para E[R(clip(u))]; a KL e a entropia registradas são da distribuição latente, não da distribuição física recortada (que possui massas nos limites). Nunca substituir u pelo valor recortado no buffer usando a mesma densidade Gaussiana. A entropia latente é um incentivo inadequado quando a ação física já satura.',
           '','O terceiro braço usa Adam projetado: apenas log_std[2] é projetado em [-3,0], antes da primeira coleta e após CADA optimizer.step. Os momentos são preservados e registrados. Não há projeção entre uma coleta e a avaliação de sua old_log_prob. O deslocamento inicial de variância é parte explícita da intervenção, não uma atualização PPO nem mudança nos pesos de média/crítico. Não se usa tanh sem Jacobiano.',
           '','Os rollouts nominais têm 2×256 transições. Para atingir exatamente cada marco de 25.000 sem ultrapassar o orçamento, apenas o rollout que cruza o marco é encurtado para 2×212 passos. O minibatch final parcial é aceito por SB3. A mesma agenda vale em todos os braços; checkpoints e avaliações ocorrem após otimização.',
           '','100 sementes fixas do estágio 0 selecionam checkpoint e governam o gate; 100 sementes holdout do estágio 0 são somente diagnósticas, sem seleção automática; 30 sementes distintas medem execução estocástica. Ao atingir o orçamento completo, 300 sementes inéditas e 100 da arena completa são avaliadas. RNG de treino é restaurado após cada avaliação. Os mesmos cenários são reutilizados entre políticas; não tratá-los como episódios independentes entre avaliações.',
           '','Interromper somente a réplica experimental se houver parâmetros não finitos; std angular >10 em três avaliações consecutivas; KL latente >0,2 em três avaliações; ou, após pelo menos 75 mil passos, três avaliações com sucesso fixo zero, colisões ≥98% e saturação angular acumulada ≥98%. Desempenho baixo isolado não interrompe o controle. A interrupção antecipada limita comparação a marcos comuns.',
           '','Execução sequencial, afinidade de uma CPU, threads BLAS/Torch=1, nice=15. Pausa entre rollouts quando MemAvailable<3 GiB ou carga de 1 min≥6,5; interrompe se RSS do processo exceder 2 GiB. Recursos registrados em resources.jsonl. O namespace de processos não permite inspecionar PIDs do host; nenhuma ausência de processos locais é interpretada como ociosidade.',
           '','## Recomendação para o próximo ciclo','']
    # Promotion rule uses fixed validation only; holdout reported separately.
    promising=[];paired={};rng=np.random.default_rng(9012)
    for arm in ['entropia_zero','variancia_angular']:
        effects=[];eligible=[]
        for seed in [4201,4202,4203]:
            a=runs.get(f'{arm}_s{seed}',[]);b=runs.get(f'controle_s{seed}',[])
            common=sorted(set(r['new_steps'] for r in a)&set(r['new_steps'] for r in b))
            if not common:continue
            step=common[-1];ar=next(r for r in a if r['new_steps']==step);br=next(r for r in b if r['new_steps']==step)
            for group in ['fixed','holdout']:
                am={r['seed']:r for r in ar['episodes'][group]};bm={r['seed']:r for r in br['episodes'][group]};ids=sorted(set(am)&set(bm))
                deltas={k:np.array([float(am[s][k])-float(bm[s][k]) for s in ids]) for k in ['success','collision','return_']}
                result={}
                for k,d in deltas.items():
                    boot=d[rng.integers(0,len(d),(2000,len(d)))].mean(1)
                    result[k]=dict(delta=float(d.mean()),ci95=np.quantile(boot,[.025,.975]).tolist())
                paired[f'{arm}_s{seed}_{group}']=dict(common_steps=step,**result)
            if step==250000 and len(a)>=4:
                recent=[r for r in a if r['new_steps']>0][-3:]
                bsteps={r['new_steps']:r for r in b}
                good=all(r['new_steps'] in bsteps and r['metrics']['fixed']['success']-bsteps[r['new_steps']]['metrics']['fixed']['success']>=.05 and r['metrics']['fixed']['collision']<=bsteps[r['new_steps']]['metrics']['fixed']['collision'] and r['metrics']['fixed']['success']>=a[0]['metrics']['fixed']['success']+.05 and r['metrics']['fixed']['collision']<=a[0]['metrics']['fixed']['collision'] for r in recent)
                eligible.append(good)
        if len(eligible)==3 and all(eligible):promising.append(arm)
    (ROOT/'paired_statistics.json').write_text(json.dumps(paired,indent=2))
    if not finished:text+=['Aguardar o término das réplicas antes de escolher configuração. Os dados parciais não demonstram recuperação sustentada.']
    elif promising:text+=[f'Candidatas para confirmação independente: {", ".join(promising)}. Superaram o controle e sua própria linha de base em ≥5 pontos percentuais de sucesso, sem piorar colisões, nos últimos três marcos fixos de cada uma das três sementes. Confirmar em novas sementes e teste inédito antes de estender o treinamento.']
    else:text+=['Nenhuma configuração satisfez o critério pré-definido nas três sementes. Não estender automaticamente nem promover por recompensa. Próximo ciclo: testar target_kl=0,02 ou learning_rate=1e-4, uma alteração por vez, mantendo recompensa e física. Se o limite angular controlar a variância mas não o sucesso, tratar como correção numérica sem alegar recuperação de navegação.']
    text+=['','## Teste final inédito e arena completa','','| Réplica | Conjunto | Sucesso | Colisão | Retorno |','|---|---|---:|---:|---:|']
    for r in rows:
        if r['group'] in ['unseen','full_arena']:text.append(f"| {r['run']} | {r['group']} | {r['success']:.1%} | {r['collision']:.1%} | {r['return_']:.2f} |")
    verification=ROOT/'preservation_after.json'
    if verification.exists():
        v=json.loads(verification.read_text());text+=['',f"Preservação: {v['files_verified']} arquivos originais verificados; alterações detectadas: {len(v['changed'])}."]
    text+=['','Diferenças pareadas e IC95% bootstrap por cenário, separados por semente de treino, estão em paired_statistics.json. São intervalos exploratórios condicionados às sementes treinadas, sem correção por comparações múltiplas; não estimam sozinhos a variabilidade entre treinos. Recompensa maior sem sucesso maior e colisões menores não qualifica uma configuração.','']
    (ROOT/'RELATORIO.md').write_text('\n'.join(text))
if __name__=='__main__':generate()
