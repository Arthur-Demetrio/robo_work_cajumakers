# Correção do treinamento — 11/11/2026

O teste de 10/11 parou em 125.000 por early stopping, não por total_timesteps.
A configuração antiga tinha steps=2.000.000, eval_every=25.000 e patience=4.
A avaliação de 25.000 estabeleceu o melhor score (-0,84); as quatro seguintes não melhoraram: -0,86, -0,87, -0,96 e -0,94. O callback retornava False na quinta avaliação. status.json confirma stopped=true e completed=false.
O max_steps=1500 limita cada episódio; o vetor reinicia episódios e continua contando as transições.

## Alterações

- experimentos_v2_20261110/train_gated.py: --steps 0 (padrão) executa continuamente; --steps positivo continua disponível como meta total, não adicional. Parada por desempenho só com --early-stop explícito. Gates de progressão continuam ativos. Salva a cada 25.000 transições, métricas de episódios, avaliações e otimizador. --resume-from restaura política, otimizador e contador com reset_num_timesteps=False; valida layout/currículo e recupera estágio. Salva estado da supervisão para próximas retomadas. Diretório novo obrigatório, checkpoints nunca sobrescritos.
- iniciar_treino_continuo.py: lança execução com sessão independente, stdin fechado, saída em arquivo, SIGHUP ignorado, prioridade reduzida e afinidade de duas CPUs.
- tests/test_gated_resume.py: regressão da parada em 125.000 e salvamento em 150.000, incluindo parada solicitada.

Backup original: backups/treino_20261111_102926/ (script e configurações/registros antigos).
Não foram alteradas arquitetura, recompensas ou hiperparâmetros PPO. Novos episódios usam novo fluxo de RNG; retomada não reproduz exatamente a trajetória interrompida nem o rollout parcial antigo.

## Checkpoint e contador

Fonte: /home/marcos/robo_rl_v2/runs/treino_v2_nativo_gated_20261110_102312/checkpoints/ppo_000000125000.zip
SHA256: aa8309e767d3ff5e7267abfa92bc38dbde6ed9f7db2dbb839f6650049c251220
Contador inicial: 125.000; meta: sem limite, conforme orientação posterior do usuário.
O histórico anterior de transferência registra 2.347.094 passos adicionais de linhagem: total histórico inicial 2.472.094. O contador real do checkpoint desta etapa continua a partir de 125.000, sem somar artificialmente o histórico.

## Validação

- ZIP original íntegro, inclui policy.pth e policy.optimizer.pth.
- Teste real em runs/validacao_retomada_20261111: contador 125.000 → 126.024; checkpoints em 125.512 e 126.024.
- Pesos e estado completo do otimizador conferidos tensor a tensor na restauração; pesos evoluíram depois do treino.
- Teste de regressão do callback: aprovado, inclusive em 150.000 com platô.
- Três testes existentes de dinâmica/sensor nativo: aprovados.

## Execução

Comando usado, a partir de /home/marcos/robo_rl_v2:

    .venv/bin/python iniciar_treino_continuo.py --output runs/treino_continuo_20261111

O launcher usa train_gated.py --resume-from runs/treino_v2_nativo_gated_20261110_102312/checkpoints/ppo_000000125000.zip --steps 0 --checkpoint-every 25000.
PID inicial: 24377. Processo independente do terminal; fechar o terminal não encerra o treino. Reinicialização do computador requer nova retomada.

Logs: runs/treino_continuo_20261111.terminal.log
Contador ao vivo: runs/treino_continuo_20261111/live.json
Checkpoints: runs/treino_continuo_20261111/checkpoints/
Último checkpoint: runs/treino_continuo_20261111/latest.json
Métricas: episodes.csv, optimizer/progress.csv e evaluations/*.json dentro dessa pasta.
Avaliações periódicas pausam temporariamente o contador enquanto avaliam 100 episódios.

Acompanhar:

    tail -f /home/marcos/robo_rl_v2/runs/treino_continuo_20261111.terminal.log
    watch -n 5 cat /home/marcos/robo_rl_v2/runs/treino_continuo_20261111/live.json

Parar com segurança:

    touch /home/marcos/robo_rl_v2/runs/treino_continuo_20261111/stop.request

Aguarde status.json, final.zip e remoção de process.json. Se estiver avaliando, aguarde terminar a avaliação para processar a solicitação. Evite kill -9.

Retomar após parar: obtenha model de latest.json e execute iniciar_treino_continuo.py --resume-from CAMINHO_DO_CHECKPOINT --output NOVA_PASTA.

Os resultados antigos têm baixa taxa de sucesso e muitas colisões no simulador. Remover early stopping permite continuar aprendendo, mas não garante melhora de desempenho; as métricas continuam registradas.

## Confirmação em execução

PID 24377 ativo, com sessão e grupo próprios (24377). Checkpoint real de 150.000 passos salvo e íntegro; hash do checkpoint original conferido e preservado. episodes.csv e optimizer/progress.csv cresceram após 125.000. Execução em avaliação periódica de 100 episódios na marca de 150.000.
