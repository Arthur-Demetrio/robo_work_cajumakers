# Execução isolada do piloto PPO

A bateria iniciada executa uma réplica de cada vez. Os braços são `controle` (ent_coef=0,01), `entropia_zero` (ent_coef=0) e `variancia_angular` (ent_coef=0,01; somente log_std angular projetado em [-3,0]). As sementes pareadas são 4201, 4202 e 4203. Cada réplica tem teto de 250.000 passos adicionais; máximo total 2.250.000 passos.

- `RELATORIO.md`: estado, metodologia, critérios, comparação e recomendação automática ao término.
- `comparacao.png` e `validacao.png`: gráficos atualizados após avaliações.
- `comparacao.csv`: métricas tabulares.
- `paired_statistics.json`: comparações pareadas por cenário, separadas por semente.
- `training.log`: avaliações concluídas e eventuais exceções. A primeira avaliação precede o treinamento.
- `runs/<braço>_s<semente>/`: config, avaliações completas, episódios, snapshots e recursos.
- `complete.json`: criado somente quando todas as réplicas foram executadas ou interrompidas pelos critérios do protocolo.
- `validation.json`, `tests_pilot.log`, `tests_regression.log`: comprovação dos 46 testes aprovados antes da bateria.
- `snapshot_manifest.json`, `versions.json`: fontes e dependências usados.
- `preservation_before.json`, `preservation_after.json`: hashes dos arquivos originais; nova verificação automática ao final.

Para acompanhar, ler `training.log` e os arquivos `live.json` e `resources.jsonl` das réplicas. O PID de `process.json` pertence ao namespace da ferramenta e pode não corresponder ao PID visível no host.

Para solicitar parada segura de toda a bateria, criar `stop.request` nesta pasta. O treino salva ao próximo ponto seguro; uma avaliação em curso termina antes de atender ao pedido. Nunca criar esse arquivo na execução original para controlar este piloto.

O relatório pode ser regenerado sem treinar, a partir desta pasta:

```bash
../.venv/bin/python report.py
```

Não iniciar uma segunda instância de `experiment.py` nesta pasta. Réplicas recusam diretórios existentes. A execução original e seus relatórios não são destinos de escrita deste piloto.

As seis falhas da primeira tentativa de regressão eram fixtures ausentes na cópia isolada. As fixtures foram copiadas, e a suíte completa de 39 testes passou. O piloto também passou em sete testes próprios, incluindo uma execução reduzida com coleta, treino, avaliação e checkpoint reais.
