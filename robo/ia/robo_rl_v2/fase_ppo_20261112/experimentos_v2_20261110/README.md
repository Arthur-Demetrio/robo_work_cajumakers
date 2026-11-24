# Experimentos para acelerar o desenvolvimento da política v2

[Relatório com resultados e gráficos](../runs/desenvolvimento_v2_20261110_093450/RELATORIO.md).

Foram concluídos benchmarks de 1/2/4 ambientes, dois pilotos de 32.768 passos e 1.100 avaliações de episódios, comparadas em três conjuntos de 100 cenários. Os modelos antigos e os mapas usados pelo treino anterior não foram alterados. Nenhum treinamento longo foi iniciado.

## Resultados principais

- Dois ambientes com LiDAR nativo: 236 passos/s; um ambiente no benchmark Python: 127 passos/s. Ganho medido de cerca de 86%. Quatro ambientes nativos: 240 passos/s, próximo de dois.
- Pesos PPO finais idênticos bit a bit entre a implementação Python e a nativa nos benchmarks correspondentes.
- Piloto direto: 3/100 sucessos; piloto com waypoints: 1/100; checkpoint anterior: 1/100, na primeira bateria v2. Nenhum passou no gate de 90% de sucesso e zero colisões observadas.
- Planejador/controlador com pose verdadeira: 100/100; com pose estimada: 33/100. A pose verdadeira é um diagnóstico privilegiado, não uma solução pronta para o robô.
- Corrigir a pose da política antiga manteve 1/100. Localização e comportamento aprendido são problemas distintos.
- Com a cobertura mais ampla do novo amostrador por footprint, referência e piloto direto tiveram 0/100 sucessos. Não há candidato aprovado para treino longo/deploy.

## Componentes isolados

`envs.py`: otimização Python e A* no mapa conhecido. A ajuda por waypoints usa a **pose estimada**. O mapa estático conhecido deve existir no sistema real para que esse módulo seja utilizável.

`lidar_native.c`, `lidar_native.so`, `native.py`: raios amostrados em C, com mesmo passo, máscara, ruído, bordas e resultados. A compilação evita FMA e fast-math. Não aproxima a geometria para ganhar velocidade.

```bash
cc -O3 -shared -fPIC -ffp-contract=off \
  experimentos_v2_20261110/lidar_native.c -lm \
  -o experimentos_v2_20261110/lidar_native.so
```

`run.py`: benchmark, piloto curto e avaliação. Recusa diretórios de saída existentes. A execução usa CPU e uma thread Torch. O rollout global tem 512 transições, dividido pelo número de ambientes; preserva pesos/otimizador do checkpoint de origem, reiniciando apenas a contagem adicional e o fluxo aleatório. O checkpoint inicial está definido explicitamente no código e registrado com SHA-256 nos resultados.

`layout_service_candidate.yaml`: aproximações reavaliadas das 14 estações, sem modificar os objetos, as alturas ou a geometria de colisão. As cópias das camadas física/LiDAR em `review/` permitem carregar essa candidata separadamente com `Arena2016V2Env(layout=...)`. Não substitui o layout ativo. As alturas ainda são estimadas; acesso do manipulador não foi validado.

`full_sampler.py` e `sampler_states.npz`: amostrador experimental no maior componente do espaço de configuração, com 24 orientações e conexões conservadoras de rotação. As 1.000 consultas de colisão em 500 resets passaram. A alternativa alcança regiões excluídas pelo disco. **O A* antigo por disco é incompatível e é rejeitado**; falta planejamento equivalente com footprint antes de usar waypoints nessa distribuição. A interface deste piloto permite o novo amostrador apenas em avaliação.

## Reproduzir

Executar em `~/robo_rl_v2`:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -u experimentos_v2_20261110/orchestrate.py
```

O orquestrador cria uma nova pasta com timestamp, executa validação de equivalência, três benchmarks, dois pilotos e as avaliações iniciais. Os diagnósticos adicionais estão em `native_benchmark.py`, `diagnostics.py` e `evaluate_coverage.py`; recebem o caminho da pasta de resultados como argumento e recusam saídas já existentes. Para uma reprodução integral, é preciso ter gerado `sampler_states.npz` com `build_sampler.py`. Os scripts `validate.py`, `test_native.py`, `test_sampler.py`, `geometry_analysis.py` e `resolve_services.py` registram os contratos e as buscas geométricas.

Exemplo de um piloto curto usando a configuração mais eficiente (não inicia ao ler este README):

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -u experimentos_v2_20261110/run.py pilot \
  --native --envs 2 --steps 32768 --seed 42 \
  --output runs/piloto_v2_nativo_novo
```

Esse comando treina somente a etapa de metas até 2 m no amostrador antigo, sem avanço automático. Metas curtas em linha reta podem exigir percursos longos em torno de paredes. O próximo currículo deve controlar também comprimento geodésico/visibilidade.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m unittest discover -s tests -v
```

Os 33 testes do projeto passaram ao final desta sessão. As avaliações têm somente uma seed de treino por piloto; não há demonstração robusta de superioridade entre algoritmos. O ganho de velocidade foi medido em testes curtos, sujeitos a variação de CPU. A extração da imagem nativa do PDF e a calibração física permanecem pendentes.

Nenhum modelo experimental foi ativado automaticamente no visualizador ou no robô. Os resultados não sustentam prolongar os pilotos atuais.
