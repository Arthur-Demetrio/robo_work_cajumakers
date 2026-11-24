# Arena RoboCup@Work 2016 — reconstrução aproximada

**Reconstrução aproximada da arena RoboCup@Work 2016 baseada em referência visual e restrições geométricas conhecidas.** Não é uma réplica exata nem uma planta oficial.

A imagem fornecida foi preservada em `referencia_usuario.png`. A perspectiva não permite obter medidas, alturas ou uma homografia calibrada. O desenho é uma interpretação manual em planta, com escala global de **12 × 10 m**, sem inferir centímetros a partir de pixels da referência.

## Arquivos e hipóteses

- `layout.yaml`: fonte editável em metros, origem sudoeste, x para a direita e y para cima. Retângulos: `[cx, cy, largura, altura]`; círculo: `[cx, cy, raio]`.
- `generated/arena_2016_*.pgm` e respectivos `.yaml`: mapas trinary compatíveis com `Grid.load`, resolução **0,025 m**, dimensões **480 × 400 células**. A ocupação inclui qualquer célula tocada pelo objeto, conservadoramente.
- `generated/arena_2016_*.png`: vistas superiores com identificadores dos objetos.
- `generated/arena_2016_treino_exemplo.png`: exemplo de caixas sorteadas em tempo de execução.
- `generated/validation.json`: separação entre estruturas e validação das cavidades internas declaradas em `passage_areas`.
- `scenario_validation.json`: inspeção de 100 sementes por variante, além das verificações de API Gymnasium/SB3.
- Implementação independente: `robo_rl_v2/arena2016.py` e `robo_rl_v2/run_arena2016.py`.

O contorno externo é retangular, com borda bloqueada artificial de 0,10 m para impedir saída do campo. A foto mostra uma borda de piso: **não há evidência de uma parede física contínua no perímetro**. As divisórias têm espessura de 0,12 m. Foram representadas uma divisória longitudinal interrompida por cruzamento, ramificações em L, quatro nichos/estações em U, seis plataformas brancas, mesa vermelha e plataforma circular. A disposição preserva características visuais e topológicas gerais; posições e simetria foram regularizadas para manter passagens válidas.

As coberturas são modeladas como estações com base sólida e divisórias laterais. A mesa vermelha ocupa toda a projeção do tampo: não é permitido passar por baixo. Plataformas baixas e círculo são obstáculos sólidos, sem rampas. Alturas, pernas, manipulação, superfícies de trabalho e passagem sob coberturas não são simuladas. Nichos criam becos e corredores; a abertura central forma um cruzamento entre os dois lados.

## Três variantes

| Variante | Geometria fixa | Caixas móveis | Ruído |
|---|---|---|---|
| `arena_2016_aproximada` | Todas as estruturas e mobiliário | 0 | Desligado |
| `arena_2016_limpa` | Mesma ocupação estática | 0 | Desligado |
| `arena_2016_treino` | Mesma ocupação estática | 3 por padrão | Configurável |

A e B têm a mesma ocupação intencionalmente: todos os objetos da reconstrução fazem parte do cenário fixo e não há decoração aleatória a remover. Seus nomes distinguem uso visual e validação. O mapa raster da variante C também contém a **base fixa**; caixas são adicionadas pelo ambiente a cada reset, com semente reproduzível. `Grid.load` sozinho não aplica randomização: para isso use `Arena2016Env` ou os comandos abaixo.

A variante de treino sorteia tamanhos de caixas entre 0,25 e 0,45 m, deslocamentos de início/objetivo de até ±0,08 m por eixo e configura atrito, escorregamento, ruído físico, ruído LiDAR, dropout, viés de odometria e atraso. `--noise off` desliga as perturbações de dinâmica e sensor; `--boxes 0` remove somente as caixas. Limites de velocidade, colisões SAT e recompensa são herdados do simulador original. A correção de dinâmica sugerida pela auditoria permanece na candidata separada e não foi incorporada silenciosamente.

## Garantias e limites geométricos

A separação conservadora entre objetos estáticos distintos é pelo menos 0,80 m; o limite inferior mínimo medido é registrado em `validation.json`. O cálculo usa distâncias entre células ocupadas, subtraindo a diagonal de uma célula. Partes conectadas de uma estação pertencem ao mesmo objeto; suas cavidades e a abertura central são verificadas separadamente como retângulos livres com dimensões ≥0,80 m. Alterações no YAML exigem reconstruir e validar novamente.

Cada caixa é rejeitada se cortada pelo mapa ou sobreposta a ocupado/desconhecido. Sua área raster deve permanecer a pelo menos 0,80 m de todas as outras estruturas e caixas, com margem conservadora de discretização. A adição não pode dividir nem eliminar as regiões conservadoramente navegáveis existentes. O gerador tenta novamente; não reduz silenciosamente o número solicitado. Quantidades exageradas podem falhar explicitamente após esgotar tentativas.

O robô usa o footprint completo de 0,57 × 0,63 m. Início e destino são amostrados após adicionar caixas, no mesmo componente conectado com clearance >0,52 m; o jitter é revalidado. Isso garante uma rota para o disco conservador do simulador. **Uma passagem de 0,80 m é menor que o diâmetro desse disco (~1,04 m)**: embora o retângulo possa atravessá-la orientado, o amostrador pode excluir esse corredor e regiões adjacentes. O campo tem múltiplos componentes nessa representação conservadora; início e objetivo ficam no maior componente. Não se afirma que todas as cavidades sejam selecionadas como objetivos, nem que o retângulo percorra toda a arena em qualquer orientação.

Essa geometria é adequada para navegação RL aproximada. Não é validação de dimensões oficiais ou sim-to-real. A referência não estabelece as medidas locais; a largura de 0,80 m é uma restrição fornecida pelo usuário.

## Comandos

Executar de `~/robo_rl_v2`. Os diretórios de saída precisam ser **novos**; o comando recusa qualquer pasta já existente para evitar sobrescrita. Não há modificação no `runs/principal`, no currículo original ou no mapa SLAM.

Reconstruir mapas e imagens após editar o YAML (somente derivados desta arena):

```bash
.venv/bin/python -m robo_rl_v2.arena2016 --build
```

Avaliar isoladamente o checkpoint atual em 100 sementes na arena limpa:

```bash
.venv/bin/python -m robo_rl_v2.run_arena2016 evaluate \
  --variant arena_2016_limpa \
  --model runs/principal/checkpoints/ppo_000002000384.zip \
  --episodes 100 --seed 3000000000 \
  --output runs/eval_2016_limpa_100
```

Avaliar com caixas e ruído, em diretório separado:

```bash
.venv/bin/python -m robo_rl_v2.run_arena2016 evaluate \
  --variant arena_2016_treino \
  --model runs/principal/checkpoints/ppo_000002000384.zip \
  --episodes 100 --seed 3000000000 \
  --output runs/eval_2016_treino_100
```

Treinar cenário específico, em nova execução:

```bash
.venv/bin/python -m robo_rl_v2.run_arena2016 train \
  --variant arena_2016_limpa --steps 2000000 --seed 42 \
  --output runs/treino_2016_limpa
```

Treinar com progressão nos cinco níveis da nova arena:

```bash
.venv/bin/python -m robo_rl_v2.run_arena2016 train \
  --variant arena_2016_treino --progression --steps 2000000 --seed 42 \
  --output runs/treino_2016_curriculo
```

Progressão: nível 1, destinos até 3 m e sem caixas; nível 2, distâncias maiores sem caixas; nível 3, uma caixa; nível 4, duas caixas; nível 5, três caixas e ruído. As fronteiras são 15%, 35%, 55% e 80% da meta, como no baseline. Isso apenas disponibiliza o pipeline: **a auditoria recomenda adotar gates de desempenho antes de investir em um novo treino longo**, pois o baseline usa avanço fixo por passos.

O treino escreve Monitor CSV, configuração, cópia do layout, checkpoints periódicos e `final.zip`. Não retoma nem sobrescreve o treino principal. PPO completa rollouts e pode exceder a meta em até `n_steps−1` passos. Avaliação escreve CSV por episódio, resumo e trajetórias/imagens dos três primeiros episódios. Use as mesmas sementes para comparar checkpoints na mesma variante e configuração; variantes com randomizações diferentes não têm necessariamente os mesmos pontos inicial/final.

Validação reproduzível:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m unittest discover -s tests -v
OPENBLAS_NUM_THREADS=1 .venv/bin/python assets/arenas/2016/validate_scenarios.py
```

Um smoke test separado de 128 passos com progressão (`validation_train/`) e avaliação de três episódios limitados a 32 ações (`validation_eval/`) verifica integração e persistência, **não qualidade da política**. Não foi iniciado treino longo nesta arena.

## Continuar o aprendizado de um checkpoint anterior

Use `--init-model` no modo `train` para carregar pesos e estado do otimizador em uma execução nova. Os passos desta execução começam em zero, e `config.json` registra a origem, o SHA-256 e os passos anteriores. `--steps` define a quantidade **adicional** de passos. A progressão da nova arena começa no nível inicial; a experiência coletada no ambiente antigo não é reaproveitada como rollout.

```bash
.venv/bin/python -u -m robo_rl_v2.run_arena2016 train \
  --variant arena_2016_treino --progression \
  --init-model runs/principal/checkpoints/ppo_000002000384.zip \
  --steps 2000000 --seed 42 --output runs/treino_2016_transferido_novo
```

Para encerrar com salvamento, crie `stop.request` dentro da pasta da execução. O callback encerra no próximo passo e grava `final.zip` e `status.json`. SIGINT/SIGTERM também solicitam parada com salvamento. Execute em uma sessão de terminal persistente; o processo supervisor deve permanecer ativo.
