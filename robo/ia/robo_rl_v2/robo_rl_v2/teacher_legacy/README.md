# Candidata: árvore e alinhador como professores da política

A implementação existente pode fornecer coordenação e demonstrações. Copiar o XML da árvore para o PPO, sozinho, não transfere o comportamento dos servidores Nav2 para os pesos da rede.

## Fontes verificadas

`~/robo_jazzy_ws/src/robo_navigation/launch/navigation.launch.py` seleciona por padrão `navigate_to_pose_smooth.xml` e inicia `scripts/alinhador_passagem.py`. O XML instalado resolve para esse mesmo arquivo fonte. Argumentos/overrides de um processo ROS em execução não foram consultados; portanto trata-se da configuração padrão verificada, não de uma afirmação sobre todo processo ativo.

Há cópias congeladas do XML, launch, parâmetros e três módulos Python em `snapshot/`, com SHA-256 e caminhos em `provenance.json`. Os originais não foram editados.

A árvore seleciona planejador/controlador, mantém/recalcula e suaviza caminhos, segue o caminho e coordena recuperação com `destravar`, limpeza de mapas e `smart_spin`. Os servidores que executam essas ações são parte do Nav2. O XML está inventariado; **não foi implementado um interpretador equivalente do Nav2 no simulador 2D**.

O alinhador oferece classes geométricas sem ROS: `Travessia`, `TravessiaLateral`, `Encosto`, `GiroFrontal` e funções de medição, rumo e detecção de oscilações. O adaptador carrega o núcleo original congelado, alterando apenas a resolução dos imports para as cópias congeladas. Não chama `main`, não inicia ROS, não publica comandos e não toca no treinamento.

## Base executável preparada

```python
from robo_rl_v2.teacher_legacy.adapter import load_core
core = load_core()
professor_encosto = core.Encosto(log=lambda mensagem: None)
professor_travessia = core.Travessia(log=lambda mensagem: None)
# Alimentar os métodos de medição com sensores e pose estimada compatíveis.
# O método passo(...) devolve vx, vy, wz do controlador real.
```

`points_from_scan` converte ranges/ângulos válidos em pontos base_link, com extrínsecos explícitos. `DemonstrationWriter` grava observação, ação física do professor, fase, estado e contexto em JSONL, recusando sobrescrita. É uma preparação de coleta; **nenhum dataset de missões foi coletado e nenhuma rede foi treinada por imitação nesta etapa**.

`command_label` verifica se a ação pode ser representada pelo simulador atual, respeitando ré assimétrica e limite combinado xy. Um giro do professor a 0,30 rad/s excede o limite RL de 0,16 rad/s: a amostra é marcada como não representável, sem converter silenciosamente para uma ação saturada diferente. Limites físicos, dinâmica e objetivos precisam ser compatibilizados antes de usar essas amostras no treino.

## Integração recomendada

1. Manter a árvore coordenando navegação, passagem, encosto e recuperação, com o Collision Monitor na cadeia de comando.
2. Usar o alinhador real como professor e controlador de reserva nas tarefas locais. Coletar exemplos de sucesso e de interrupção/recuperação em simulação e depois conferir com registros reais.
3. Treinar políticas por habilidade, com fase explícita ou memória. A máquina de estados mantém medições anteriores e timers; a MLP atual, sem fase e com pouca história, não representa automaticamente essas decisões.
4. Iniciar por imitação supervisionada dos comandos válidos. Só depois testar ajuste por reforço ou correções limitadas sobre o controlador, mantendo avaliação independente.
5. Comparar nas mesmas sementes: sistema clássico, política imitadora e variante híbrida. Medir sucesso de navegação, travessia e encosto separadamente, além de colisões e intervenções do controlador de reserva.

## Pendências concretas

- O objetivo atual do RL aceita proximidade de 0,20 m, sem exigir yaw ou velocidade final. Encosto precisa de tarefa própria: face correta, folga alvo, erro angular/lateral e parada.
- O alinhador usa estado temporal e guards de scan ausente/desatualizado. Um dataset de pares instantâneos sem histórico/fase pode conter ações diferentes para observações parecidas.
- Os 72 raios e a máscara atuais podem fornecer menos informação que o scan usado no robô; a zona cega descrita no alinhador também precisa ser reproduzida/calibrada.
- Com as alturas ainda estimadas da v2, plataformas baixas podem ser fisicamente sólidas e invisíveis ao scan. Não se deve dar a pose verdadeira de bancada ao professor e apresentar isso como medição LiDAR.
- O próprio código registra velocidades laterais físicas diferentes dos comandos enviados. Reproduzir pulsos e tempos exige calibração da dinâmica, não apenas copiar constantes.
- A ligação completa dos servidores Nav2 ao simulador e a coleta de demonstrações ainda não estão implementadas. O treino ativo permanece PPO com as configurações anteriores a esta candidata.

Cinco testes passaram: carregamento sem ROS, medição real de corredor, parada do Encosto sem medição atual, limites de ações e gravação sem sobrescrita. Testar com:

```bash
cd ~/robo_rl_v2
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m unittest discover -s tests -p test_teacher_legacy.py -v
```

## Clonagem supervisionada (candidata isolada)

O treinador em `experimentos_v2_20261110/imitation/train_bc.py` consome as demonstrações deste adaptador, filtra comandos fora da faixa representável e separa validação por `context.episode_id`. A rede recebe observação e fase explícita (`navigate`, `prepare`, `traverse`, `dock` ou `recover`); `bc_policy.py` fornece a inferência correspondente. É necessário que o supervisor externo forneça a fase correta no runtime.

Nenhum checkpoint BC foi criado: o workspace não contém demonstrações com rótulos do alinhador/árvore. Não usar a saída da própria PPO como professor, pois isso só destilaria as falhas existentes. O simulador ainda não executa a árvore Nav2 nem conecta as máquinas de estado do alinhador aos seus sensores e dinâmica; portanto a coleta/treino só deve começar depois de ligar um professor válido e registrar episódios independentes.

## Estado posterior da integração

Foi ligado um coletor Gym isolado para o núcleo real `Encosto`/`RastreioFaceWS`, e criado um candidato BC por fase/estado. A coleta alcançou 48 sucessos em 120 tentativas; avaliações independentes dos atores não aprovaram nenhum (o BC sem FSM colidiu em 100/100; o híbrido condicionado à FSM teve 0/100 sucesso sem colisões). Os pesos e dados ficam em `runs/dock_teacher_dataset_20261110_144200`, `runs/candidato_bc_encosto_20261110_144500`, `runs/candidato_bc_encosto_state_20261110_150000` e as avaliações relacionadas. Não ativar os atores. O Behavior Tree Nav2 completo ainda não executa no Gym; veja `experimentos_v2_20261110/imitation/RESULTADOS.md`.

Na reavaliação correta a 20 Hz (1.300 passos, 100 seeds), o alinhador original concluiu 61/100 docking e o híbrido BC 0/100, sem colisões. Por isso o alinhador continua sendo o controlador recomendado; não ative os checkpoints BC. A causa observada foi informação de controle ausente na entrada da rede (alvos internos e tempo no estado). Relatório atualizado: `experimentos_v2_20261110/imitation/RESULTADOS.md`.
