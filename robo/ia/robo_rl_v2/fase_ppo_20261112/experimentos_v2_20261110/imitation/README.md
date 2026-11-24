# Clonagem do alinhador: candidata isolada

O coletor `collect_dock_teacher.py` executa uma tarefa local de docking na arena v2 com uma parede virtual. Ele chama o núcleo real congelado `Encosto` e `RastreioFaceWS`, converte os 72 raios simulados em pontos no frame do robô, aplica colisão com footprint completo e grava JSONL com estado, fase, episódio e comando professor. Não inicia ROS nem envia comandos ao robô.

```bash
cd ~/robo_rl_v2
.venv/bin/python -u experimentos_v2_20261110/imitation/collect_dock_teacher.py \
  --episodes 120 --seed 71000 --output-dir runs/minha_coleta_dock
```

O modo simples `train_bc.py` treina uma MLP condicionada à fase. `train_state_bc.py` treina adicionalmente condicionada ao estado da FSM do `Encosto`, e `evaluate_state_bc.py` compara alinhador e ator híbrido nas mesmas seeds. Cada comando exige diretório de saída novo.

Não usar como treinamento longo/deploy. Na coleta final a 20 Hz, o professor concluiu 73/120 episódios. Na avaliação corrigida de 100 seeds, o professor concluiu 61/100 e o BC condicionado à FSM 0/100; ambos tiveram zero colisões. O BC simples teve 100 colisões em sua avaliação inicial. Nenhum candidato foi ativado. A análise completa está em `RESULTADOS.md`.

O mapeamento de atuadores local foi baseado em medições/notas já existentes; continua aproximado e requer calibração. O teste cobre somente encosto numa parede virtual, não navegação na arena nem o Behavior Tree Nav2 inteiro. Os servidores Nav2 não estão conectados ao Gym.
