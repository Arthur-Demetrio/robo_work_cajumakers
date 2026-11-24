# Bag `nav_integrada_corredor_aberta_20261110_114847` — análise e correções

Bag de 15 min, 11:48–12:04. NAV_INTEGRADA sem ajuste: corredor, depois área aberta.

## Resultado medido (pose `map→odom→base_link` no fim de cada goal do Nav2)

| Início s | Goal | Resultado do Nav2 | Robô a | Real |
|---:|---|---|---:|---|
| 468 | (1,58; 4,12) | succeeded | 0,06 m | **chegou** (64 s para 2,2 m) |
| 547 | (2,20; 1,54) | succeeded | 2,70 m | **sucesso falso** |
| 574 | (2,22; 1,54) | succeeded | 2,71 m | **sucesso falso** |
| 587 | (1,71; 2,15) | succeeded | 2,03 m | **sucesso falso** |
| 606 | (1,65; 1,60) | failed | 2,58 m | sem caminho |
| 646 | (2,25; 1,47) | failed | 2,80 m | sem caminho |
| 686 | (2,23; 1,30) | succeeded | 1,66 m | **sucesso falso** |
| 879 | (0,05; 0,04) | failed | 0,44 m | sem caminho |

O Nav2 chegou de verdade em 1 de 11 goals. 4 dos 5 "succeeded" foram falsos, e o alinhador seguiu a missão acreditando neles.

## Causas

1. **Sucesso falso (erro da integração).** O blackboard do `bt_navigator` sobrevive entre goals. A sequência no `/behavior_tree_log` (548,5 s):
   - o replanejamento falha (suavização colide e o `IsPathValid` do caminho original também falha);
   - o ramo "manter caminho" aceita o `smoothed_path` do goal anterior;
   - o `FollowPath` "chega" ao fim desse caminho, onde o robô já estava.

   A árvore anterior tem o mesmo furo (reproduzido no teste), mas como ela nunca falhava na suavização, quase nunca o expunha.
2. **Suavização com checagem de colisão + `IsPathValid` do original (erro da integração).** Das 16 colisões da suavização, 5 foram a menos de 15 cm do goal: destinos perto de mesa ou parede ficam na zona inflada. Com o replanejamento a 2 Hz, cada reprovação parava o robô no meio do trajeto e chamava o destravar.
3. **Sem caminho, o destravar v4 não tinha direção e desistia** (o v3 se afastava).
4. **Localização (já existia, é a raiz).** O `map→odom` deu 327 saltos acima de 5 cm durante 305 s de movimento (1,07/s; a missao32 teve 1,28/s), com picos de 73 cm e 13°. O obstacle layer do global (alcance 3,5 m) carimba paredes fantasmas: uma faixa em y≈2,8 fechou a passagem. Em 9 de 12 falhas do planner o costmap global estava sem ligação entre o robô e o goal, enquanto o mapa estático ligava.
5. **Já existia:** a saída lateral do alinhador foi abortada pelo CM 4× com a frente a 7–12 cm (11:54:38).
6. **Hardware:** IMU parada por 46 s antes do primeiro movimento.

## Correções aplicadas (offline; nada rodou no robô)

| Marca | Onde | O quê |
|---|---|---|
| CAMINHO_DO_GOAL | árvore | flag `caminho_do_goal`: zera no início de cada goal e em cada replanejamento, volta a true só com caminho novo do goal atual; o ramo "manter caminho" a exige. Provada no BT.CPP real (`sonda`: `caminho_do_goal=OK`) |
| ORIGINAL_SEM_GATE | árvore | suavizado que colide → caminho do planner sem o `IsPathValid` (critério do raio inscrito, calculado agora no costmap atual) |
| SEM_CAMINHO | `recuperacao_avancada.py` | sem caminho: limpa os costmaps, espera 2 s parado, replaneja; até 2× |
| AFASTAR | `recuperacao_avancada.py` | sem caminho e corpo a menos de 8 cm: 6–10 cm para o lado que mais abre folga (volume varrido, cobertura, CM, confirmação pelo LiDAR) |

Backups: `navigate_to_pose_smooth.xml.bak_caminhogoal_*`, `recuperacao_avancada.py.bak_afastar_*`.
Testes: **245 aprovados, 0 falhas, 0 pulados**. Novos: vários goals na mesma árvore (sem sucesso falso; a anterior tinha o furo), suavização reprovada seguindo o original, afastar e espera sem caminho.

## Não corrigido (cada um num teste separado)

- **Global obstacle layer 3,5 → ~1,5–2,0 m:** com 5–13° de salto, um ponto a 3,5 m é carimbado 30–80 cm fora do lugar; a 1,5 m, 13–34 cm. Reduz as paredes fantasmas sem perder o desvio, que é decidido perto.
- **SLAM:** a raiz. Antes de cada teste, verificar os saltos com o robô andando.
- **Controle:** GoalAlign 0 + horizonte 1,6 (`early_test`) para contornar a caixa.
