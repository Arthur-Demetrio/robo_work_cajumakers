# Refinamento da navegação: relatório de 11/11/2026

**Bags analisados**, ambos em `~/bags/`, com SHA256 conferido:
- principal: `2nav_integrada_corredor_aberta_20261110_123314_041073`;
- comparativo: `nav_integrada_corredor_aberta_20261110_114847_814937`.

**Baseline.** O workspace de hoje é idêntico ao snapshot de fontes do bag 2nav. Conferi o alinhador, o destravar, a recuperação, o filtro, o suavizador, o `nav2_params`, a árvore, o launch e o executor. O `install/` usa symlinks para os arquivos do src.

**Nada foi movido.** O robô não andou, nenhum goal foi enviado e o Nav2 do robô não foi reiniciado (ele nem estava rodando). As conferências com ROS rodaram no domínio 166, só em localhost: um `bt_navigator` sozinho e servidores falsos que rejeitam qualquer goal.

## 1. Linha do tempo (2nav, 12:33–12:43)

Arquivos: `LINHA_DO_TEMPO.csv` (284 eventos, cada um com a prova: nó, arquivo e linha do log ou tópico) e `LINHA_DO_TEMPO_comparativo_114847.csv`.

| Goal | Hora | Destino | O que aconteceu | Classe |
|---|---|---|---|---|
| G1 | 12:33:31 | (1,57; 4,49) | ROTA_MAPA trocou o planner (10,5 m) pela rota do mapa (6,1 m). Travessia ok; saída lateral para a esquerda no T falhou: o CM bloqueou 4 vezes com folga frontal de 4 cm. Pediu ajuda às 12:34:36 | falha autônoma |
| G2 | 12:34:56 | (1,49; 1,95) | Pré-giro +76° (andou 20 cm de lado antes). Smoother reprovou 7 vezes e o DWB ficou sem trajetória válida. O alinhador assumiu o corredor; a 1ª travessia abortou (CM 5 vezes) e a 2ª alinhou | **preempção do operador** às 12:35:57 |
| G3 | 12:35:57 | (1,69; 4,71) | Chegou a 7 cm; o DWB girava para o yaw 5° | **preempção** às 12:36:27 (estava quase no fim) |
| G4 | 12:36:27 | (1,67; 1,39) | Pré-giro −115°. ANTI_CISCADO, destravar sem alternativa, novo pré-giro +52°. Chegou a 4 cm às 12:37:22; o DWB girou +57° e +151°, passou do yaw pedido e não encerrou | **preempção** às 12:37:41 (robô preso no giro: defeito autônomo) |
| G5 | 12:37:41 | (1,68; 4,90) | Pré-giro −61°. Sem trajetória válida, sem progresso, destravar sem alternativa: **Goal failed** às 12:38:06 | falha autônoma |
| G6 | 12:38:17 | (1,32; 4,59) | Pré-giro +48° (dois recuos curtos). Chegou; o DWB girava para o yaw 5° | **preempção** às 12:38:44 |
| G7 | 12:38:44 | (−0,30; −0,12) | PREPARO com pré-giro −106°. Planner sem caminho; destravar lateral (ICP 27 cm de lado e **16 cm para trás**). PREPARO ok às 12:39:43. Duas travessias abortadas pelo CM dentro da passagem, com ré de 1,3 m. Pediu ajuda às 12:40:53 | falha autônoma |
| G8 | 12:41:07 | (0,35; 0,06) | Travessia ok (FREIO_CRUZAMENTO). Saída lateral para a direita em 61 s, com 10 paradas de segurança e **folga traseira de 3–4 cm**. TRAJETO CURTO +92° para o yaw pedido | concluído |

As 4 trocas de goal (G2→G3, G3→G4, G4→G5, G6→G7) foram **intencionais** e não contam como falha. Nenhuma navegação do Nav2 terminou em sucesso neste bag: 6 cancelamentos, todos por preempção ou pelo alinhador, e 1 aborto (G5). Não houve nenhum "Reached the goal!". No bag das 11:48 foram 5.

## 2. Origem dos giros longos

Arquivos: `TABELA_GIROS.csv` (≥ 45°) e `MAPA_GIROS_2nav.png`. A origem de cada giro vem do comando: `/cmd_vel_ctrl` é do DWB; alinhador e destravar publicam direto no `/cmd_vel_nav`. O ângulo vem da IMU.

| Origem | Giros | Tempo | Ângulo |
|---|---|---|---|
| Pré-giro do alinhador (VIRAR_CAMINHO, limiar 45°) | 6 | 29,1 s | 456° (74, −115, 52, −61, 48, −106) |
| DWB, **todos no goal**, girando para o yaw 5° | 4 | 28,4 s | 331° (−69, +57, +151, −54) |
| TRAJETO CURTO (yaw final) | 1 | 4,8 s | +92° |

Os números da análise anterior (+76, −119, +56, −65, +51, −110; +96 às 12:42:35) **conferem**. As diferenças de 2–4° são o ângulo pedido contra o medido pela IMU.

O que os explica:
- **Pré-giros.** O Nav2 não dá ré (DWB_SEM_RE). Com o caminho a 115° da frente, o giro é necessário. Os de 48–61° são candidatos ao teste `virar_caminho_graus` 90. Eles usam o caminho do **planner** (`_caminho`, costmap atual), não rota estática.
- **DWB no goal.** O yaw 5° vem dos goals manuais. Às 12:37:22–41, o `/evaluation` mostra **1 trajetória válida de 392** (girar a +0,16). `min_speed_theta` 0,15 com `decel_lim_theta` −0,2 a 10 Hz deixa a janela em [0,14; 0,16]: o DWB não consegue frear nem inverter. Passou **126° além** do yaw pedido.
- **O goal checker não encerrou.** No frame do controller (odom), às 12:37:29,9 o robô estava a 4,8 cm e 0° do fim do caminho, dentro de 0,06 m e 0,17 rad. Ainda assim, nenhum "Reached the goal!".
  - **[Atualizado em 11/11, ver `giro_travado/GIRO_TRAVADO.md`: confirmado no fonte e reproduzido com o código real do Nav2. O mecanismo exato é o XY nunca registrado após o reset de 12:37:26,58.]** Leitura inicial: às 12:37:27,4 o SLAM corrigiu o `map→odom` em cerca de 15 cm e 9°. Convertendo o goal com o `map→odom` antigo, o alvo em odom fica em (1,01; 0,47; 16,5°), exatamente o `/transformed_global_plan` de 12:37:24–26. Com o novo, fica em (1,18; 0,48; 8°). Contra o alvo antigo, o robô estava a 12 cm, fora da tolerância.
  - O goal checker transforma o fim do caminho no carimbo da pose (`controller_server::isGoalReached` → `nav_2d_utils::transformPose`); o DWB usa carimbo zero. **Confirmado no fonte 1.3.12.** Correção mínima preparada e não aplicada: `GOAL_TF_ATUAL`.
- **Ação:** Patch 02 (o alinhador faz o yaw final, desligado por padrão). Vale também para os giros do DWB das 12:36:21 e 12:38:40.

## 3. Ré no deslize lateral

Arquivos: `TABELA_LATERAL.csv` e `FICHA_CALIBRACAO_LATERAL.md`.

Há dois mecanismos diferentes:
1. **Comandado (alinhador, esquerda).** `vx = −k_ff·vy − 0,03` (folga frontal < 9 cm) = −0,046. É exatamente o mínimo citado. Pelo SLAM o robô andou +1,4 cm: o comando cancelou a deriva para a frente. A odom diz −22 cm, então **não pode** ser usada como prova.
2. **Mecânico (destravar e ajustes curtos com vx = 0; alinhador indo para a direita).** Indo para a **direita** o robô **recua**:
   - 12:39 (destravar): SLAM −12 cm, ICP −16 cm;
   - 12:40:18: −16 cm em 11 cm de lado;
   - 12:42:16, mesmo com ff +0,016: −29 cm em 39 cm;
   - no bag das 11:48, o destravar recuou −18 e −21 cm.

   Indo para a esquerda, avança. Isso aparece também em comandos abaixo do mínimo de lado do motor (0,025–0,03 contra ~0,06). Num piso sem frisos, isso é compatível com o que você viu.

O destravar não tem feedforward. **Não mexi em compensação:** a skill exige trena primeiro, e a ficha traz o protocolo.

## 4. Fallback do SmoothPath (Patch 01)

- 67% das suavizações do 2nav falharam: 36 de 54 (16 de 23 no bag das 11:48).
- Meu modelo offline do checker reproduz **54/54 e 23/23** veredictos do Nav2.
- O `IsPathValid` do Jazzy reprova os 80 caminhos originais, porque usa o yaw 0 do NavFn e reprova custo inscrito. Por isso ele não serve de portão.
- Um portão estrito no footprint reprovaria 30 de 37, sendo 23 raspões de 2–8 cm.

O portão implementado reprovaria **11 de 38** fallbacks. Todos caem em momentos em que o robô de fato não avançou: boca do T às 12:35, falha às 12:37:49. Detalhes, A/B e rollback estão em `PATCHES.md`. Instalado em modo `registrar`.

## 5. Rota do mapa contra obstáculo (Patch 04) e GoalAlign (Patch 05)

- **Rota do mapa.** Às 12:33:39 a troca foi reproduzida. As células "do momento" junto da rota estão na parede norte do corredor, que o robô atravessou. Uma regra ativa recusaria uma rota boa. Por isso entrou só `registrar`/`recusar`, com padrão `ignorar`.
- **GoalAlign.** O perfil isolado só muda `GoalAlign.scale`. No 2nav o controller manteve 10 Hz (p95 106 ms), então a latência não é limite hoje.

## 6. Testes offline

| Conjunto | Resultado |
|---|---|
| `robo_bt_plugins`: gtest do núcleo (13 casos) e do nó numa árvore BT.CPP real (7 casos) | **22/22** passaram (`testes/gtest_robo_bt_plugins_final.txt`) |
| Suíte `robo_navigation/test` inteira (com 6 testes novos da árvore e 8 dos patches 02/04/05), sondas com plugins reais do Nav2 | **260/260** passaram (`testes/pytest_robo_navigation_final.log`) |
| `bt_navigator` real do Jazzy 1.3.12 carregando a árvore modificada e o plugin (configure e activate) | **ativo** (`testes/bt_navigator_isolado_resultado.txt`) |
| Patch scripts: aplicar, recusar a 2ª vez e `--desfazer` numa cópia (Patches 02 e 04 nas duas ordens) | ok |
| Replay do núcleo C++ nos costmaps gravados (`dados/casos_*`) | 2nav: 11 de 38 reprovados; 11:48: 1 de 17 |

**Erros meus, achados e corrigidos durante o trabalho:**
- comentário XML com `--` (o BT.CPP aceitou, o parser estrito não);
- backup com `package.xml` sob `entregas/`, que fazia o colcon ver um pacote duplicado (resolvido com `entregas/nav_refinamento_20261111/COLCON_IGNORE`);
- troca de modo por `ros2 param set` no nó cliente do BT, que trava (passou a ser por tópico);
- processos `bt_navigator` de teste que sobraram no domínio 166 (encerrados).

## 7. Estado de cada patch

| Patch | Integrado | Compilado/instalado | Ativo | Validado fisicamente |
|---|---|---|---|---|
| 01 CAMINHO_ORIGINAL (modo registrar) | sim | sim | **não** (falta reiniciar o Nav2) | não |
| 02 GIRO_FINAL_COMUM (desligado) | sim | não se aplica (script) | não | não |
| 03 ferramentas e ficha lateral | sim | não se aplica | não se aplica | não |
| 04 ROTA_MAPA_OBSTACULO (ignorar) | sim | não se aplica | não | não |
| 05 perfil GOALALIGN0 | sim | sim (share) | não | não |

## 8. Próximos testes (um bag por linha, nesta ordem)

1. **Baseline com o pacote novo.** Patch 01 em `registrar`, todo o resto no padrão. Prova que nada mudou e junta os casos de "REPROVARIA".
2. `giro_final_comum true`, mesmo roteiro de goals.
3. `virar_caminho_graus 90`.
4. Patch 01 em `aplicar`.
5. Perfil GOALALIGN0, com a caixa no laço ideal.
6. Calibração lateral pela ficha (só teleop e trena).
7. `rota_mapa_obstaculo registrar`.

## 9. Pendências

- Feito: confirmado no fonte e reproduzido offline (`giro_travado/GIRO_TRAVADO.md`). A correção `GOAL_TF_ATUAL` está preparada e não aplicada; o teste físico supervisionado está descrito lá.
- A trava de giro do DWB (`min_speed_theta` contra `decel_lim_theta`; só no sentido anti-horário, ver GIRO_TRAVADO.md) continua na configuração. Mudar isso fere o DWB_GIRO_REAL e precisa de um teste próprio.
- `k_ff` por lado e feedforward no destravar dependem da ficha de calibração.
- Atualizar o `CLAUDE.md` com as marcas novas. Está proposto na resposta, não aplicado.

## Rollback geral

```bash
cd ~/robo_jazzy_ws/ferramentas/patches
python3 aplicar_rota_mapa_obstaculo.py --desfazer
python3 aplicar_giro_final_comum.py --desfazer
python3 aplicar_caminho_original.py --desfazer
python3 aplicar_perfil_goalalign0.py --desfazer
cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash
```

A cópia exata de antes está em `backups/baseline_20261111_084923/`. Os testes alterados têm `.bak_caminho_original_*` ao lado.
