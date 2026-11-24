# missao33 (11/11, 10:29–10:39): análise

Bag: `~/bags/missao33_20261111_102905_949360`.
Dados extraídos: `dados/missao33/`. Log legível: `missao33/rosout.txt`.

O stack desta missão rodava a árvore anterior. A sessão foi aberta às 09:33 e o `GOAL_TF_ATUAL` foi aplicado depois, sem reinício. O mapa `arena_04-11_1235` foi carregado às 10:24 com `match_type 2`, e o slam_toolbox estava em modo **mapping**.

## 1. Deslocamento lateral: há comando para frente?

**Não.** Na saída lateral para a esquerda (CURVA 1, 10:30:35–10:32:13), em todos os estágios da cadeia:

| Tópico | Amostras (vy > 0,04) | vx médio | vx mín / máx | vx < 0 | vx > 0 |
|---|---|---|---|---|---|
| `/cmd_vel_nav` (alinhador) | 488 | −0,032 | −0,046 / −0,016 | 100% | 0% |
| `/cmd_vel_smoothed` | 485 | −0,032 | −0,046 / 0,000 | 97% | 0% |
| `/cmd_vel` (depois do CM) | 408 | −0,031 | −0,046 / 0,000 | 97% | 0% |
| `/cmd_vel_hardware` | 412 | −0,031 | −0,046 / 0,000 | 97% | 0% |

O comando é **para trás**:
- feedforward −0,2·vy = −0,016;
- mais o afastamento da parede, −0,03, quando a folga frontal está abaixo de 9 cm.

A ponte converte cada roda por PWM = 70 + (|v| / 0,31)·145. Esse offset de 70 **atenua** o vx, que chega à cinemática como ~−0,02 m/s em vez de −0,03, mas **não inverte o sentido**.

**O robô vai para frente fisicamente.** Pelo SLAM, no 1º deslize foram +9,8 cm para frente em 42,8 cm de lado. Nos 98 s da CURVA 1, foram +17 cm para frente e 92,6 cm de lado. O ciclo se repetiu 17 vezes:
1. desliza;
2. a folga frontal cai de 8 para 3–4 cm;
3. PARADA DE SEGURANÇA;
4. recua 4–8 cm;
5. desliza de novo.

Até o tempo máximo estourar: "Saida lateral abortada: tempo maximo".

**Conclusão.** A frente vem do **escorregamento mecânico das rodas mecanum** indo para a esquerda (+0,05 a +0,23 cm de frente por cm de lado), que supera a compensação comandada. É o mesmo padrão das missões 2nav e 11:48: indo para a esquerda, avança; indo para a direita, recua. **Não é** um comando de software.

**Correção:** precisa de medição com trena (`FICHA_CALIBRACAO_LATERAL.md`). Pelos números deste bag, o feedforward para a esquerda teria de ser bem maior que 0,2. **Hipótese** a calibrar: 0,4–0,6, considerando também a atenuação da ponte. Não alterei.

## 2. Perda de orientação (fisicamente para um lado, no RViz para outro)

Comparei o yaw no mapa (SLAM) com o yaw da IMU, segundo a segundo. A diferença fica estável (−3°) até 10:30.

- **Entre 10:36:09 e 10:36:22** (goal (0,83; 4,64), DWB "No valid trajectories", robô girando na região norte):
  - a IMU registra **+59°** de giro (de 125° para −176°);
  - o SLAM registra **+8°**;
  - a localização fica cerca de **55° errada**, chegando a −74° no fim.
- **Daí em diante, todo planejamento falha:** "failed to plan from (0.42, 4.96)", 45 vezes. O robô se via numa pose sem caminho; o destravar e o alinhador ficaram sem saída.
- **Antes disso, na CURVA 1** (10:30:33–10:32:07), o yaw do SLAM já oscilava ±10–20° com o robô **parado em yaw**: a IMU estável e o alinhador medindo "girar +0,0 graus".

**Conclusão.** O slam_toolbox em mapping, com os parâmetros de casamento padrão (`angle_variance_penalty` 1,0, `minimum_angle_penalty` 0,9, nó novo a cada 3 cm / 2°), aceitou casamentos errados durante o giro. A IMU, que alimenta o yaw da odometria, estava certa: era o que você via fisicamente.

**Não alterei o SLAM.** O próximo passo, separado, é um replay offline deste bag pelo slam_toolbox (domínio isolado), comparando os parâmetros atuais com **um** ajuste por vez (por exemplo, `angle_variance_penalty` maior). O critério é a diferença SLAM − IMU ficar abaixo de 5° no trecho 10:36:09–22. Só depois disso vale um bag físico.

## 3. Recuperação: insistir antes de dar FAILED

Nos travamentos de **10:33:10** e **10:34:11**, o destravar respondeu "sem alternativa segura" uma única vez. Quando o ramo de recuperação falha, o `RecoveryNode` da árvore desiste sem usar as 6 tentativas. Resultado: Goal failed, e o alinhador parou ("ajude o robô e mande o goal de novo").

**O que funcionou na prática:**
- 10:33:14: você mandou um goal novo;
- o alinhador fez o pré-giro de 120°, com 6 cm de ré e 3 cm de lado para a quina passar;
- **10:33:43: Goal succeeded.**

**Implementado: `REINSISTIR`** (`ferramentas/patches/aplicar_reinsistir.py`, marca `REINSISTIR (11/11)`).
- Quando o Nav2 desiste do GOAL FINAL, o alinhador faz sozinho o que você fez: reanalisa o caminho, vira para ele (VIRAR_CAMINHO) e manda o goal de novo.
- São até `reinsistir_goal_final` = **2** tentativas por goal; com 0, o comportamento é o de antes.
- Um goal novo do operador continua interrompendo na hora.
- Nada no Nav2 ou no destravar muda.

**Estado:** aplicado no `src`, com testes offline (12 no arquivo de refinamento). **Não está ativo**: o alinhador em execução foi iniciado às 09:33, e só um reinício do stack ativa o patch. Também não foi testado fisicamente.

**Desligar sem reiniciar** (depois de ativo): `ros2 param set /alinhador_passagem reinsistir_goal_final 0`.

**Desfazer:** `python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_reinsistir.py --desfazer`. É uma troca reversa, independente dos outros patches.

**Limite.** Se a falha vier de localização errada, como em 10:36 em diante, reinsistir não resolve: o planner continua sem caminho. Ele só repete até 2 vezes e então dá FALHOU, como antes.
