# Resultados de 26/10/2026

## 1. Prova1 (09:10–09:17): tarefa BTT1 completa, 11 de 11 etapas

Executor com `tarefa_exemplo_btt.yaml` (4 objetos, WS_1–3, pegar/soltar simulados com 5 s).
A tarefa terminou às 09:14:42, 4 min 25 s depois do início (limite da prova: 10 min).

| Etapa | Distância da mesa |
|---|---|
| WS_1 | 8,7 cm |
| WS_3 | 7,1 cm |
| WS_2 | 8,9 cm |
| WS_1 | 8,2 cm |

Passagens pelo LiDAR: 2 de 2. Log da missão: `prova1_log_missao.txt`.

### Travamento na ida ao FINISH (09:14:51–09:17:15)

O robô chegou a **3 cm** do FINISH, mas virado para ~127°. O goal pedia 20° com tolerância
de 2°. O DWB, girando no canto, encostou nos obstáculos (0–1 cm à direita e na frente) e o
Collision Monitor deu STOP. As recuperações recusaram, e com razão:

- **backup:** "Collision Ahead";
- **mecanum_escape:** precisava de 37,5 cm livres de um lado, havia 31,7 cm à esquerda e 24,2 cm à direita;
- **smart_spin:** o desenho do robô no mapa já estava em cima de obstáculo.

O tempo ia quase todo na espera: 12 s sem progresso, limpar o mapa, mais 12 s, e só então
uma ação (que falhava em 0,1 s), 6 vezes. Ajuda manual às 09:17:15. Detalhes:
`prova1_travamento_finish.txt`.

**Correções:**
- v3.10: ponto sem mesa recebe o ângulo de chegada e é aceito a ≤10 cm por 1,5 s;
- recuperação rápida: 6 s sem andar 3 cm nem girar 10°, 4 tentativas, sem o Wait de 5 s.
  A 1ª ação passa a sair em ~12 s (era ~24 s); desiste em ~1 min (era ~2,5 min).

### Onde foram os 264 s

| Parte | Tempo |
|---|---|
| Navegação do Nav2 | 122 s (46%), sendo ~52 s girando parado |
| Encosto nas mesas | 73 s (28%) |
| Passagens pelo LiDAR | 32 s (12%) |
| Pegar/soltar simulado | 20 s (8%) |
| Sair da mesa e outros | ~17 s (6%) |

Por trecho: `prova1_trechos_nav2.txt`. A convergência final em cada goal leva menos de
1 s (`prova1_fim_de_trecho.txt`), então a tolerância de 2° não custa tempo em lugar aberto.

## 2. Velocidade real x comando

### Medido no bag da prova1 (`prova1_velocidade_real.txt`)

| Comando linear | Real (mapa) |
|---|---|
| 0,03 m/s | 0,12 m/s |
| 0,04 m/s | 0,21 m/s |
| 0,08 m/s | 0,17 m/s |

Giro parado: 14–18 °/s para qualquer comando de 0,03 a 0,14 rad/s.

### Teste de giro (`teste_velocidade_giro_26-10.txt`)

| Comando | Real |
|---|---|
| 0,10 rad/s | 16–17 °/s |
| 0,14 rad/s | 17–18 °/s |
| 0,20 rad/s | 19–20 °/s |
| 0,28 rad/s | 22 °/s |
| 0,40 rad/s | 25 °/s |
| 0,55 rad/s | 29 °/s |

### Por quê: a conversão comando → PWM (bridge `robo_hardware` + `.ino`)

- **Bridge:** `PWM = mínimo + (velocidade da roda / 0,45) × (215 − mínimo)`, com mínimo 150
  para andar e 80 para girar parado.
- **Arduino:** só corta em `MOTOR_MAX_PWM = 215` e para os motores após 300 ms sem comando.
- **Nos máximos atuais do Nav2:** andar 0,08 m/s dá **PWM ~162**; girar 0,14 rad/s dá **PWM ~100**.

Como o PWM mínimo já é alto, qualquer comando sai perto da mesma velocidade. Há folga até
215 sem mexer no Arduino.

- **Giro:** a folga é grande e a odometria não sofre (o ângulo vem da IMU).
- **Reta:** no máximo +35% (PWM 162 → 215). Exige recalibrar a odometria, que hoje é o
  comando × 5; não foi feito.

### Zonas de velocidade (rápido em reta longa e livre)

Simulado sobre a prova1 com o nó `zonas_velocidade.py` (`zonas_velocidade_simulacao_prova1.txt`):

| Critério | Tempo em modo rápido | Ganho estimado |
|---|---|---|
| Padrão | 5 s | ~2 s |
| Médio | 14 s | ~4 s |
| Solto | 34 s | ~9 s |

A arena é pequena: quase todo trecho está perto de um fim, de uma curva ou de um giro.
**Não aplicado.**

## 3. Giro rápido e a prova2 (10:57–11:03)

Aplicado: shim a 0,40 rad/s (antes 0,10), smoother com giro até 0,40, Collision Monitor com
polígonos `giro_esq`/`giro_dir` (7 cm de margem) e giro até ±0,60 nos polígonos de andar.
Antes, giro acima de 0,20 rad/s não tinha polígono nenhum.

**Resultado:** vai e volta por 45 s na pose de preparo (`prova2_vai_e_volta_comandos.txt`,
`prova2_log_nav2.txt`).

1. 11:00:28–11:00:32: o shim gira a −0,40 de 13° até −78° e o robô vai até o preparo (0,22; −0,21).
2. Daí em diante: o DWB gira a +0,14 para o ângulo do goal. O BT manda caminho novo 1×/s e,
   com `rotate_to_heading_once: false`, o shim volta a valer a cada caminho. Perto do goal o
   "rumo" que ele mede aponta para trás, e ele gira a −0,40 até −174°.
3. Ciclo de ~6 s, repetido até 11:01:13. O PoseProgressChecker conta o giro como progresso,
   por isso não cortou antes.
4. Ao parar, o 0,40 levava 0,8 s até zero no smoother (freio −0,5), uns 10° a mais.

**Correção a testar:** `corrigir_giro_shim.py` (`rotate_to_heading_once: true` e freio −1,2).

Os logs do alinhador e do executor não estão no bag da prova2 (só os do Nav2).

## 4. Regras (pesquisa de 26/10)

- **Regulamento principal do @Work 2022:**
  - §4.4: equipes podem mapear a arena antes das provas;
  - §3.2.2, §3.2.4: paredes e fita vermelha/branca não mudam;
  - §4.9: proibido desenhar obstáculos no mapa.
- **Regulamento Entry Level 2026:** não confirmado (o link não abriu). Perguntar aos organizadores.
- **Plano B, sem mapa prévio:** SLAM do zero + exploração + nome das mesas pela câmera.
  As mesas já são detectadas no mapa ao vivo pelo `service_areas.py`.
