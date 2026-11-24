# Auditoria: comando para a frente durante movimento lateral do alinhador — 11/11/2026

Só leitura: nenhum código, parâmetro ou processo foi alterado. Bags: missao20, 22 (06/11 08:32), 23, 24, 32.
Scripts em `scripts/` (rodar com `source /opt/ros/jazzy/setup.bash`):
- `lateral_vx.py <bag...>`: inícios de lateral no `/cmd_vel_nav` (|vy| ≥ 0,03), `vx` em nav/smoothed/final;
- `lateral_vx_fino.py`: o mesmo com |vy| ≥ 0,012 (preparo e encosto);
- `lateral_lidar.py lateral.json saida.json`: avanço real pelo LiDAR (paredes da frente e de trás do corredor).

## 1. Código: onde o alinhador manda vx junto com vy
| Trecho | Comando lateral | vx junto? |
|---|---|---|
| Preparo do corredor (`Travessia`, deslize) | `(0, ±lat_cmd, wz)` | **não** |
| Travessia (andando dentro do corredor) | vx para a frente + vy de correção ≤ 0,025 | sim, intencional (o movimento é para a frente) |
| **Saída lateral** (`TravessiaLateral`, deslizar, ANGULO_FUNDIDO **ligado** por padrão) | `vy = ±0,08` | **sim: `vx = −fundido_k_ff·vy + 0,4·centro` (±0,04), com mínimo de 0,016** |
| Saída lateral, estado `ajustar` (antes de deslizar, parado entre medidas) | — | pulso só em vx (±0,018 cmd, ~0,09 real, ≤ 6 cm), sem vy |
| Encosto (deslize na frente da mesa) | `(0, ±0,02, 0)` | **não** |
| TRAJETO CURTO | pernas separadas (lado, depois frente) | **não** |

Com `fundido_k_ff` 0,2 (padrão):
- **indo para a direita**, `vx = +0,016` (para a frente) mais a correção de centro;
- **indo para a esquerda**, `vx = −0,016` (para trás).
O comentário do código diz "deslize torto medido: para a frente indo para a esquerda".
A ponte (`calculate_motor_pwm`) soma o PWM mínimo em todas as rodas: ela não aumenta nem apaga esse vx.

## 2. Bags: o que saiu de fato (início de cada saída lateral, 1,5 s)
| Missão | Sentido | n | vx no `/cmd_vel_nav` | vx no `/cmd_vel` (depois do CM) |
|---|---|---|---|---|
| missao24 | DIR | 10 | **+0,016 a +0,045 em 30/30 mensagens** | igual (+0,016 a +0,040) |
| missao24 | ESQ | 30+ | −0,016 a −0,038 (nenhum para a frente) | 0 ou negativo |
| missao20, missao22 (antes do ANGULO_FUNDIDO) | ambos | 72 | 0 | 0 |
Preparo, encosto e TRAJETO CURTO na missao23/24: vx = 0 em todos os deslizes.
Fora do alinhador: nos goals do Nav2 o DWB + `suavizador_nav` mandaram vx junto com lateral (até 0,098 no final).
É resíduo do movimento anterior: no legado `preservar_eixos_zero` é false (o LATERAL_SEM_PULSO_M35 só liga no nav2_principal).

## 3. Avanço real pelo LiDAR (cm; + = para a frente; mediana, máximo entre parênteses)
| Missão | Sentido | vx cmd | 0,5 s | 1,0 s | 1,5 s |
|---|---|---|---|---|---|
| missao20 | ESQ | 0 | +0,4 | +1,0 (+16,3) | +1,1 |
| missao22 | ESQ | 0 | +0,5 | +0,6 | +1,1 (+9,1) |
| missao23 | ESQ | −0,016 | +0,2 | +2,0 | +2,5 (+7,4) |
| missao24 | ESQ | −0,016 | +0,6 | +2,5 | +2,8 (+10,7) |
| missao20 | DIR | 0 | −0,4 | −2,2 | −1,8 |
| missao22 | DIR | 0 | −0,4 | −0,9 | −2,7 |
| missao24 | DIR | +0,016 | −0,3 | −2,6 | −6,2 |

Leitura:
- **Indo para a esquerda, o robô avança sem nenhum comando para a frente.** Com lateral pura, foi cerca de 1 cm em 1,5 s, com picos de 9–16 cm. Esse é o "impulso frontal": é mecânico (mecanum/rolos/peso), não vem do software.
  Ele cresce no primeiro segundo; a 0,5 s ainda é menor que 1 cm.
- Indo para a direita, o robô recua.
- Com a compensação do ANGULO_FUNDIDO (missões 23/24), a deriva **não diminuiu**: foi +2,8 em vez de +1,1 para a esquerda e −6,2 em vez de −2,7 para a direita.
  **Isso não prova que o sinal está invertido.** O peso do robô mudou em 06/11, o lugar do corredor muda entre missões, e a correção de
  centro (0,4·centro) reage à própria deriva. Na missao23/24, mais vx para trás veio junto com mais avanço (correlação −0,28 a −0,67),
  o que é coerente com a correção reagindo à deriva e não serve como prova de causa.

## 4. Conclusão e próximo passo (sem mudar compensação lateral sem calibração física)
1. O software manda vx para a frente durante o lateral **só na saída lateral para a direita** (ANGULO_FUNDIDO).
   No preparo do corredor, no encosto e no TRAJETO CURTO, o lateral é puro.
2. O impulso para a frente indo para a **esquerda** é físico: o comando é para trás.
3. Isso confirma, em 6 missões, o que a `entregas/nav_refinamento_20261111/FICHA_CALIBRACAO_LATERAL.md` viu no bag de 10/11:
   para a esquerda o robô avança, para a direita ele recua, com o mesmo sinal que o `k_ff` supõe, mas maior e assimétrico.
   Lá também aparece um recuo de 12–21 cm no destravar com vx = 0.
4. Calibração: seguir a ficha (teleop, trena, sem alinhador, 3 corridas por linha). Depois, para confirmar no alinhador,
   1 bag por valor na mesma saída lateral: `ros2 param set /alinhador_passagem fundido_k_ff 0.0`, depois `0.2` (atual),
   e medir com `scripts/lateral_lidar.py`. Se os dois sentidos pedirem k diferentes, o modelo `vx = −k·vy` (simétrico)
   não serve: precisa de um k por lado, como opção desligada por padrão. Isso é compensação lateral: só depois da medida.
