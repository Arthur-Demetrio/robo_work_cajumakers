# PARTIDA_LATERAL — início do deslize lateral do alinhador (12/11/2026)

**Estado:** implementado no src, testado offline, **desligado por padrão** (sem efeito no comportamento), **não ativo** em nenhum processo, **não testado fisicamente**. Ponte (C++), firmware, Nav2, Collision Monitor e destravar **não foram alterados**. O robô não foi reiniciado nem movido.

## 1. O que a missão 34 mostra (CURVA 1, saída lateral para a ESQUERDA, 11:03:12–11:04:08)
Fonte: bag `~/bags/missao34_20261111_110217_913450`, só leitura. Scripts em `ferramentas/analise/partida_lateral/`.
O avanço foi medido pelas paredes da frente e de trás do corredor (`/scan_front`, faixa |y| < 0,22 m). As duas paredes concordam.
O primeiro deslize (11:03:12) foi excluído: o robô ainda estava no T, e a parede da frente troca durante o deslize.

| | mediana (n = 10 deslizes) |
|---|---|
| Avanço real por deslize | **4,6 cm** (0,5–6,2) |
| … na rampa (primeiros ~0,35 s) | 1,4 cm (3,2 cm/s) |
| … em regime | 2,1 cm (5,7 cm/s) |
| … depois do zero (inércia) | 0,4 cm |
| vx no `/cmd_vel_nav` / `/cmd_vel_hardware` | **−0,016 a −0,039 / −0,016 a −0,035** (sempre para trás) |
| Duração com motor ligado | 0,45–1,3 s |

**Comando × deriva:** em nenhum momento houve vx positivo comandado. Os 4,6 cm são **deriva mecânica**, e acontecem *contra* um vx comandado para trás.
O ciclo se repetiu 10 vezes: desliza ~1 s, a folga da frente cai para 3–4 cm (PARADA DE SEGURANÇA), recua 7 cm, mede e desliza de novo. Terminou com "Collision Monitor bloqueou 4 vezes".

**Hipótese (n = 9, r = 0,80):** a taxa de avanço cai com vx mais negativo. O ajuste é deriva ≈ 20 + 574·vx (cm/s), e a deriva zera perto de **vx ≈ −0,035** (k_ff ≈ 0,44 para a esquerda; hoje é 0,2).
A confusão do "centro reage à deriva" puxaria a correlação para o lado oposto, então o sinal é plausível. Mesmo assim, são janelas curtas com LiDAR a 5,9 Hz: **isso não é calibração.**

## 2. Cadeia de comando reproduzida offline (`ferramentas/analise/partida_lateral/modelo.py`)
- **velocity_smoother** (OPEN_LOOP, scale_velocities, deadband 0,015, estado antes do deadband): 99,5% das 1264 amostras do `/cmd_vel_smoothed` reproduzidas com erro < 0,003.
- **Ponte** (`calculate_motor_pwm` / `wheel_speed_to_pwm`): 1335 de 1335 quadros de `/hardware/tx_frames` idênticos, diferença máxima de 0 PWM.

## 3. Causa do início ruim
1. **O feedforward chega atrasado (software).** O smoother escala vx junto com vy (para preservar a direção) e zera |vx| < 0,015.
   Com k_ff 0,2 (vx −0,016), o vx só chega às rodas quando vy passa de ~0,06: **passo 7, ~0,3 s** depois da primeira roda girar. No bag o atraso foi de 0,05–0,30 s (mediana 0,15).
   Nesse trecho o deslize é lateral puro: PWM (−79, 79, 79, −79) → (−103, 103, 103, −103), sem compensação.
2. **O feedforward é fraco em regime.** vx −0,02 dá 120 vs 94 PWM entre os pares, e o robô ainda avança 5,7 cm/s. O par FL/RR (que gira para trás) rende menos que o FR/RL.
   O sinal da deriva bate com os dois lados: esquerda avança, direita recua (auditoria de 11/11). Isso é coerente com desequilíbrio entre os pares diagonais (hipótese mecânica: peso, rolos, motores).
3. **Não são a causa:**
   - **Firmware `stepMotorRamp`:** parte direto de 0 para o PWM pedido (79) nas 4 rodas no mesmo ciclo de 5 ms. Depois disso a ponte sobe ~100 PWM/s, abaixo do limite de 200 PWM/s, então a rampa do firmware nunca atua. Não houve "tranco" mensurável: a deriva na rampa (3,2 cm/s) é menor que em regime. A IMU não publica aceleração (`linear_acceleration` = 0), então não há como medir o tranco por ela.
   - **`wheel_speed_to_pwm`:** o offset do PWM mínimo é igual nos dois pares e se cancela na diferença. O vx comandado chega às rodas com a mesma escala (~0,7) do resto.
   - **Heading hold:** correção de 0,005–0,025 rad/s (≤ 0,012 m/s por roda), dyaw < 1,1° por deslize. Segura o rumo, mas não gera avanço em x.
   - **Parada:** firmware zera na hora (`requested == 0`) e o smoother leva 3–4 passos (0,5 m/s²). Inércia de 0,4 cm. Nada a mudar.

## 4. Correção preparada (`ferramentas/patches/aplicar_partida_lateral.py`, marca `PARTIDA_LATERAL (12/11)`)
Só a `TravessiaLateral._deslizar_fundido`, a declaração de parâmetros e a chamada em `_travessia_lateral`.

| Parâmetro (`/alinhador_passagem`) | Padrão | Efeito |
|---|---|---|
| `fundido_k_ff_esq` | −1 (desligado → usa `fundido_k_ff`) | k do feedforward indo para a esquerda |
| `fundido_k_ff_dir` | −1 (desligado) | k indo para a direita |
| `partida_lateral_s` | 0 (desligado; para testar: 0,5) | na partida segura vy em `partida_lateral_v0` (0,03) por `partida_lateral_espera` (0,15 s) e sobe a `partida_lateral_acel` (0,16 m/s²), já com o vx do regime |

Proteções da partida:
- |vx| ≤ |vy| − 0,012, para que nenhuma roda fique abaixo de ~PWM 76 (o mínimo é 70): **torque mínimo preservado**;
- as 4 rodas partem juntas em 79, como hoje;
- parada, PARADA DE SEGURANÇA, Collision Monitor, correções paradas, abortos, preparo, travessia e encosto ficam iguais.

Replay offline (alinhador real → smoother → ponte), esquerda:

| | vx chega às rodas | vy chega a 0,08 | PWM mínimo em movimento |
|---|---|---|---|
| baseline, k 0,2 | passo 7 (~0,30 s) | passo 7 | 79 |
| partida 0,5, k 0,2 | **passo 2 (~0,05 s)** | passo 10 (+0,15 s) | 77 |
| baseline, k 0,44 | passo 3 | passo 7 | 79 |
| partida 0,5, k 0,44 | **passo 2** | passo 10 | 76 |

**Ganho esperado (estimativa, não medida):** ~0,25 s a mais de compensação por deslize. Com 3,2 cm/s na rampa, isso dá ~0,5–0,8 cm a menos dos 4,6.
**O ganho principal depende do k por lado calibrado.** A partida sozinha não resolve a missão 34.

## 5. Testes offline
- `src/robo_navigation/test/test_partida_lateral.py` (13 testes):
  - desligado = idêntico, comando a comando, ao código original (reconstruído pela reversão do patch), para os 2 lados e 6 folgas;
  - k por lado muda só o lado configurado;
  - rampa monotônica, e igual à baseline depois da partida;
  - nenhuma roda abaixo do mínimo (k 0,2/0,5/1,0);
  - PARADA DE SEGURANÇA igual;
  - replay: vx chega antes; desaceleração e parada com PWM idênticos;
  - baseline reproduz o atraso medido (passo 7);
  - patch: aplicar / recusar a 2ª vez / desfazer byte a byte numa cópia.
- Suíte completa (`python3 -m pytest -q --ignore=test/sondas_nav2 test`, `ROS_DOMAIN_ID=87`, stack desligada): baseline **413 aprovados / 0 falhas** (`pytest_baseline.log`); depois do patch, ver `pytest_final.log`.

## 6. Ativação (só em janela autorizada, um bag por mudança)
O alinhador roda do src: a mudança vale ao reiniciar o `~/subir_robo.sh`. Com os padrões, nada muda.
Ativar em tempo de execução (um por vez, cada um com o seu bag):
```bash
ros2 param set /alinhador_passagem partida_lateral_s 0.5        # teste A: só a partida
ros2 param set /alinhador_passagem fundido_k_ff_esq 0.44        # teste B: só depois da calibração (valor da ficha)
```
O parâmetro vale no próximo deslize (a `TravessiaLateral` é criada a cada curva).
Desligar: `ros2 param set /alinhador_passagem partida_lateral_s 0.0` e `... fundido_k_ff_esq -1.0`.
Rollback do código: `python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_partida_lateral.py --desfazer`. Backup: `alinhador_passagem.py.bak_partida_lateral_20261112_083101`.

## 7. Calibração física (operador, robô supervisionado; nenhum script comanda movimento)
1. Siga a `entregas/nav_refinamento_20261111/FICHA_CALIBRACAO_LATERAL.md`:
   - teleop lateral puro (vx 0), 5 s, vy 0,08;
   - esquerda e direita; piso liso e com frisos;
   - 3 corridas por linha; trena para dx/dy;
   - gravar com `~/gravar_teste.sh`.
2. Repita com vx = −0,035 indo para a esquerda e +0,035 indo para a direita (teleop diagonal). Isso confirma ou derruba a hipótese do item 1.
3. Calcule k_lado = vx que zera dx ÷ 0,08, por lado e pela mediana. Se os dois lados derem k parecidos (±0,05), use só `fundido_k_ff`.
4. No alinhador, um bag por valor, na mesma saída lateral da missão 34:
   1. baseline;
   2. `partida_lateral_s 0.5`;
   3. `fundido_k_ff_esq <k>`.

   Meça com `fases.py` (rampa / regime / inércia) e compare com os 4,6 cm por deslize. **Critério:** mediana < 2 cm por deslize e nenhuma PARADA DE SEGURANÇA pela frente.
5. Só depois disso mudar o padrão (num patch próprio).

## Arquivos
- `ferramentas/patches/aplicar_partida_lateral.py` (patch, `--arquivo`, `--desfazer`);
- `src/robo_navigation/test/test_partida_lateral.py`;
- `ferramentas/analise/partida_lateral/`: `extrair_m34.py`, `analisar_m34.py`, `fases.py`, `linha.py`, `modelo.py`, `smoother_cmp.py`, `ponte_cmp.py` e os JSON extraídos;
- `hashes.txt`, `pytest_baseline.log`, `pytest_final.log`.
