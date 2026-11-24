# Ficha de calibração do deslize lateral (Patch 03)

O operador faz tudo à mão. **Nenhum script comanda movimento.** Para medir no bag depois, use `~/gravar_teste.sh` e `scripts/lateral_vx.py`.

## O que o bag de 10/11 já mostra

Pelo SLAM, que vem do casamento de scan, e pelo ICP do destravar. A odom é derivada do comando e **não** prova deslocamento.

| Caso | Comando | Lateral real | Longitudinal real | dx/dy |
|---|---|---|---|---|
| 12:34:21, alinhador, esquerda, vy +0,08 | vx −0,034 em média (ff −0,016 + parede −0,03) | +25,9 cm | +1,4 cm (a odom dizia −22 cm) | ≈ 0 (o comando cancelou a deriva) |
| 12:39:02, destravar, direita, vy −0,03 | vx = 0 | −24,4 cm (ICP −27) | **−12,1 cm (ICP −16)** | ≈ +0,5 (recuo) |
| 12:40:18, alinhador, direita, vy −0,025 | vx = 0 | −11,1 cm | **−15,8 cm** | > 1 |
| 12:42:16, alinhador, direita, vy −0,08 | vx +0,023 em média (ff +0,016 + correção) | −38,9 cm | **−29,2 cm** | ≈ 0,75 (recuo mesmo com ff) |
| 11:57:10 e 12:01:07 (bag 11:48), destravar, direita | vx = 0 | −16,5 / −23,6 cm | **−18,0 / −21,3 cm** | ≈ 1 |
| 11:56:52 (bag 11:48), destravar, esquerda | vx = 0 | +11,0 cm | **+23,2 cm** (avança) | ≈ 2 |

**Leitura.** Indo para a **direita**, o robô **recua**. Indo para a **esquerda**, **avança**. É o mesmo sinal que o `k_ff` assume, mas o tamanho é maior e assimétrico. Comandos de 0,025–0,03 m/s estão abaixo do mínimo de lado (~0,06 m/s), então o motor executa mais forte e torto.

No destravar, a "ré durante o deslize" é **deriva mecânica**: o `vx` comandado é 0. No alinhador, o `vx` negativo indo para a esquerda é **comandado** e intencional.

Na saída para a direita das 12:41:33–12:42:34 (61 s), houve 10 paradas de segurança com folga traseira de 3–4 cm.

Os números acima vêm de uma corrida por caso, em piso com frisos. **Ainda não são calibração.**

## Protocolo (um bag por linha, mesma bateria, robô parado entre corridas)

1. Marque no chão o centro do robô, com fita e caneta, e a direção da frente.
2. Comande só o lateral pelo teleop. **Não** use o alinhador. Duração fixa de 5 s.
3. Meça com trena: dx (frente +) e dy (esquerda +) do centro, e dyaw com transferidor ou pela IMU.
4. Anote também o ICP e o SLAM do bag (`lateral_vx.py`).

| # | Piso | Lado | vy comandado | vx comandado | dx trena (cm) | dy trena (cm) | dyaw (°) | dx SLAM | dy SLAM | dx/dy |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | liso | direita | 0,08 | 0 | | | | | | |
| 2 | liso | esquerda | 0,08 | 0 | | | | | | |
| 3 | liso | direita | 0,03 | 0 | | | | | | |
| 4 | liso | esquerda | 0,03 | 0 | | | | | | |
| 5 | frisos | direita | 0,08 | 0 | | | | | | |
| 6 | frisos | esquerda | 0,08 | 0 | | | | | | |
| 7 | frisos | direita | 0,03 | 0 | | | | | | |
| 8 | frisos | esquerda | 0,03 | 0 | | | | | | |

Repita cada linha 3 vezes e use a mediana.

## Decisão depois da medição (fora deste pacote)

- Se o dx/dy da direita e o da esquerda forem diferentes, troque o `fundido_k_ff` único por `k_ff` por lado, como opção desligada por padrão, e faça um bag por lado.
- Se o destravar recuar ≥ 5 cm por 20 cm de lado, aplique a ele o mesmo feedforward, também desligado por padrão.
- Não use deslize abaixo de 0,06 m/s de lado: o motor não faz.
