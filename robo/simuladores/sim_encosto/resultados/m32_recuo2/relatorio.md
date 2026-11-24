# Bancada de encosto (Gazebo) - 09/11/2026 19:26

Criterio honesto: folga 6-12 cm, centro +-3 cm, angulo +-3 graus (pose real do Gazebo).

| cenario | esperado | classe | avaliacao | o alinhador disse | real: folga / centro / ang | menor dist mesa | frag/parcial/sem face |
|---|---|---|---|---|---|---|---|
| m32_t3 r0 | seguro | SUCESSO_FALSO | ruim | WS_1 SUCESSO folga=10.4cm ang=+0.9 centro=+0.5cm | 9.5 / +3.1 / -3.0 | 8.3 | 1/0/3 |
| m32_t3 r1 | seguro | SUCESSO_FALSO | ruim | WS_1 SUCESSO folga=10.5cm ang=+0.9 centro=+0.7cm | 9.6 / +3.0 / -3.0 | 8.5 | 0/1/2 |
| m32_t4 r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.6cm ang=-0.8 centro=+3.8cm | 9.8 / +1.5 / -1.3 | 8.7 | 0/1/1 |
| m32_t4 r1 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.8cm ang=-0.7 centro=+3.5cm | 9.9 / +0.5 / -1.4 | 8.8 | 0/0/2 |

Resumo: bom 2, ruim 2
