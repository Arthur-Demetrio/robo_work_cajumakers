# Bancada de encosto (Gazebo) - 09/11/2026 19:23

Criterio honesto: folga 6-12 cm, centro +-3 cm, angulo +-3 graus (pose real do Gazebo).

| cenario | esperado | classe | avaliacao | o alinhador disse | real: folga / centro / ang | menor dist mesa | frag/parcial/sem face |
|---|---|---|---|---|---|---|---|
| centrado r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=11.0cm ang=-1.8 centro=-1.1cm | 11.6 / -2.1 / -0.2 | 9.0 | 0/0/3 |
| lado_m15 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.5cm ang=-0.8 centro=+3.8cm | 9.6 / +1.6 / -1.4 | 8.6 | 1/0/2 |
| lado_p30 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.4cm ang=+0.3 centro=-2.4cm | 11.1 / -1.6 / -2.4 | 8.5 | 0/0/1 |
| obst_frente r0 | falha | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 25.2 / -0.0 / +0.0 | 22.5 | 0/0/20 |
| ponta_tapada r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=-1.9 centro=-0.7cm | 11.3 / -1.8 / -0.2 | 8.6 | 0/0/1 |
| m32_t3 r0 | seguro | SUCESSO_FALSO | ruim | WS_1 SUCESSO folga=10.9cm ang=+1.1 centro=+2.3cm | 10.0 / +2.1 / -3.1 | 8.9 | 0/1/5 |
| m32_t4 r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=-0.7 centro=+3.8cm | 9.7 / +1.1 / -1.3 | 8.7 | 0/2/1 |
| m32_t4_k08 r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.8cm ang=-0.8 centro=+3.7cm | 9.9 / +1.0 / -1.3 | 8.9 | 0/1/0 |
| m32_lado_m20_k1 r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.8cm ang=-0.2 centro=+2.0cm | 9.9 / +1.6 / -1.9 | 8.8 | 0/0/1 |

Resumo: bom 8, ruim 1
