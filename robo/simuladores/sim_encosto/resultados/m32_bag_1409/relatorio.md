# Bancada de encosto (Gazebo) - 09/11/2026 19:09

Criterio honesto: folga 6-12 cm, centro +-3 cm, angulo +-3 graus (pose real do Gazebo).

| cenario | esperado | classe | avaliacao | o alinhador disse | real: folga / centro / ang | menor dist mesa | frag/parcial/sem face |
|---|---|---|---|---|---|---|---|
| m32_t3 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 8.5 / +16.2 / -1.6 | 7.0 | 0/0/24 |
| m32_t3 r1 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 8.6 / +16.3 / -1.6 | 7.1 | 0/0/25 |
| m32_t4 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 8.1 / +4.6 / -1.0 | 7.0 | 0/0/24 |
| m32_t4 r1 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 8.4 / +4.8 / -0.9 | 7.3 | 0/0/26 |
| m32_t4_k08 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: perdi a face da mesa | 8.2 / +2.0 / -1.2 | 7.1 | 0/0/18 |
| m32_t4_k08 r1 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: perdi a face da mesa | 8.0 / +2.3 / -1.2 | 7.0 | 0/0/18 |
| m32_lado_m20_k1 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 8.3 / +8.1 / -1.2 | 7.2 | 0/0/24 |
| m32_lado_m20_k1 r1 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 8.6 / +8.3 / -1.2 | 7.4 | 0/0/23 |

Resumo: bom 8
