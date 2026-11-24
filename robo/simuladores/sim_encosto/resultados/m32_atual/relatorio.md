# Bancada de encosto (Gazebo) - 09/11/2026 19:12

Criterio honesto: folga 6-12 cm, centro +-3 cm, angulo +-3 graus (pose real do Gazebo).

| cenario | esperado | classe | avaliacao | o alinhador disse | real: folga / centro / ang | menor dist mesa | frag/parcial/sem face |
|---|---|---|---|---|---|---|---|
| m32_t3 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: face ausente ou desatualizada | 9.1 / +16.9 / -1.5 | 7.7 | 1/6/0 |
| m32_t4 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: centro da face nao medido (so face parcial, sem as 2 pontas): nao alinho nem avanco | 7.4 / +4.1 / -1.0 | 6.3 | 4/5/2 |
| m32_t4_k08 r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=9.0cm ang=-0.9 centro=+3.2cm | 8.1 / +2.3 / -1.2 | 7.0 | 2/2/0 |
| m32_lado_m20_k1 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: centro da face nao medido (so face parcial, sem as 2 pontas): nao alinho nem avanco | 6.8 / +6.7 / -1.4 | 5.7 | 5/5/1 |

Resumo: bom 4
