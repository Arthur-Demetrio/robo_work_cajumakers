# Bancada de encosto (Gazebo) - 09/11/2026 18:57

Criterio honesto: folga 6-12 cm, centro +-3 cm, angulo +-3 graus (pose real do Gazebo).

| cenario | esperado | classe | avaliacao | o alinhador disse | real: folga / centro / ang | menor dist mesa | frag/parcial/sem face |
|---|---|---|---|---|---|---|---|
| centrado r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=+0.1 centro=-2.4cm | 10.7 / -0.0 / +0.0 | 9.7 | 0/0/0 |
| lado_p05 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.8cm ang=+0.5 centro=-0.5cm | 10.8 / -0.4 / -0.4 | 9.6 | 0/0/0 |
| obst_frente r0 | falha | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 23.5 / -0.0 / +0.0 | 22.5 | 0/0/20 |
| lado_p10_sem_deslize r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.5cm ang=+0.1 centro=-2.3cm | 10.5 / -1.2 / +0.0 | 9.5 | 0/0/2 |
| imu_falhando r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.1cm ang=+0.5 centro=-0.7cm | 10.1 / +0.5 / -0.5 | 8.8 | 1/0/0 |

Resumo: bom 5
