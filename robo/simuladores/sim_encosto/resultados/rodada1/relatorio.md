# Bancada de encosto (Gazebo) - 09/11/2026 18:54

Criterio honesto: folga 6-12 cm, centro +-3 cm, angulo +-3 graus (pose real do Gazebo).

| cenario | esperado | classe | avaliacao | o alinhador disse | real: folga / centro / ang | menor dist mesa | frag/parcial/sem face |
|---|---|---|---|---|---|---|---|
| centrado r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.5cm ang=-1.8 centro=-1.1cm | 11.2 / -2.3 / -0.2 | 8.5 | 0/0/0 |
| lado_p05 r0 | sucesso | SUCESSO_FALSO | ruim | WS_1 SUCESSO folga=10.5cm ang=-2.0 centro=-0.3cm | 11.1 / -4.2 / -0.1 | 8.4 | 0/0/2 |
| lado_m05 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=-1.7 centro=+3.4cm | 9.7 / +0.1 / -0.5 | 8.7 | 0/0/0 |
| lado_p10 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=-1.4 centro=+0.0cm | 11.3 / -1.5 / -0.7 | 8.7 | 0/1/0 |
| lado_m10 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.5cm ang=-1.2 centro=+2.1cm | 9.5 / +1.3 / -0.9 | 8.5 | 0/1/0 |
| lado_m15 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=-0.8 centro=+3.7cm | 9.8 / +1.7 / -1.4 | 8.8 | 0/0/0 |
| lado_p30 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.6cm ang=+0.4 centro=+0.3cm | 11.3 / -1.0 / -2.5 | 8.6 | 0/3/0 |
| ang_p10 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.9cm ang=-0.2 centro=+2.2cm | 11.6 / -0.4 / -1.9 | 8.9 | 0/0/1 |
| ang_m10 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.6cm ang=-0.1 centro=+1.8cm | 11.3 / -0.6 / -2.0 | 8.6 | 0/0/4 |
| lado_p10_ang_m8 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.6cm ang=+0.4 centro=-0.0cm | 11.3 / -0.3 / -2.6 | 8.6 | 0/0/0 |
| perto_045 r0 | seguro | FALHA_EXPLICITA | bom | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 10.2 / -0.0 / +0.0 | 7.5 | 0/0/20 |
| obst_frente r0 | falha | COLISAO | ruim | WS_1 FALHOU encosto: nao vejo a face da mesa a frente | 25.0 / +0.8 / -0.9 | 23.3 | 0/0/17 |
| ponta_tapada r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.3cm ang=-1.8 centro=-0.8cm | 10.9 / -2.3 / -0.2 | 8.3 | 0/0/1 |
| lado_p10_sem_deslize r0 | sucesso | SUCESSO_FALSO | ruim | WS_1 SUCESSO folga=10.7cm ang=-2.0 centro=-0.0cm | 11.4 / -3.6 / +0.0 | 8.7 | 0/0/0 |
| lado_p10_deslize_re r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.3cm ang=-1.4 centro=-2.1cm | 11.0 / -2.3 / -0.6 | 8.3 | 0/0/0 |
| lado_m10_k05 r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.5cm ang=-1.1 centro=+1.7cm | 11.2 / -0.3 / -1.0 | 8.5 | 0/0/0 |
| raios_perdidos r0 | sucesso | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.9cm ang=-1.8 centro=-1.2cm | 11.6 / -2.1 / -0.2 | 8.9 | 0/0/5 |
| imu_falhando r0 | sucesso | SUCESSO_FALSO | ruim | WS_1 SUCESSO folga=10.3cm ang=-1.9 centro=-0.3cm | 11.0 / -4.4 / -0.0 | 8.3 | 0/2/0 |
| mapa_10g r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=-1.8 centro=-0.7cm | 11.4 / -1.7 / -0.2 | 8.7 | 0/0/1 |
| mapa_y15 r0 | seguro | SUCESSO_OK | bom | WS_1 SUCESSO folga=10.7cm ang=-1.9 centro=-0.7cm | 11.4 / -1.7 / -0.2 | 8.7 | 0/0/0 |
| ws2_centrado r0 | sucesso | SUCESSO_OK | bom | WS_2 SUCESSO folga=10.6cm ang=-1.8 centro=-1.1cm | 11.3 / -2.2 / -0.3 | 8.6 | 0/0/0 |
| ws2_lado_m10 r0 | sucesso | SUCESSO_OK | bom | WS_2 SUCESSO folga=10.9cm ang=-1.1 centro=-0.8cm | 11.6 / -0.6 / -1.1 | 8.9 | 0/2/0 |

Resumo: bom 18, ruim 4
