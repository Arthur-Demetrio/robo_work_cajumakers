# Resultados de 28/10/2026: curva4, curva5, curva6, alinhador v3.15 e v3.16

Logs completos: `curva4_log.txt`, `curva5_log.txt`, `curva6_log.txt`. Gráfico: `curva4_folgas.png`.

## curva4 (09:08–09:13), alinhador v3.14
- **Ida:** travessia ok (parou a 15 cm da parede, alvo 12). Na saída lateral para a esquerda, deslizou 88 cm centrado (erro real ≤ 4 cm), mas abortou a 17 cm do fim: "não consegui alinhar em 8 correções". O contador só zerava depois de 15 cm de deslize, e cada deslize real tinha 10–12 cm.
- **Volta:** saída lateral concluída, mas as 8 correções acabaram no meio. A partir daí, ele deslizava sem parar. O robô derivou 11 cm para trás em 2 s, e a traseira ficou a 0–1 cm da parede por ~2 s.
- **Medida pela parede da frente:** supondo 0,82 m de largura, errava até 5 cm. A largura real no cruzamento variou de 0,80 a 0,87 m.
- **Deriva de lado:** 10–20% para frente/trás e uma rajada de ~50% por 2 s. Rumo: −2 a −3,5° por deslize para a esquerda.

## v3.15
- `folgas_lateral()`: folga real do para-choque e da traseira pelo LiDAR.
- Parada de segurança a 3 cm, que vale sempre; precisa de 2 scans seguidos.
- Rumo acima de 7° é corrigido sempre.
- O contador de correções zera a cada 6 cm de avanço. Sem correções e com folgas boas, segue deslizando.

## Reset da orientação sem perder o mapa (10:41)
`serialize_map`, reiniciar o slam_toolbox, `deserialize_map` com `match_type: 2` e `initial_pose` (0, 0, 0), limpar os costmaps.

## curva5 (10:50–10:54), v3.15: sucesso nas duas saídas laterais
- **Ida:** 24 s, dos quais ~10 s de vai e vem. A quina do cruzamento, vista de lado, contava como "parede de trás".
- **Volta:** 33 s, sem parada de segurança.
- **Preparo errado na volta:** o 1º goal saiu com eixo de −147° e preparo a 1 m, na área aberta. O caminho cortava o canto do L.
- **Área aberta:**
  - zigue-zague do DWB (1 m em 25 s), por causa do PWM mínimo;
  - giro final travado pelo Collision Monitor por 12 s, com uma parede a 17 cm na frente.

## v3.16
- O trecho estreito termina onde o caminho vira mais de 45°.
  - Replay dos caminhos do curva4/curva5: todos com o eixo certo.
  - `teste_cruz`: igual à v3.14.
- O centro pelas folgas só vale com parede de verdade dos dois lados. A parada de segurança usa o ponto mais perto, quina inclusive.

| Teste | v3.14 | v3.16 |
|---|---|---|
| `sim_lateral2.py`, modelo antigo (`DERIVA=0`) | — | 120/120, folga mín. 2,7 cm |
| `sim_lateral2.py`, deriva do curva4 | 108/120, encosta (0 cm) em 7 de 8 tipos | 112/120, folga mín. ≥ 1,0 cm |
| `teste_no_curva.py`, 6 seeds | 18/36 | 19/36 (as falhas da travessia já existiam) |

## curva6 (12:01–12:03): patch `dwb_sem_lateral.py` piorou e foi desfeito
- **O que aconteceu:**
  - Com vx de 0 a 0,08 em 7 amostras, o menor passo virou 0,013 m/s, abaixo do `deadband_velocity` 0,015 do velocity_smoother.
  - O DWB mandava 0,013, o smoother zerava, e veio o "Failed to make progress".
- **Conclusão:** o zigue-zague vem do PWM mínimo da ponte e não se resolve só no DWB.
- **Nav2 caiu uma vez:** às 12:02:31, sem heartbeat do planner_server e com o controller a 5,8 Hz. O lifecycle religou sozinho.
