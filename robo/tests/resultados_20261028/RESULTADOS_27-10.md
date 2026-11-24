# Resultados de 27/10/2026

## Provas do dia
- **prova3 (08:24):** travou 2 min em área aberta. O LiDAR (a 4 cm do chão) viu um objeto embaixo do robô, parado no mapa enquanto o robô girava. Com ≥ 3 pontos no polígono, o Collision Monitor deu STOP e todas as recuperações foram bloqueadas. Não foi a árvore.
- **Correção:** `filtro_corpo_scan.py` remove os pontos dentro do corpo do robô (`/scan_front` → `/scan_nav`), e o `usar_scan_nav.py` faz o CM e o costmap local usarem esse tópico. Nos dados gravados, o STOP falso caiu de 98% dos scans para 0%.
- **prova4 (09:22, WS_1 → WS_2 → WS_1):** missão completa em ~2 min 50 s, sem STOP falso. Falhou no FINISH: o ponto era o START de ontem e ficava a 0,31 m da parede. O `executor_prova.py --marcar-start` agora recusa gravar se a folga for menor que 0,50 m.

## Alinhador v3.11–v3.14: cruzamentos
- Reto (+ ou T atravessado): uma travessia só. Curva (L, T, +): para no cruzamento e sai de lado, sem girar.
- v3.12: para no cruzamento pelo LiDAR (o mapa errava 11–15 cm no eixo do corredor).
- v3.13/v3.14: ajuste em pulsos, parede de trás só perto de onde deve estar, faixa de medida estreita perto da saída.
- Testes no robô: curva1–curva3 (parcial; curva3 11:51 funcionou, 12:06 se perdeu na saída).
