# Teste 11/11 18:44 — corredor bloqueado, alinhador insistiu (log ~/logs/11-11_184040/nav2.log)

Relato do usuário: o mapa perdia orientação. Um corredor estava bloqueado e o outro livre; o robô insistiu no bloqueado.

## O que o log mostra (só leitura)
1. Goal (1,67; 2,35). `ROTA_MAPA_OBSTACULO (recusar)`: objeto a < 35 cm da rota do mapa, a 1ª célula em (1,42; 0,08).
   Logo depois: **`Planner: sem caminho; mapa: 3.8 m: sigo a rota do mapa`**.
2. TRAVESSIA 1 abortou 2 vezes ("algo a 10 cm / 9 cm do para-choque"), cada uma com saída de ré e volta ao preparo. Depois:
   "Nao consegui atravessar a passagem 1".
3. Goal (1,67; 3,03): de novo "Planner: sem caminho; sigo a rota do mapa". O destravar respondeu SEM CAMINHO duas vezes
   (limpou os costmaps) e terminou em "sem alternativa segura" (ré bloqueada, lateral com pontos do LiDAR na área varrida).
4. Goal (0,03; 0,06): planner sem caminho também para um destino a 1,2 m.

## Causa no código (`_analisar`, ROTA_MAPA + ROTA_MAPA_OBSTACULO)
- O `recusar` só vale quando o planner **tem** caminho (`novos and modo_obs == 'recusar' and pontos is not None`).
  Com o planner sem caminho, o alinhador usa a rota mais curta do mapa estático, mesmo com objeto detectado nela.
- Uma travessia que falhou por obstáculo não marca a passagem como bloqueada. Na próxima tentativa a rota do mapa é a mesma, e a
  alternativa pelo outro corredor nunca é calculada (`rota_no_mapa` não exclui passagens).
- O planner sem caminho até para 1,2 m indica um costmap global fechado. Isso é coerente com a perda de orientação (paredes fantasmas
  do SLAM, como na nota de 10/11). **Hipótese**: não medi o costmap.

## Proposta (não implementada; uma mudança por vez, depois do remapeamento)
`PASSAGEM_BLOQUEADA` no alinhador legado. Depois de uma travessia abortada por obstáculo, ou com o ROTA_MAPA_OBSTACULO detectando
objeto e o planner sem caminho, a passagem fica marcada como bloqueada por N s para esse destino. `rota_no_mapa` passa a calcular a
rota sem ela. Se não houver rota alternativa: falha explícita, sem insistir. Opção desligável, com teste offline usando o mapa e a
caixa no corredor.

## Implementado (11/11 19:00): `ferramentas/patches/aplicar_passagem_bloqueada.py` (marca PASSAGEM_BLOQUEADA)
Estado: **implementado no src, testado offline** (17 testes puros em `test/test_passagem_bloqueada.py`). **Não está ativo**:
o alinhador em execução é anterior ao patch, então só vale na próxima subida. Ainda não testado no robô.
- Travessia abortada por "algo a N cm do para-choque": os pontos do LiDAR livres no mapa viram bloqueio (r 0,35 m,
  `passagem_bloqueada_s` 120 s). Com outro caminho no mapa, não faz a 2ª tentativa: reanalisa e vai pelo outro. No máximo 2 desvios por goal.
- Planner sem caminho e objeto na rota do mapa (ROTA_MAPA_OBSTACULO recusar): tenta a rota do mapa sem o objeto.
- Objeto parcial com espaço para o corpo: continua no mesmo corredor.
- Desligar: `ros2 param set /alinhador_passagem passagem_bloqueada_s 0.0`. Desfazer: `python3 ferramentas/patches/aplicar_passagem_bloqueada.py --desfazer`.
- Limite: o Nav2 não conhece o bloqueio; o preparo do outro corredor é levado pelo Nav2 com o costmap do momento.
