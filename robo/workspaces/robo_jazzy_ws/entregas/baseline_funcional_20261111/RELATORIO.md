# Baseline funcional da navegação — 11/11/2026 (interrompido a pedido)

Estado: o trabalho foi **encerrado pelo usuário** durante a última execução da suíte completa (foi interrompida, sem resultado).
Nada foi reiniciado: nenhum goal foi enviado e o robô não se moveu. Não mexi em SLAM, firmware nem nos parâmetros do DWB.

## 1. Escolha da baseline (evidência: `/rosout` dos bags, `resumo_missoes.json`, script `scripts/resumo_missoes.py`)
| Missões | Resultado |
|---|---|
| missao11/12 (04/11, v3.24/25) | 4/4 sucessos cada, com obstáculo |
| missao20 (05/11) | 8/8 (duas voltas MISSAO2) |
| **missao24 (06/11) = git HEAD 7201367** | 9 sucessos/12, duas voltas completas; encosto 7/7 quando chegou |
| missao25–32 (09/11) | ~1 sucesso; quase tudo "face ausente/fora da referência WS" |
| missao33–35 (11/11, integrada/nav2_principal) | nenhuma área concluída |

Causa das falhas de encosto a partir de 09/11: a face da mesa passou a ser **filtrada pela referência do mapa** (pose do SLAM + cadastro).
Exemplo da missao32 (WS_1): a face real estava a d0.62 a+19g e era recusada como `FORA:ang` (referência a−1g) dezenas de vezes.
Na missao24 a face era escolhida só pelo LiDAR.
Nas missões 24–35 o SLAM esteve em mapping, exceto a missao30.

**Baseline adotada:** arquitetura legada (2D Goal Pose → `/goal_pose_alinhado` → alinhador → Nav2/destravar), código atual,
com o encosto voltando ao comportamento da missao24.
Ficam mantidos: corredores, T, saídas laterais, aproximações, GOAL_TF_ATUAL, DWB direto, destravar v4/recuperação avançada,
CM e ré bloqueada. O perfil legado instalado já tinha DWB direto e GOAL_TF_ATUAL, e `min_vel_x` é 0.

## 2. Backup
`~/robo_jazzy_ws/backups/baseline_funcional_20261111_161212/` contém: `src_ferramentas_docs.tar.gz` (src+ferramentas+CLAUDE/AGENTS,
inclusive os arquivos não commitados), `git_diff_tracked.patch`, `git_status.txt`, `git_head.txt`, `subir_robo.sh`, `mapas_yaml.tar.gz`,
`install_robo_navigation.tar.gz` (install antes do build) e `SHA256SUMS`.
Cada patch também deixou um `<arquivo>.bak_<nome>_20261111_16xxxx`.

## 3. Alterações (todas por patch em `ferramentas/patches/`, com marca, recusa dupla, `--desfazer` e teste em cópia)
Ordem de aplicação:
**Baseline**
1. `aplicar_face_lidar_m24.py` (face_ws.py, FACE_LIDAR_M24): `RastreioFaceWS(face, usar_mapa=False)`. A fase A escolhe a maior face
   com as 2 pontas e largura igual à cadastrada ±12 cm, a ≤1,0 m e ±35°. A fase B (trava pela odometria, fragmentos) não mudou.
   O padrão da classe continua `usar_mapa=True`.
2. `aplicar_baseline_legado.py` (alinhador): novo parâmetro `encosto_referencia_mapa` (padrão **False** = missao24).
3. `aplicar_baseline_subida.py` (~/subir_robo.sh): **legado por padrão**. O supervisor novo só sobe com `--supervisor` ou
   `--supervisor-aplicar`, e o código dele foi preservado. `--legado` continua aceito.
4. `aplicar_baseline_launch.py` (navigation.launch.py): passa `reinsistir_goal_final:=0` ao alinhador (o REINSISTIR não existia na missao24).
   O padrão do código continua 2, para não quebrar o patch nem o teste do REINSISTIR.

**Melhorias isoladas** (cada uma com parâmetro para voltar ao comportamento anterior)
5. `aplicar_controlador_orfao_recuperacao.py` + `aplicar_controlador_orfao_destravar.py` (CONTROLADOR_ORFAO): se o controlador
   ainda está comandando quando a recuperação começa, o destravar cancela todos os FollowPath (`/follow_path/_action/cancel_goal`
   com goal_info vazio) e espera até 2 s parado. A decisão continua exigindo exclusividade.
   Um comando de outra origem nunca é cancelado.
   Evidência: no bag ensaio_goal_superior, um FollowPath órfão do goal cancelado deu ré a −0,10 m/s por cerca de 10 s
   e provocou 2 recusas "sem controle exclusivo".
6. `aplicar_controlador_orfao_alinhador.py`: ao fim de cada NavigateToPose do caminho legado, espera 0,35 s. Se o controlador
   continuar comandando, cancela o FollowPath e espera até 1,5 s até ele parar.
   Parâmetro `cancelar_controlador_orfao` (True). Não faz nada no perfil nav2_principal.
7. `aplicar_pre_giro_unico.py`: não repete o pre-giro (VIRAR_CAMINHO) para o mesmo alvo (±15 cm) dentro de 20 s.
   Parâmetro `pre_giro_repetir_s` (0 = comportamento anterior).
8. `aplicar_ciscado_progresso.py`: `_chamar_destravar` passa a devolver se teve sucesso. A partir da 2ª intervenção do ANTI_CISCADO, se o destravar
   falhou, o goal não é reenviado e o resultado é 'falha'. Parâmetro `ciscado_exige_destravar` (True).
9. `aplicar_rota_segura_launch.py`: o launch passa `rota_mapa_obstaculo:=recusar` (com objeto novo a <35 cm da rota do mapa,
   fica com o caminho do planner). O padrão do código continua 'ignorar'.

10. `aplicar_envio_goal_seguro.py` (ENVIO_GOAL_SEGURO, aplicado depois da 1ª versão deste relatório): novo `_enviar_goal`
   no NavigateToPose legado e no goal do destravar. Se o goal novo (ou o timeout de aceitação) chega antes de o Nav2
   aceitar o velho, o goal aceito depois é cancelado na hora. Antes ele ficava rodando e dirigia junto com o pré-giro
   ou as manobras do alinhador até o próximo envio. Só script; não precisa de build.

Build: `colcon build --packages-select robo_navigation` foi executado no install **operacional** (backup feito antes). O install ficou igual ao src
para launch, nav2_params.yaml e BT. Os scripts rodam direto do src.

## 4. Testes
- Baseline (antes das melhorias): suíte completa **366 aprovados, 0 falhas** (incluindo os testes ROS isolados do destravar).
  Antes disso, 1 falha veio do meu primeiro patch, que alterava o texto do CURVA_INSISTE; foi corrigido mudando a abordagem para o launch.
- Testes novos, todos aprovados:
  - `test_baseline_legado.py` (11): face a 19° aceita sem mapa e recusada com mapa; parede longa recusada; scans reais da missao32
    reconhecidos; subida e launch legados.
  - `test_controlador_orfao.py` (5, simulação): o órfão é cancelado e a recuperação segue; um órfão que não para continua falhando
    sem mover o robô; comando de outra origem não é cancelado; cancelamento do goal durante a espera.
  - `test_controlador_orfao_ros.py` (5, **ROS real em domínio isolado**): o CancelGoal vazio cancela o FollowPath falso
    e o `/cmd_vel_ctrl` para; a lógica do alinhador foi extraída do fonte.
  - `test_melhorias_legado.py` (12): pré-giro, `_chamar_destravar`, ANTI_CISCADO e rota segura.
  - `test_recuperacao_avancada.py`: 38 aprovados depois do CONTROLADOR_ORFAO.
- `test_substituicao_goal_ros.py` (5, **ROS real em domínio isolado**, `_navegar_legado` real extraído do fonte contra
  um NavigateToPose falso): substituição durante a execução, antes da aceitação, aceitação depois do timeout,
  `cancelar_t` e uma sequência de 3 goals. **Antes do ENVIO_GOAL_SEGURO: 3 falhas** (goal velho ativo); depois: 5 aprovados.
- Smoke em domínio ROS isolado: alinhador ("Alinhador pronto") e destravar ("v4 pronto, ré BLOQUEADA") sobem sem erro.
- **Suíte completa depois das melhorias 5–9 (repetida em 11/11): 388 aprovados; com o ENVIO_GOAL_SEGURO: 393 aprovados, 0 falhas, 176 s** (log em `pytest_final.log`). Comando:
  `cd ~/robo_jazzy_ws/src/robo_navigation && source /opt/ros/jazzy/setup.bash && source ~/robo_jazzy_ws/install/setup.bash && python3 -m pytest -q --ignore=test/sondas_nav2 test`

## 5. Pendências
- Validação física: encosto sem referência do mapa (WS_1/WS_2), cancelamento do FollowPath órfão no robô, pre-giro único e ANTI_CISCADO.
  Fazer um goal e um bag por teste.
- Teste de substituição com o nó alinhador inteiro (sensores/TF falsos) ainda não existe; o atual roda o código real de
  envio/espera/cancelamento.
- Duas mudanças da M34 no perfil legado (SEM_SUAVIZADOR, CM_PROTECAO) nunca rodaram no robô: ver `COMPARATIVO_M24_ATUAL.md`.
- Traceback no SIGINT do alinhador (`_parar` depois do shutdown, em `main()`): já existia antes desta rodada e é cosmético.
- Ainda sem explicação: no perfil nav2_principal, o árbitro manteve o lease 2,6 s depois do fim do goal (o teste isolado bloqueia em 0,43 s).
  Isso não afeta o legado.
- Achado da rodada anterior (só leitura): o SLAM em localization caça ±3–6° (reproduzido em replay). O scan bate mal com o
  mapa arena_04-11_1235 (cerca de 50% dos pontos a mais de 15 cm de qualquer parede, mesmo no melhor ajuste), o que indica que a arena mudou.
  Remapear deve ser avaliado. Material em `entregas/supervisao_localizacao_20261111/`.
- Atualizar CLAUDE.md/AGENTS.md com esta rodada (não foi feito).

## 6. Ativação (com o robô parado no START)
```
~/subir_robo.sh arena_04-11_1235      # legado por padrão (mapping)
~/gravar_teste.sh <nome> "baseline legado"   # em outro terminal
```
Voltar comportamentos em tempo de execução:
`ros2 param set /alinhador_passagem encosto_referencia_mapa true`, `... pre_giro_repetir_s 0.0`, `... ciscado_exige_destravar false`,
`... cancelar_controlador_orfao false`, `... rota_mapa_obstaculo ignorar`, `... reinsistir_goal_final 2`.
Supervisor novo: `~/subir_robo.sh --supervisor arena_04-11_1235`.

## 7. Rollback
`bash ~/robo_jazzy_ws/entregas/baseline_funcional_20261111/desfazer_tudo.sh` (desfaz em ordem inversa) e depois
`cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation`.
Alternativa: restaurar os arquivos de `backups/baseline_funcional_20261111_161212/`.
