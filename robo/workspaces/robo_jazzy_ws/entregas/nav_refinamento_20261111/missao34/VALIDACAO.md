# Missao34 — validacao offline, 11/11/2026

## Resultado
- Suite completa inicial: **265 passaram / 6 falharam**, 309 s (`pytest_inicial.log`).
- Duas falhas corrigidas: teste incompatível com SEM_SUAVIZADOR e perfis DWB/early sem CM_PROTECAO.
- Perfil GoalAlign desatualizado corrigido mantendo igualdade integral exceto GoalAlign.scale.
- Selecao final dos arquivos afetados: **76 passaram**, 48 s (`pytest_final.log`, `pytest_final.xml`). Inclui sondas com plugins reais Nav2, BT, configuracoes, tres patches e complemento de seguranca.
- Quatro testes ROS de destravar permanecem reprovados na execucao completa: CM inativo, retorno normal, comando estrangeiro e cancelamento. Diagnostico repetido somente para CM inativo: servidor nao descoberto e `getifaddrs: Operation not permitted` no ambiente restrito (`pytest_ros_diagnostico.log`). O processo destravar chegou a anunciar inicializacao. Isso nao valida nem reprova a logica dos quatro cenarios: impede executar suas assercoes comportamentais.
- Assercao de infraestrutura agora mostra JSON/log em vez de KeyError; nenhum skip/xfail foi introduzido. A suite completa nao foi repetida apos as correcoes; a selecao final cobre os arquivos afetados.

## Alteracoes e limites
- Testes de patch: aplicar, recusar duplicacao, desfazer e comparar bytes em copias temporarias. Nenhum backup anterior removido.
- CM_PROTECAO isolado: cobertura de -180 a 180 graus, velocidades de 0,005 a 0,35 m/s (inclui limites 0,05/0,12), rotacao -0,16/0/+0,16; selecao das diagonais, frente lenta/veloz e laterais; dimensoes frontais. Valida YAML/logica de selecao, nao frenagem real nem Collision Monitor ROS em execucao.
- SEM_SUAVIZADOR: ausencia explicita de SmoothPath, chamada proibida no simulador, copia do caminho e bloqueio explicito em modo aplicar. CaminhoOriginalSeguro permanece registrar em producao, sem veto adicional. Suavizadores de velocidade continuam existentes.
- CM_PROTECAO tambem aplicado aos perfis dwb_test e early_test pelo script existente; GoalAlign regenerado da config atual, sem mudar o teste de igualdade.
- CURVA_INSISTE_SEGURO: retentativas/fallback so para timeout; bloqueios CM/TF/odom interrompem. Maximo duas retentativas; cancelamento impede novas tentativas; giro_frontal experimental nunca cai no Nav2. Timeout ainda permite repeticao sem comprovar progresso. Nao houve teste fisico.
- Rollback: `python3 ferramentas/patches/aplicar_curva_insiste_seguro.py --desfazer` ANTES de desfazer o patch curva_insiste original. Os demais scripts mantem --arquivo/--desfazer e backups. Outros arquivos alterados possuem .bak_validacao34_*; perfis DWB/early possuem .bak_cm_protecao_*.

## SLAM mapping vs localization
- Replays existentes completos: 29.707 amostras cada, aproximadamente 594 s. Comparador `../scripts/comparar_slam_m34.py`; saida `../slam_replay/comparacao_m34.json`.
- Apos descartar identidade inicial: mapping 602 correcoes, localization 605. Salto angular maximo 21,00 / 36,05 graus; percentil 95 6,70 / 6,01 graus; 64 / 50 correcoes acima de 5 graus.
- Localization nao apresenta vantagem suficiente para promocao: menor contagem >5 graus, mas pior extremo. Ambos instaveis. Sem verdade de referencia nao se conclui precisao absoluta.
- Deltas x/y da origem odom NAO sao saltos de posicao do robo; exigem odom simultanea para essa interpretacao. Logs registram descarte de scans nos dois modos e encerramento forcado do processo pelo script depois da gravacao. O marcador fim sozinho nao prova qualidade do replay.
- Nenhum novo replay foi iniciado; nenhuma config SLAM foi promovida.

## Proximas validacoes
1. Reexecutar os quatro testes ROS isolados em ambiente com descoberta DDS permitida, antes de declarar a suite integral aprovada.
2. CM: validar frenagem e cobertura do obstaculo real (incluindo diagonal) em bancada/simulacao apropriada antes de teste fisico autorizado.
3. Verificar progresso das retentativas por timeout e estabilidade SLAM com referencia independente. Manter localization sem promocao.
4. Mudancas estao locais: build/subida futura conforme procedimento do projeto, apenas em janela autorizada. Nesta sessao nao houve restart de Nav2 nem movimento do robo.
