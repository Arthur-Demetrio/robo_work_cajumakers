# Supervisor Nav2 + alinhador e perfil localization

Nenhum restart operacional, goal ou movimento foi executado. Apenas ensaio SLAM isolado com dominio 187/use_sim_time, sem publicar scans, TF de hardware ou comandos. Backups incluem arquivos tracked e untracked anteriores a esta rodada. Instalacao operacional `install/` preservada; build em `build/nav2_principal`, `install_nav2_principal`.

## Auditoria e implementacao

- executor_prova envia areas por `/ir_para_area`, espera resultado do alinhador. RViz/interface usa `/goal_pose_alinhado`. Essas interfaces foram mantidas; topicos nao carregam UUID externo. Supervisor cria UUID logico por pedido, copia posicao e quaternion completos, mantem o mesmo objeto original em retomadas; cada envio Nav2 recebe UUID novo. Preempcao usa trabalhador serial e ultimo pedido pendente existente.
- Nav2/recuperacoes -> `/cmd_vel_nav_base`; alinhador -> `/cmd_vel_nav_especial`; arbitro exclusivo com lease e filtro temporal -> `/cmd_vel_nav` -> velocity_smoother/Collision Monitor -> `/cmd_vel` -> hardware. Nao foi criado publisher paralelo nem contorno do CM. Cancelamento aguarda resultado terminal, inclusive aceitacao tardia. Antes de transferir, heartbeat e retirado e exige comando final recente zerado por 0,4 s; ausencia de comando nao e parada confirmada.
- Estados NAV2/APROXIMACAO/TRANSFERENCIA/ALINHADOR/RETOMADA/FALHA registrados com UUID em rosout. Default observar so registra candidatos; aplicar exige whitelist geometrica no mapa mais detector existente do plano restante e duas paredes LiDAR concordantes com largura do mapa. Caixa fora da whitelist ou parede unica nao transfere. Cadastro vazio e intencional: nao existem coordenadas de entradas/saidas confiaveis dedutiveis dos pontos de bancada. Medir e cadastrar no mapa selecionado antes de aplicar.
- Preparo seguro e rotinas Travessia/TravessiaLateral existentes reutilizados, inclusive T. Nao reinicia navegacao comum durante travessia. Nova NavigateToPose com pose original obriga replanejamento ao retomar. Bancadas e GOAL_TF_ATUAL preservados. DWB, BT, CM e destravar nao foram ajustados; principal ja permitia re DWB -0,10 antes desta tarefa, legado/destravar continuam bloqueados. Nao reintroduzido recuo automatico do alinhador em falha no perfil principal.
- Evidencia independente integra incrementos ICP do modulo existente recuperacao_avancada, conferindo yaw com IMU e saltos com SLAM. Frescor 0,5 s, intervalo ICP <=0,8 s, divergencia angular <=4 graus, salto translacional <=10 cm/incremento, divergencia acumulada <=15 cm. Incerteza inicial 6 cm cresce por residuo/ambiguidade e aborta acima de 14 cm. Valores conservadores precisam de validacao fisica; nao foram afrouxados para fazer o teste passar.
- Saida exige todos os vertices do footprint alem do plano de saida em ambos os referenciais SLAM e ICP, margem 5 cm mais incerteza, pelo menos dois incrementos/progresso observado >10 cm, saida LiDAR com observacao nos dois lados e sem retorno na faixa de passagem, controle final parado. Tempo e odometria de comandos nao produzem sucesso. ICP ambiguo, SLAM saltando, IMU/scan ausente ou deriva levam a falha segura; rotinas antigas sozinhas nao autorizam retomada.
- Latch de passagens concluidas persiste entre pedidos no processo. Rearma somente ao voltar pelo lado da entrada (mais de 65 cm antes e ate 30 cm do eixo). Nao ha retomada automatica apos reiniciar processo; arbitro inicia bloqueado. `/navegacao/cancelar` reaproveita cancelamento existente; resposta e solicitacao, nao prova de parada.

## SLAM

`slam_mapping.yaml` permanece inalterado. Launch mapping usa online_async; launch localization usa localization_slam_toolbox_node. `robo_localization` publica odom->base_link por comando/IMU, nao map->odom. AMCL nao e iniciado por esses launches. O script de subida anterior escolhia localization com mapa por default, mas podia reutilizar um slam_toolbox previamente existente sem verificar seu modo; portanto arquivo mapping sozinho nao provava modo ativo. Sem acesso ao grafo operacional, modo efetivamente em execucao nao pode ser confirmado nesta sessao.

Perfil localization agora parte do mapping atual e difere somente em mode, map_file_name e map_start_pose. Mapa sem extensao: `/home/marcos/mapas/arena_04-11_1235`; pose START [0.192,0.004,-0.171]. Posegraph 5.339.893 bytes e data 3.065.379 bytes, cabecalho Boost e SHA256 registrados na geracao. PGM nao e necessario para desserializar. Validacao estrutural nao garante integridade semantica do grafo; replays anteriores desse par completaram, mas novo perfil precisa validar inicializacao.

`subir_robo.sh --localization` valida mapa/START antes de hardware, gera perfil para o mapa escolhido, recusa SLAM ja existente, exige map e map->odom antes de Nav2. Sem flag conserva fallback mapping; nao ativar localization operacional antes de validar inicializacao e unicidade TF no ambiente permitido. Launch localizacao falha antes do node se faltar par/pose. A assinatura do servico DeserializePoseGraph nao tem campo de resultado; /map e TF devem ser confirmados, nao somente exit code de service call.

Reprocessado comparador de replays34 completos existentes (~594 s, 29.702 amostras uteis cada). Mapping/localization: 64/50 correcoes >5 graus; max 21,00/36,05 graus; P95 6,70/6,01 graus. Nao houve novo replay completo. Comparacao IMU original em `imu_m34.json`: 593 amostras a 1 Hz, maior incremento do yaw no mapa 21,14 graus mesmo com IMU <1 grau no entorno amostrado. Isso nao alinha IMU aos dois replays nem comprova precisao absoluta. Localization continua sem evidencia suficiente de estabilidade.

## Ativacao futura supervisionada

Apos encerrar a instancia Nav2 anterior, com mapa/localizacao ja validos, sem goals automaticos:

```bash
cd ~/robo_jazzy_ws
bash ferramentas/iniciar_supervisor_validado.sh observar
# Depois de cadastrar regioes e conferir sensores, em outra subida autorizada:
bash ferramentas/iniciar_supervisor_validado.sh aplicar
```

O wrapper recusa instancia Nav2/alinhador/arbitro existente e cadastro vazio em aplicar. Nao executar ambos. Teste SLAM isolado em ambiente DDS permitido, sem stack operacional no mesmo dominio:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install_nav2_principal/setup.bash
ROS_DOMAIN_ID=187 ros2 launch robo_navigation localizacao_slam.launch.py use_sim_time:=true
```

Exige replay de /clock, scan e odom/TF adequado; somente iniciar o node nao comprova localizacao. Subida futura no START, apos validacao e com stack anterior encerrado: `~/subir_robo.sh --localization arena_04-11_1235`. Rollback de modo: `~/subir_robo.sh --mapeando arena_04-11_1235`, em janela autorizada, nunca em paralelo.

## Limites e pendencias

- Cadastrar regioes medidas; aplicar permanece sem transferencia com arquivo vazio.
- Validar ICP em scans durante movimento (modulo original nasceu para fotos paradas); ambiguidades de corredor podem impedir toda travessia. Sensores sem referencia independente suficiente falham, nao estimam sucesso por comando.
- Margens/deriva lateral, latencia de frenagem e footprint precisam de ensaio fisico autorizado. A saida exige tambem incrementos ICP menores que 1 cm/0,01 rad e gyro menor que 0,03 rad/s por 0,4 s; esses limiares de imobilidade precisam de validacao em hardware.
- Clientes que enviam diretamente a action Nav2 contornam a interface supervisionada; usar os topicos existentes. Nao implementado proxy publico de NavigateToPose nem persistencia de goal apos restart.
- Falta teste multiprocesso com DDS e verificacao ao vivo de unicidade do map->odom; ensaio desta rodada encontrou `getifaddrs: Operation not permitted`. Nao ativar localization automaticamente.

## Resultado das verificacoes

- Build isolado robo_navigation aprovado (`build.log`); dependencia robo_recovery usada do underlay operacional existente, sem recompila-la nem altera-la.
- Selecao final: **155 passaram em 29,81 s** (`testes_corrigidos.log`): supervisor, coordenador real por extracao AST, missao35, BT/config, lateral/T e bancada/face. Inclui espera real de parada com comandos nao zerados, stale, IMU em giro e ICP parado; saida parcial, T, ICP invalido, salto SLAM, deriva, preempcao/retomada, caixa/whitelist, caminho livre e rollback dos perfis existentes.
- Execucao ampliada anterior: **187 passaram / 1 falhou**, 106,90 s (`testes_finais.log`); inclui os **38 testes de recuperacao** aprovados. Unica falha: fake do adaptador lateral nao implementava novo hook de evidencia. Fixture atualizada sem relaxar sua assercao de TF perdida/sem movimento; passou na selecao final. Os seis erros da primeira selecao eram igualmente fixtures do coordenador sem os novos atributos, corrigidas preservando verificacoes e adicionando ordem parada->especial->confirmacao->Nav2.
- `py_compile`, `bash -n` e `git diff --check`: aprovados. Geracao do perfil valida par, START e igualdade integral de todos os outros parametros mapping/localization.
- Patch reproduzivel e rollback: `python3 entregas/supervisor_localization_20261111/patch.py`; idempotente quando ja aplicado. `--arquivo <diretorio-copia>` testado: aplicar, reaplicar, desfazer duas vezes e comparar bytes aprovados. Recusa qualquer edicao posterior antes de alterar arquivos.
- Rollback desta rodada, sem iniciar processos: `python3 entregas/supervisor_localization_20261111/patch.py --desfazer`. Restaura inclusive ~/subir_robo.sh e arquivos previamente untracked; remove somente os novos arquivos desta implementacao cujos hashes ainda conferem. Preserva backups, relatorio e evidencias. Recompilar overlay isolado antes de voltar a usar perfis instalados. Rollback nao troca parametros de processos vivos.
- Ensaio ROS isolado localization: inicializacao/TF **nao validados**. DDS repetiu `getifaddrs: Operation not permitted`; node encerrou pelo timeout/SIGINT, sem hardware. Nao mascara essa pendencia com skip/xfail. Replay completo novo e ensaio fisico nao executados.

Arquivos da rodada: consultar `patch_manifest.json` (15 arquivos, incluindo AGENTS/CLAUDE, dois testes existentes, CMake, launch Nav2/SLAM, alinhador, novo supervisor, regioes, perfil localization, preparador/wrapper e ~/subir_robo.sh). Diff desta rodada pode ser reconstruido comparando backup/ com aplicado/; o git diff global inclui muitos patches anteriores que nao pertencem a esta entrega.
