# Navegacao — regras locais

- Trabalhar em portugues, de forma objetiva. Nao reiniciar stack/Nav2, enviar goals ou movimentar hardware sem autorizacao explicita para o ensaio.
- Antes de editar, preservar backup datado inclusive de arquivos untracked. Nao apagar backups nem usar reset/clean destrutivos.
- Arquitetura opt-in `nav2_principal:=true`: Nav2 navega; alinhador e adaptador da interface `/goal_pose_alinhado` e `/ir_para_area`, e executa somente bancada/saida de bancada e passagem confirmada por mapa + duas paredes LiDAR.
- Em navegacao comum, nao reintroduzir pre-giro obrigatorio, trajeto curto, rota estatica substituindo NavFn, anti-ciscado paralelo ou retentativa fora da BT. Preservar yaw pedido, GOAL_TF_ATUAL, DWB direto, destravar v4, recuperacoes e Collision Monitor.
- Toda transferencia especial exige resultado terminal das actions anteriores, nao apenas ACK de cancelamento. Goal novo deve interromper e o ultimo pedido prevalece. Aceitacao tardia/timeout nunca autoriza movimento concorrente.
- No novo modo, publishers Nav2/recuperacoes usam `/cmd_vel_nav_base`; alinhador usa `/cmd_vel_nav_especial`. Somente `arbitro_nav_principal` publica `/cmd_vel_nav`, seguido por velocity_smoother -> CM -> hardware. Nao contornar essa cadeia. Lease vencida ou comando anterior a troca deve ser descartado.
- Manter rotinas existentes de bancada/corredor e modo legado. Flag somente de inicializacao; nao alternar arquitetura no meio de goal.
- Os tres patches pendentes da missao34 nao fazem parte do perfil novo: `nav2_params_principal.yaml` exclui CM_PROTECAO; `navigate_to_pose_principal.xml` conserva SmoothPath; CURVA_INSISTE fica sem retentativa/fallback no launch. Nao promove-los silenciosamente.
- Compilar esta arquitetura em `build/nav2_principal` / `install_nav2_principal`; nao sobrescrever a instalacao operacional como parte de validacao offline.
- Testes: `test_nav2_principal.py`, suites de BT/config/rotinas existentes e `ferramentas/testes/replay_nav2_principal.py`. Replay de decisoes nao comprova trajeto fisico alternativo. Testes ROS multiprocesso exigem DDS permitido; documentar limitacoes, nao mascarar com skip/xfail.
- Evidencias e rollback: `entregas/nav2_principal_20261111/`. Ler CLAUDE.md para contexto operacional.
- Missao35: o perfil principal agora permite re DWB ate -0,10 m/s; nao restaurar a assercao global de que toda configuracao proibe re. Perfil legado continua sem re; destravar v4 conserva permitir_re=false. CM traseiro do perfil principal ampliado, nao reduzido.
- Scan corporal: remover apenas pontos dentro do envelope fisico medido (-0,215..0,375; y +-0,325), como NaN antes de /scan_front. Nao substituir por inf nem bloquear setores inteiros para esconder motores; obstaculos alem do corpo devem permanecer. Montagem da mascara deve acompanhar os argumentos da TF, inclusive fluxo temporal.
- SLAM: manter configuracao original de mapping conforme escolha do usuario. Arquivo foi comparado byte a byte com a missao33. Saltos observados no map->odom devem ser medidos contra IMU/odom; nao prometer estabilizacao apenas pela filtragem.

## SUPERVISOR_LOCALIZATION_20261111
- Pedido atual autoriza preparar localization para teste e substitui a antiga proibicao de editar esse perfil; mapping permanece byte a byte e rollback preserva o estado anterior.
- Supervisor integrado ao trabalhador do alinhador, nao iniciar outro controlador. `supervisor_modo=observar` e padrao; `aplicar` exige regioes medidas em `regioes_passagem.yaml` (cadastro atualmente vazio, transferencia vetada).
- Estados e identificador logico do pedido em `/rosout`; copia integral da pose original. Interfaces topicas nao fornecem UUID externo; identificador local acompanha a retomada, que recebe novo UUID Nav2.
- ICP reutiliza recuperacao_avancada e valida incrementos contra SLAM/IMU; perda de confianca encerra sem recuo automatico. Toda saida exige footprint completo nos referenciais SLAM e ICP, margem/incerteza, observacao de saida e parada. Nao desabilitar checks para obter sucesso em corredor ambiguo.
- `/navegacao/cancelar` solicita cancelamento pelo mecanismo existente; ACK nao confirma parada. Novos pedidos devem usar `/goal_pose_alinhado` ou `/ir_para_area`; clientes diretos do action Nav2 contornam a supervisao.
- `subir_robo.sh --localization <mapa>` e somente opcao futura supervisionada; mapping fica como fallback padrao enquanto inicializacao/TF nao forem validados no ambiente ROS. Nunca inferir estabilidade pelo modo localization.
- Detalhes, limitacoes, testes e rollback: `entregas/supervisor_localization_20261111/RELATORIO.md`. Nao reiniciar stack nem enviar goals nesta sessao.

## SUBIDA_SUPERVISOR_20261111 — correcao do comando operacional
- `~/subir_robo.sh` agora seleciona o novo perfil por padrao via `iniciar_supervisor_validado.sh observar`. Legado somente com `--legado`; `--supervisor-aplicar` exige cadastro valido de regioes, sem ignorar whitelist.
- Validar overlay/perfil/BT antes de hardware. Antes de anunciar ROBO PRONTO, exigir parametros reais nav2_principal=true, supervisor_modo correto e arbitro no grafo. Lifecycle ativo sozinho nao comprova perfil.
- Codigo alterado nao modifica processos vivos. Nao afirmar que supervisor passou a controlar o robo sem verificar nova subida. Restricao DDS/GUI desta ferramenta permanece; nao contornar via segundo controlador.
- Detalhes e rollback separado: entregas/correcao_subida_supervisor_20261111/RELATORIO.md. Nao zerar yaw nem gravar START a partir da imagem/pose divergente.

## PARTIDA_ARBITRO_20261111
- Ambiente agora permite ROS/DDS e GUI. Subida real 15:27 confirmada: localization com arena_04-11_1235, nav2_principal=true, supervisor=observar e arbitro exclusivo. Restricoes de acesso anteriores sao historicas.
- Nao usar uma unica consulta `ros2 node list --no-daemon` como prova de ausencia: descoberta pode ser parcial. Usar ferramentas/conferir_supervisor_ativo.py, contexto ROS persistente, servico/parametros/endpoints estaveis por 1,5 s. Verificador nunca solicita autoridade nem publica velocidades.
- Validacao falha fechada para parametro errado, servico ausente ou qualquer segundo publisher de /cmd_vel_nav. Timeout nao autoriza continuar.
- Arbitro trata SIGINT/contexto invalidado sem tentar publicar depois de shutdown. Abertura via ferramentas/abrir_robo_supervisor.sh mantem terminal aberto ao encerrar.
- Evidencias, testes ROS isolados e rollback: entregas/partida_arbitro_20261111/. Nao confundir observar com travessias ativas; cadastro de regioes segue vazio. Sem goals/movimento nesta rodada; START nao regravado.

## LOGGER_SEVERIDADE_20261111 / goal superior
- rclpy fixa severidade por callsite. Nao usar getattr(logger,nivel)(texto) na mesma linha para info/warn/error: usar chamadas explicitas em linhas distintas. Regressao com rclpy real reproduz erro antigo e valida alternancia corrigida.
- Goal 15:31/15:32 falhou no NavFn antes de /plan novo. Observar nao autoriza takeover; regioes seguem vazias. Nao promover aplicar nem substituir caminho sem validar geometria/sensores.
- Snapshot somente leitura: mapa bruto conecta inicio ao goal, costmap global nao (mesmo permitindo unknown). Nao tratar isso como ausencia comprovada de espaco fisico nem ajustar SLAM/DWB por esse log. Evidencias/testes/rollback: entregas/goal_superior_20261111/.
