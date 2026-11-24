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
