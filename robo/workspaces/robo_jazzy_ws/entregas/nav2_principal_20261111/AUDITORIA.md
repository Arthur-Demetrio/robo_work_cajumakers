# Auditoria e arquitetura

## Fluxo preservado
- Executor publica String `/ir_para_area`; espera `/ir_para_area/resultado`. Nao comanda rodas. Cadastro distingue bancada real de START/FINISH (sem_encosto).
- RViz/interface envia PoseStamped `/goal_pose_alinhado`. Callback mantido, ultimo pedido substitui anterior.
- Alinhador adapta ambas as entradas para NavigateToPose. No novo modo, envia pose/yaw originais; nao chama _analisar/rota_no_mapa, trajeto curto, VIRAR_CAMINHO, anti-ciscado ou REINSISTIR.
- Nav2 conserva BT integrada, DWB direto, plugins CaminhoOriginalSeguro/CaminhoSemCarimbo (GOAL_TF_ATUAL), planejamento, replanejamento e destravar v4. Goal recebido/reenvio apos manobra reinicia caminho_do_goal=false.

## Especializacao
- Vigia le somente plano Nav2 publicado apos inicio do goal, endpoint correspondente, idade <=16 s (BT mantem plano por 15 s), trecho restante mais proximo da pose atual. Nao calcula rota estatica alternativa nem limpa costmap para encurtar rota.
- Tomada de controle requer geometria de passagem do mapa, largura fisicamente compativel de 0,67 a 0,90 m, duas paredes LiDAR com largura concordante em 12 cm, scan recente, TF valido e entrada a <=1,5 m. Caixa/parede unica nao basta. Sem confirmacao, Nav2 permanece dono.
- Preservados preparo com giro seguro local, travessia LiDAR, T lateral, encosto e saida da bancada. Modo giro_frontal continua suspenso. Falha de manobra encerra com parada; nao declara sucesso nem tenta forcar continuidade.
- Cada travessia/T concluido e seguido por nova NavigateToPose para o goal original; bancada terminal devolve autoridade no proximo goal, inclusive recuo antes da nova navegacao.

## Comandos e concorrencia
- Legado: controller -> suavizador_nav e recuperacoes/alinhador compartilham cmd_vel_nav; cancelamentos eram apenas solicitados em alguns caminhos, com risco de aceitacao tardia e sobrescrita de comandos.
- Novo: controller -> suavizador_nav -> cmd_vel_nav_base; behavior_server, progressive_spin, mecanum_escape e destravar usam base tambem. Alinhador -> cmd_vel_nav_especial. Arbitro unico -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> Collision Monitor -> cmd_vel -> hardware.
- Arbitro inicia bloqueado, troca sempre publica zero, filtra origem e stamp posterior a troca, descarta mensagens antigas/futuras e exige heartbeat com lease de 0,5 s. Heartbeat carimbado; backlog anterior a troca nao renova lease. Morte/perda de autoridade resulta em zeros. Suavizadores de velocidade e CM nao foram removidos.
- Antes de enviar goal ou tomar controle especial, action anterior deve terminar. Pendente de aceitacao e conservada, nao esquecida. Cancelamento tardio cancela action aceita depois de preempcao. Timeout bloqueia autoridade; trabalhador tambem drena na saida e antes do proximo pedido. Nao ha thread de movimento por goal, apenas um trabalhador com fila de ultimo pedido.
- Arbitragem garante exclusividade entre navegacao e especial. Clientes externos que publiquem diretamente em cmd_vel_nav/cmd_vel contornam o contrato e precisam estar ausentes na bancada. Nao se trata de autorizacao criptografica ou E-stop.

## Separacao da missao34
- Perfil novo foi derivado do YAML atual desfazendo somente CM_PROTECAO; BT nova desfaz somente SEM_SUAVIZADOR (mantem GOAL_TF_ATUAL e recuperacoes). Novo coordenador nao executa CURVA_INSISTE; launch tambem passa insistir_curva=0/curva_segue_nav2=false.
- Arquivos principais pendentes da missao34 nao foram revertidos nem promovidos. Instalacao operacional intacta. Build usa overlay separado. Nenhum processo do stack foi reiniciado; nao houve envio de comandos ao dominio do robo nem movimento nesta sessao.
