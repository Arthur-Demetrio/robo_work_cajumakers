# Encerramento e dificuldades — 11/11/2026

## Estado final
A pedido do usuario, solicitado cancelamento em /navegacao/cancelar e encerrada a subida PID58266/grupo58264 por SIGINT. Nav2, alinhador, arbitro, SLAM, odometria/hardware/LiDAR da subida encerrados, sem processos ros2 launch remanescentes na verificacao. Gravador PID59514 finalizado por SIGINT e cache descarregado. Mapa arena_04-11_1235.posegraph/.data preservado. Nao iniciado remapeamento, nao gravado START e nenhum goal enviado apos ordem de encerrar. Janelas de logs podem permanecer abertas, mas sao leitores de arquivos encerrados; RViz nao e um processo SLAM.

## 1. Orientacao/localizacao instavel
SLAM estava em localization. Bag de aproximadamente200s tem9582 transformacoes map->odom,4662 IMUs e58 correcoes detectadas acima de0,001grau ou1mm. Houve9 incrementos de yaw maiores que3graus; maior6,535graus em cerca20ms (timestamp1794422880,7067). Nesse evento a origem map->odom tambem deslocou17,29cm. A janela IMU de0,5s contem12 amostras e amplitude angular0grau; isso apoia que a mudanca de orientacao era correcao do referencial de localizacao, nao giro medido pela IMU. Nao e ground truth de posicao: translacao da origem odom nao equivale a deslocamento fisico do robo. Detalhes em yaw_bag.json e yaw_bag.py. Confirma numericamente a observacao do usuario de perda de orientacao.

Remapear pode ser necessario. Ainda nao foi isolado se a causa e mapa/ambiente alterado, ambiguidades do scan matcher, calibracao/tempo do LiDAR, odometria por comandos ou combinacao. Trocar para localization nao estabilizou o sistema nesta sessao. Um novo mapa precisa ser acompanhado de validacao desses fatores para nao herdar o problema.

## 2. Conexao do mapa nao corresponde a conexao do costmap
Snapshots anteriores ao movimento mostram inicio e goal(1,72;1,81) em celulas livres. Mapa bruto conecta; costmap global nao conecta, mesmo permitindo desconhecidos no teste offline. Extra de65 celulas letais em celulas livres do mapa na captura nova. Fechamento relevante na entrada da curva perto de(1,62;0,13), com zona inscrita253. O caminho diagnostico de celulas teria de atravessar5 celulas bloqueadas; NAO foi usado para controlar o robo.

Esse fechamento tem correspondencia em scans atuais: retorno em(1,6163;0,1370) tanto no scan_front quanto scan_nav, a4,9mm do centro de uma celula bloqueante. Nao era somente memoria antiga ou efeito introduzido pelo filtro downstream. Pode haver obstaculo real, deslocamento de geometria em relacao ao mapa ou erro de registro. Nao existe evidencia suficiente para simplesmente apagar retornos/reduzir inflacao. Mapas/scans e correlacoes preservados em snapshot.npz, scans.npz, obstaculos_e_scans.json e grade_diagnostico.json.

Busca offline de registro limitada encontrou melhoria deslocando10cm/girando3graus, mas score por endpoints sem raycasting/IMU/observabilidade nao autoriza corrigir pose automaticamente. Nao aplicada essa sugestao ao SLAM.

## 3. Planejamento e controle falham em momentos diferentes
Primeiras tentativas: NavFn sem caminho, inclusive apos BT limpar global costmap; resultado208 corresponde NO_VALID_PATH no ComputePathToPose instalado. Depois, com movimento/goals diferentes, apareceram caminhos e o DWB recusou404 trajetorias por ObstacleFootprint/Trajectory Hits Obstacle. Logo o bloqueio nao e apenas uma excecao de logger ou uma unica falha estatica. Costmap e pose mudam ao longo da sessao. Margens, footprint, DWB e SLAM nao foram alterados nesta analise para forcar movimento.

## 4. Supervisor ainda nao executou a funcao completa pedida
Arquitetura principal/arbitro ativos e confirmados; supervisor estava observar. Nao concede takeover nesse modo. Regioes conhecidas continuam vazias, portanto aplicar permanece vetado. Detector depende de plano novo valido do goal atual; quando NavFn falha antes de gerar caminho, a travessia nao e acionada. Cadastro/validacao de regioes e ensaio real de transferencia, travessia, curva, confirmacao de saida e retomada continuam pendentes. Testes offline aprovados nao representam validacao fisica dessas rotinas. A implementacao ainda nao esta validada para conduzir a missao desejada.

## 5. Recuperacao e exclusividade
Houve recusas por falta de continuacao lateral e, em momentos posteriores, por comando estrangeiro recente. A entrada de exclusividade de destravar esta remapeada para /cmd_vel_nav_base; portanto NAO e demonstrado que o arbitro na saida /cmd_vel_nav estivesse sendo contado indevidamente. Nao ignorada protecao de conflito. E necessario identificar o emissor e a sequencia de cancelamento/termino antes de corrigir essa integracao. A primeira coleta passiva de publisher_gid falhou porque MessageInfo veio como dict; helper foi corrigido, mas coleta nao revalidada com stack apos ordem de encerrar. Nenhuma conclusao de identidade do emissor e sustentada por essa ferramenta nesta sessao.

## 6. Ensaio nao isolado e atribuicao de movimento
Usuario confirmou que enviou os outros goals. Bag registra6 goals durante a janela, sendo um unico envio autorizado da ferramenta em(1,72;1,81), timestamp1794422875,8101. Esse envio foi feito uma vez por /goal_pose_alinhado, passou pelo supervisor e terminou ABORTED/status6/codigo208. Resultado em resultado_goal.json. Nao houve retentativa automatica pela ferramenta.

Os comandos/avanco observados na janela nao podem ser atribuidos integralmente ao envio da ferramenta: ja havia goals do usuario antes dele, inclusive de retorno, e houve outro depois. Corrigida essa atribuicao em comunicacao com usuario. No bag ha489 comandos finais,363 nao zero, agregando todos os goals da janela. Transicoes e pose por comando nao sao prova de distancia percorrida, pois odometria nao tem encoders.

## 7. Defeitos de software ja corrigidos
Checagem DDS parcial de arbitro substituida por verificacao persistente com servico/parametros/endpoints; subida real confirmou arquitetura. Erro rclpy de severidade por callsite corrigido, testado e carregado na ultima subida; log do ultimo fracasso nao apresenta a excecao. Esses consertos nao resolveram registro do mapa, conectividade do costmap nem completaram travessia.

## Evidencias
- Bag MCAP finalizado: ensaio_goal_superior/ (aproximadamente27MB), metadata.yaml presente. Hidden action status nao foi gravado pelo rosbag; estados do envio proprio foram capturados separadamente em resultado_goal.json. Rosout e TF/scan/comandos foram gravados.
- Logs operacionais: ~/logs/11-11_153943/; anteriores em ~/logs/11-11_152704/.
- Nenhuma nova configuracao de SLAM, DWB, Collision Monitor ou mapa foi aplicada nesta rodada de diagnostico. Sem remapeamento automatico. Ferramentas Python novas nesta pasta sao artefatos de leitura/analise e o script de envio unico autorizado; nao devem ser usados para reenviar goals automaticamente.

## Encaminhamento proposto, sem execucao
Primeiro validar montagem/tempo do LiDAR e transformacoes contra IMU em poses fisicas conhecidas; comparar paredes/obstaculos atuais com mapa salvo. Se remapear, salvar com novo nome, preservar anterior e marcar START somente depois de confirmar pose/scan no mapa. Depois testar localization parada e em pequenos trechos, um goal/um bag, antes de habilitar transferencias e curva. Encerramento solicitado pelo usuario prevalece: nenhum desses passos foi iniciado.
