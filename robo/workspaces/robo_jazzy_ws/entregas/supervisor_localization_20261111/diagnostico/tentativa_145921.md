# Tentativa 11/11 14:59 — leitura dos logs

Fonte: ~/logs/11-11_145921/{slam,nav2,mapa,map_odom}.log.
SLAM: async_slam_toolbox_node, mapping, deserialize match_type=2, arena_04-11_1235, START solicitado (0.192,0.004,-0.171). Logs posteriores registram SIGINT e encerramento. Nao houve restart executado por esta analise.
Nav2: subida via subir_robo.sh sem nav2_principal:=true, com remapeamentos cmd_vel_nav legados, sem processo arbitro_nav_principal. Nao constitui ensaio do supervisor novo.
Curva: juncao estimada (1.49,-0.18), lado esquerdo, rotina lateral. Travessia aborta duas vezes por retorno frontal a 9 cm antes de chegar a rotina lateral. Registra recuos calculados em odom de 2.14 m e 1.12 m; nao equivalem a deslocamentos fisicos medidos.
Segunda solicitacao: plano falha; preparo entregue a NavigateToPose. Mais tarde, CM para e controlador falha por ausencia de progresso. Destravar recusa giro por 13 retornos LiDAR na area varrida. Isso e decisao do modelo/sensores, nao prova de ausencia de espaco fisico para a curva. Sem scan e TF/IMU simultaneos gravados nao e possivel identificar quais retornos nem separar pose errada, geometria/footprint e obstaculo real.
Nao gravar START a partir de pose sabidamente divergente. Necessario distinguir mapa novo na posicao atual de recarga do mapa existente com relocalizacao. Preservar posegraph e pontos existentes. Acesso DDS desta ferramenta bloqueado; nao foi feita mudanca em pose, mapa, sensores ou comandos.
