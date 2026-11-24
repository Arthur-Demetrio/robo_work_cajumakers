# Partida do arbitro e validacao ROS real

## Causa observada
A subida 11-11_151919 iniciou arbitro pid55559, alinhador no perfil principal e Nav2 ativo. Os parametros foram lidos corretamente. Logo depois, uma consulta `ros2 node list --no-daemon` retornou lista parcial, sem varios nodes que estavam ativos, e o script declarou arbitro ausente. O traceback do arbitro aparece so depois do SIGINT enviado pelo encerramento da subida, nao como causa da ausencia. Nao ha evidencia de crash anterior do arbitro. A falha foi na verificacao de descoberta.

## Alteracoes
- ~/subir_robo.sh usa conferir_supervisor_ativo.py em vez de consultas CLI independentes e instantaneo de lista. Probe inicial esperada de ausencia de SLAM antes da subida deixou de exibir mensagem de erro espuria.
- Novo verificador usa um unico contexto rclpy e espera ate 25s. Confirma servico /navegacao/especial (sem chama-lo), GetParameters nav2_principal=true e supervisor_modo correto, publisher heartbeat do alinhador e exatamente um publisher de /cmd_vel_nav, pertencente ao arbitro. Exige 1,5s estavel. Falta/mismatch/duplicidade continua sendo falha. Salva JSON com evidencia/pendencias.
- Arbitro registra prontidao e trata SIGINT/ExternalShutdownException; nao tenta publicar parada se contexto ja foi invalidado. Autoridade inicial bloqueada, lease e filtro temporal intactos.
- abrir_robo_supervisor.sh abre terminal grafico e mantem shell aberto apos erro/encerramento; variaveis de referencia a terminal antigo removidas. Sem flags usa localization/arena_04-11_1235. Nao envia goals.
- AGENTS/CLAUDE atualizados; fixture existente recebeu get_logger para nova linha de log do arbitro, mantendo assercoes de lease/filtragem.
- Nao alterados parametros SLAM, DWB, CM, hardware, regioes, posegraph ou START.

## Testes
Com DDS agora permitido, teste ROS em dominio171/LOCALHOST executou arbitro real e alinhador passivo de teste. Descoberta real aprovada; pelo menos dois comandos observados e todos zero; segundo publisher recusado; retirada do arbitro detectada; SIGINT retornou0 sem traceback. Nenhum hardware conectado nesse dominio, nenhum SetBool chamado.

Selecao inicial: 31 passaram/1 fixture falhou (fake Node sem get_logger). Suite nova tem 10 testes aprovados nessa execucao: politica fail closed, integracao shell simulada com verificador recusando e ROS multiprocesso real. Depois de corrigir fixture, coordenador: 22 passaram em2,18s. Total de32 testes distintos aprovados entre as duas selecoes. Logs testes.log e testes_coordenador.log. py_compile, bash-n e diff-check executados.

## Subida operacional autorizada
Usuario liberou acesso e solicitou corrigir o necessario apos stack encerrar. Antes da subida, grafo operacional continha apenas RViz/listener; verificador registrou falta completa de supervisor/arbitro em antes.json. Arquivos pontos/service_areas preservados em dados_antes/ antes de o script executar suas copias habituais.

Subida real: ~/logs/11-11_152704, processo inicial56660. Terminal principal aberto via wrapper e cinco terminais somente de logs (hardware, LiDAR, odometria, SLAM, Nav2). Nao ha segunda instancia ROS por esses terminais.

Tanto o checker da propria subida (~/logs/11-11_152704/supervisor_ativo.json) quanto checker independente (validacao_ativa.json) confirmaram sucesso: principal=true, observar, servico presente, publisher unico /arbitro_nav_principal e heartbeat anunciado pelo alinhador. Lifecycle Nav2/CM ativo. Consulta real de parametros SLAM retornou localization e /home/marcos/mapas/arena_04-11_1235.

Amostragem somente leitura por10s em diagnostico_parado.json: 199 mensagens /cmd_vel_nav, todas zero; nenhuma mensagem observada em /cmd_vel final (nao afirmar confirmacao de velocidade mecanica ou comandos finais a partir dessa ausencia); 19 amostras map->base em x0,34385/y-0,08748/yaw0,65012 graus, constantes; 193 IMUs com yaw relativo0. Nao houve goals ou velocidades publicados pela ferramenta. Arbitro zerado nao e prova de localizacao absoluta correta. Nenhum novo START salvo e nenhuma coordenada foi inventada a partir de screenshot.

## Limites e comandos
Supervisor esta ATIVO EM OBSERVAR. Aplicar nao foi habilitado: regioes ainda vazias e verificacao anterior do wrapper deve recusar --supervisor-aplicar. Isso nao e validacao de travessia ou curva, nem teste fisico. Nao afirmar estabilidade SLAM em movimento com amostra parada.

Para nova abertura futura (nao executar enquanto instancia atual estiver ativa):
`bash ~/robo_jazzy_ws/ferramentas/abrir_robo_supervisor.sh --localization arena_04-11_1235`

Reverificacao somente leitura:
`python3 ~/robo_jazzy_ws/ferramentas/conferir_supervisor_ativo.py --modo observar`
(carregar /opt/ros/jazzy/setup.bash antes, se necessario).

Rollback de arquivos: `python3 entregas/partida_arbitro_20261111/patch.py --desfazer`. Recusa edicoes posteriores, preserva backups/evidencias e nao interfere em processos vivos. Desfazer esta rodada antes de correcao_subida_supervisor_20261111. Aplicar/desfazer em copia via --arquivo DIR. Fechar terminais de log nao encerra stack; Ctrl+C no terminal principal encerra os grupos iniciados pelo script.
