# Goal superior e logger — 11/11

## Sequencia do log
Goal do usuario(1,57;2,31), aceito em1794421914.221. NavFn falha antes disso em1914.212 e novamente em1914.260, apos limpeza de costmap. Origem(0,34;-0,09). Nao houve caminho global novo desse envio. Recuperacao diagnostica planner sem caminho; candidatos laterais nao tem continuacao, por isso nao executa deslocamento. ABORTED em1917.169. Nenhuma transicao TRANSFERENCIA/ALINHADOR aparece. Outra tentativa em seguida apresenta mesmo erro de logger.

Supervisor estava observar. Esse modo nao concede controle ao alinhador. Alem disso, detector exige caminho fresco do goal atual; nao pode reutilizar plano de outro goal nem inventar travessia quando planner nao encontra rota. Regioes de passagem continuam vazias: aplicar nao foi habilitado.

## Defeito corrigido
_log_nav_principal usava getattr(logger,nivel)(texto) na mesma linha. rclpy registra severidade por callsite, entao info de envio/aceitacao seguido de warn de resultado provocava ValueError Logger severity cannot be changed between calls. O erro mascara o registro do resultado original, mas nao causa o primeiro fracasso do planner (que ocorre tres segundos antes).

Chamadas info/warning/error agora sao explicitas em linhas distintas, com rejeicao de nivel desconhecido. Sem alteracao de planejamento, transferencia, sensores, SLAM, DWB ou CM. Correcao no src, nao carregada retroativamente pelo processo ativo. Nao reiniciado stack ou alinhador nesta rodada; nao foram enviados goals/velocidades nem chamadas de limpar costmap pela ferramenta.

## Snapshot real somente leitura
inspecionar_costmap.py usa GetCostmap (leitura), /map e TF. snapshot.npz e conectividade.json preservam dados apos a falha; nao e bag do instante da tentativa.

- Resolucao0,05m; global214x219; origem(-3,40435;-4,24019).
- Inicio(0,34385;-0,08748), celula(74;83), custo87. Goal(1,57;2,31), celula(99;131), custo0.
- BFS8 nas celulas do costmap com custo<253, incluindo teste com255 permitido: inicio/goal desconectados; componente inicial689 celulas.
- No mapa bruto, ambos custo0 e conectados. Modelo offline por distancia a ocupacao continua conectado com margem circular0,215/0,315/0,325m. Nao e validacao do footprint real, curva nem manobra mecanum.
- No costmap ha82 celulas letais254 em celulas livres do mapa e9422 celulas de custo253 em mapa livre. Isso indica obstaculos adicionais/zonas de inflacao na composicao; nao permite chamar os retornos de falsos sem scans correspondentes.
- /map tem um publisher(slam_toolbox); global_costmap esta inscrito com QoS transiente e static_layer habilitada/subscribe_to_updates=true. Nao encontrada evidencia de dois mapas publicando.
- Figura mapa_costmap.png compara os grids; areas desconhecidas do mapa bruto aparecem sem preenchimento e255 no costmap esta na extremidade alta da escala, portanto cor clara nao significa somente obstaculo.

## Testes e pendencias
47 testes aprovados em4,79s (testes.log): coordenador, supervisor e duas regressoes do logger real. Uma reproduz o erro da implementacao antiga; outra alterna info/warn/error/info/warning/error pela implementacao real corrigida sem excecao. py_compile e diff-check executados.

Ainda falta correlacionar os retornos que fecham o costmap com /scan_nav, pose e IMU no instante do problema, cadastrar regioes medidas e validar aplicar. Nao afrouxadas margens para forcar passagem. A conclusao nao e que falta espaco fisico: e que Nav2 nao recebeu uma conexao transitavel no costmap atual. Observar nao e teste de travessia pelo alinhador.

Rollback desta rodada: python3 entregas/goal_superior_20261111/patch.py --desfazer. Verifica hashes antes de restaurar, preserva evidencias/backups e nao toca em processos vivos. Executar antes de rollbacks anteriores. Para comparar patch em copia usar --arquivo DIR. Mudar arquivo nao corrige o logger do processo que ja estava aberto; sera necessario recarrega-lo em proxima subida autorizada.

## Recarga autorizada posterior

Usuario autorizou reiniciar terminais. Subida antiga PID56660/grupo56658 encerrada por SIGINT; grupos Nav2/hardware/SLAM anteriores terminaram antes de nova instancia. Subida nova em ~/logs/11-11_153943, mapa salvo localization, principal=true, observar e arbitro unico confirmados por checker da subida e checker independente. Alinhador novo PID58802 iniciou depois do mtime da correcao e executa src direto, SHA256/tempos em recarga/processo.json. Terminais de logs antigos fechados e cinco monitores apontam para a nova subida. Nao enviados goals nem comandos de movimento, nao alterado START. A recarga carrega correcao do logger; nao resolve desconexao do costmap nem habilita aplicar.
