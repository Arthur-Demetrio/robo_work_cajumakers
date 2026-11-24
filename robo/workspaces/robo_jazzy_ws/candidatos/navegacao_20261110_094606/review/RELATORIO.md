# Implementação candidata — navegação antecipativa

10/11/2026. Diretório: `/home/marcos/robo_jazzy_ws/candidatos/navegacao_20261110_094606`.

## O que foi implementado agora

A candidata foi montada **a partir do src local atual de robo_jazzy_ws**, preservando alterações não commitadas e arquivos novos. O código experimental anterior em `/home/marcos/robo_mecanum_candidate` foi reaproveitado somente para extensões e ferramentas de teste; não substituiu a base local. Não se trata apenas de localizar os experimentos anteriores.

O backup anterior a qualquer modificação contém status com todos os não rastreados visíveis ao Git, HEAD, diff binário, tar e SHA-256 de 426 arquivos, verificados por leitura do tar. Local: `../../backups/nav_candidata_20261110_094606`. Os caminhos exatos constam em `origem.json`. O tar não inclui build/install/cache ignorados pelo Git. O snapshot da candidata inclui o src local, sem caches Python. Nenhum checkout/reset/clean/stash destrutivo foi usado.

Arquivos principais criados/modificados **somente na cópia candidata**:

| Arquivo relativo à candidata | Conteúdo comprovado |
|---|---|
| `src/robo_navigation/config/nav2_params.yaml` | `FollowPath.plugin=dwb_core::DWBLocalPlanner`, sem primary_controller ou parâmetros do Shim; global consolidado; GoalAlign 0; horizonte 1,6 s |
| `src/robo_navigation/config/nav2_params_dwb_candidate.yaml` | Etapa DWB direto com velocidades, acelerações, críticos e horizonte originais |
| `src/robo_navigation/config/nav2_params_global_shim_candidate.yaml` | Somente consolidação do global, ainda com Shim, para isolar a primeira mudança |
| `src/robo_navigation/config/nav2_params_{early,predictive}_candidate.yaml` | Etapas separadas dos ajustes de alinhamento e horizonte |
| `src/robo_navigation/behavior_trees/navigate_to_pose_candidate.xml` | Validade a 2 Hz, manutenção de caminho viável, planejamento condicional, smoothing com colisões, recuperação transacional e novo plano após recuperação |
| `src/robo_navigation/launch/navigation_candidate.launch.py` | Seleciona YAML principal **e** XML candidato no perfil padrão `candidate`; exige domínio 166, namespace e lifecycle inativo |
| `src/robo_recovery/robo_recovery/supervised.py` | Geometria, cobertura, volume varrido, decisão e orçamento de tentativas |
| `src/robo_recovery/robo_recovery/supervised_node.py` | Propriedade do comando, ação DriveOnHeading `supervised_escape`, parada, novas evidências e falhas travadas |
| `src/robo_recovery/setup.py`, `package.xml` | Executável e dependências da recuperação candidata |
| `tests/mecanum/test_delivery.py` | Preservação operacional, provas de YAML/BT/launch, portas dos plugins instalados e resultados scan/planner |
| `tests/mecanum/native_navigation/scan_replan.cpp` | Teste integrado real de LaserScan→camadas globais→Navfn, em memória |
| `tests/mecanum/truncate_probe/probe.cpp` | Execução do plugin real de recorte de caminho restante |
| `scripts/*.sh`, `scripts/report.py` | Build isolado, reprodução, relatório e consulta somente leitura |

`nav2_params.diff` mostra as diferenças entre o YAML local capturado e a candidata principal. `sha256_candidata.json` registra os arquivos centrais. Há referências ao Shim em comentários preservados do Collision Monitor; isso não é um plugin Shim configurado na candidata. A prova é o valor de FollowPath.plugin e a ausência de primary_controller, não uma busca textual indiscriminada.

## Preservação

O teste compara os hashes de todos os arquivos src presentes no snapshot operacional e de YAML/XML/Python no install operacional. Alinhador, identificação das WS, hardware, firmware e Collision Monitor não foram editados. Na candidata, o bloco inteiro do Collision Monitor continua semanticamente idêntico ao snapshot, assim como footprint, costmaps efetivas, velocidades, acelerações e velocity_smoother.

O alinhador copiado conserva suas modificações locais. Ele possui decisões próprias de giro antes do goal e uso de rota estática na preparação das etapas; a candidata não elimina esses comportamentos e não os altera. O launch experimental não o inicia; a composição operacional completa continua pendente de validação.

## Global costmap

Foi descartada a **primeira** declaração duplicada de obstacle_layer. Ela continha alcance 5,0/5,5 m; a última definição efetiva usa 3,5/4,0 m, marking/clearing e inf_is_valid ativos, tópico `/scan_nav`. Os demais parâmetros efetivos permanecem iguais. A fonte global ativa continua somente `scan`; a declaração de `bloqueio` permanece como estava, sem ser ativada globalmente. Plugins: static_layer, obstacle_layer, inflation_layer.

Prova além de inscrição: `review/scan_replan_result.log` registra:

```
configured_source=/scan_nav before_max_y=0 after_max_y=0.586224 unsafe_plan_cells=0
```

O teste usa o callback oficial LaserScan, buffer de observações, ObstacleLayer, InflationLayer e Navfn instalados. Um caminho inicialmente reto muda de passagem após nova observação. Nenhum sensor ou objetivo é publicado. O fundo estático desse teste é uma grade livre sintética; não reproduz o mapa da arena, atrasos de sensor/TF nem o processo Nav2 completo.

O teste isolado da camada também comprovou marcação e limpeza: `before=0 marked=254 cleared=0`. Não houve tentativa de inferir funcionamento atual do robô apenas da inscrição em `/scan_nav`.

## Planejamento antecipativo e mecanum

A árvore smart_replan local foi examinada. Foram aproveitados consulta a 2 Hz, goal atualizado, caminho restante e manutenção de rota válida. Não foram aproveitados RoundRobin, smoothing sem checagem, Spin incondicional ou BackUp de 30 cm a 0,15 m/s.

O recorte usa limite **positivo** de 1.000.000 m à frente e zero atrás; não assume que -1 seja um contrato suportado do plugin Jazzy. Teste real tickado, com pose fornecida e sem ações: `remaining_points=11 first_x=1 last_x=2`. A validade é consultada no trecho restante do caminho **suavizado que o FollowPath executa**. IsPathValid verifica custos de caminho; footprint, colisão durante controle e parada continuam dependendo da composição com DWB, smoothing e CM.

Caminho válido é mantido até expiração de 15 s; mudança do goal ou invalidez provoca novo planejamento antes disso. Não há troca de rota a cada consulta. Custos globais continuam a 2 Hz: o período de consulta de 0,5 s soma-se a atrasos de scan, TF e serviços. Essa política reduz replanejamento desnecessário por construção; sua taxa de trocas de passagem ainda não foi medida com a BT em execução fechada.

Etapas de controlador:

1. DWB direto: remove wrapper e nove opções exclusivas. Limites e críticos intactos.
2. GoalAlign 18→0: variante separada após contraprovas de outros pesos; RotateToGoal, goal checker, PathAlign e Twirling permanecem.
3. Horizonte 0,8→1,6 s: variante separada, sem aumento dos limites. É a configuração principal candidata.

A variante anterior já havia explorado esses pesos; esta execução repetiu as contraprovas e os cenários usando o novo snapshot. Horizonte maior sozinho não concluiu o contorno; zerar GoalAlign sozinho concluiu a caixa existente, mas não a dinâmica. PathAlign=0, PathDist=12 e ObstacleFootprint=1 não resolveram o caso de contorno testado. Não há garantia universal de melhora.

## Recuperação integrada

Somente após falha recuperável do planejamento/smoothing/controle a BT solicita `supervised_escape`. A política prefere lateral com continuação observada; avalia ré curta de até 10 cm quando há cobertura comprovada; exige parada e novos scan/pose após ré; permite giro apenas se necessário e autorizado; falha explicitamente quando não existe segunda alternativa. Não usa sucesso de publicação ou odometria por comando como prova de deslocamento.

O servidor limita transações, detecta ausência de progresso na manobra e mantém falhas travadas. Cancelamento, timeout, publicador conflitante e CM STOP não viram autorização para outra manobra. A árvore tem no máximo duas transações; após sucesso exige novo ComputePathToPose e SmoothPath com colisões antes de voltar ao seguimento. Limpezas automáticas de costmap foram removidas da árvore final para não apagar observações novas como efeito de falha de passagem.

Mudanças adicionais feitas nesta integração: caminho preservado deixou de ser tratado como heartbeat de 2 s; a navegação normal não precisa da certificação própria da recuperação. Ela continua usando o progress checker original, monitoramento de saída e CM. A supervisão independente extra de progresso normal é opt-in via `supervise_navigation_progress`; sua configuração padrão é false. Essa opção não libera manobras: cobertura e pose independente continuam obrigatórias na recuperação, e giro continua false por padrão. Sem evidência adequada, a ação de recuperação aborta e a navegação termina com falha; não promete escapar de qualquer caixa.

## Ensaios e limites

Build final: robo_navigation e robo_recovery concluídos em install_candidate. DWB principal carregado por pluginlib e configurado com o YAML principal; gerador real produziu amostras frontais, laterais para os dois lados e combinadas. Resultado registrado: 352 amostras, pure_left=4, pure_right=4, pure_front=20, diagonal=56, rear=0, máximos x=0,2 / y=0,08 / theta=0,16. A ausência de ré no DWB preserva min_vel_x=0. O teste adicional com o YAML principal em `main_lateral.json` configurou o controlador real e concluiu a lateral em 0,8 s sem rotação; a etapa DWB direto isolada fez 0,7 s. A tolerância XY preservada permite concluir um goal lateral de 10 cm após aproximadamente 4–5 cm; esse teste não prova precisão de deslocamento de 10 cm.

A BT foi verificada contra os tipos registrados e as portas declaradas pelos plugins instalados. O nome da ação DriveOnHeading é compatível com o servidor candidato; callbacks de sucesso, falha, cancelamento, parada e reavaliação foram testados com entradas simuladas. Isso **não é execução completa da BT com clientes de ação e todos os nós**.

A suíte final teve **72 testes aprovados**, registrados em `pytest_delivery.log` e `pytest_delivery.xml`. A conferência final também verificou novamente os hashes dos 426 arquivos originais, sem diferenças (`preservacao_final.json`). A prova extraída do YAML/XML entregues está em `PROVA_CONFIGURACAO.json`. A matriz executou 36 casos de quatro perfis e seis contraprovas de críticos/horizonte. Resultados completos em `COMPARATIVO.md` e `antecipacao/native_comparison.json`.

| Medição no modelo ideal | Referência Shim | Candidata preditiva |
|---|---:|---:|
| Pequena lateral: duração | 12,5 s | 0,8 s |
| Pequena lateral: yaw intermediário | 1,48 rad | 0 rad |
| Caixa existente com novo plano | Não concluiu em 30 s | 23,1 s |
| Caixa dinâmica com novo plano | Não concluiu em 30 s | 27,2 s |
| Caixa dinâmica: sem translação | 9,3 s | 1,4 s |
| Caixa dinâmica: yaw intermediário | 3,312 rad | 0,827 rad |
| Caixa dinâmica: primeira lateral | x=0,185 m | x=0,20 m |

O primeiro lateral não foi mais cedo em x que a referência no caso dinâmico; o ganho demonstrado é concluir o contorno com menos parada/rotação, não antecipar mais que o Shim em todo cenário. Na candidata, x=0,20 deixa aproximadamente 0,285 m entre a frente do footprint e o início da caixa naquela geometria. Isso não é distância física de segurança certificada.

Não houve interseções geométricas nos nove ensaios preditivos. Caixa próxima e quina continuam sem conclusão; alguns ensaios de referência apresentaram interseções geométricas no modelo. A preditiva também foi mais lenta que outras variantes em área livre/yaw final. Não ocultar esses resultados negativos. Recuperações não são medidas no laço ideal, que não executa a BT; zero ações nele não é taxa de recuperação comprovada na composição integrada.

O P95 de cálculo observado na candidata chegou a ~160 ms em um cenário durante execução concorrente de testes, acima dos 100 ms de um controlador a 10 Hz. O ensaio não é benchmark isolado da CPU do robô; latência e carga precisam ser medidas antes de liberação. Nenhuma frequência/velocidade foi alterada para esconder esse limite.

Duas bags foram lidas sem replay: frontal histórica (164 globais/363 locais/407 scans amostrados) e lateral WS de 09/11 (1165 globais/2539 locais/2831 scans). A segunda tem até 60 células globais recém-ocupadas versus grid inicial, mas nenhuma coincidência amostrada de retorno com marcação global nova. Isso não atribui causa nem prova ausência de camada; mudanças de mapa/pose também afetam essa comparação. Dados integrais em bag_frontal.json e bag_lateral.json.

A restrição de sockets DDS deste terminal aparece nos logs das sondas, que terminaram os cálculos em memória. Não foi executado Gazebo ou navegação física nesta entrega. Reprodução fechada com todas as costmaps, BT, suavizadores, CM, sensores atrasados, recuperação e alinhador preservado continua pendente; não há alegação de aprovação operacional por testes unitários.

## Instalação, verificação e reversão

Procedimentos e comandos exatos em `../README_CANDIDATA.md`. O instalador já foi executado apenas na candidata. O lançamento seleciona YAML e XML explicitamente e não inicia os componentes protegidos. O install operacional permaneceu intacto. Não basta substituir YAML no launch legado, que fixa navigate_to_pose_smooth.xml.

Estado entregue: código/configuração/árvore integrados e instalados no overlay candidato, testes executados, prova de novos obstáculos chegando ao planner em memória e preservação verificada. **Validação integrada completa e liberação física ainda não concluídas.**
