# Validação realizada em 09/11/2026

## Estado da entrega

Implementação em `/home/marcos/robo_rl_v2`, isolada de ROS. Treinamento principal preparado, **não iniciado**. O usuário inicia por `./iniciar_treino.sh` em terminal Linux normal. Nenhum comando físico foi publicado.

## Evidências

- V1: toda a árvore de fontes, scripts, testes e documentos examinada; 13 testes unittest passaram antes de implementação. `stable_baselines3.common.env_checker.check_env` passou na V1. ZIP original e snapshot do diretório com resultados/modelo existentes preservados em `referencia/`.
- V2: **13 testes passaram**, em `runs/tests_final.log`. Incluem os checkers Gymnasium e SB3, todos os níveis, semente determinística, mapa com origem rotacionada, thresholds e desconhecido, footprint com célula interior, sweep de translação e rotação, odometria adulterada sem vantagem na recompensa, atraso/aceleração, máscara e truncamento. Há um decoder protobuf independente para os eventos TensorBoard, além do teste CRC32C.
- Treino curto final: PPO **1.024 transições**, `runs/smoke_final/status.json`, 10 episódios completos, checkpoints em 256/512/768/1024 e avaliação automática nesses mesmos quatro marcos. Cada avaliação usa seis cenários, um episódio por cenário, limite reduzido a 100 ações para verificar o pipeline. O padrão principal é 1.500 ações e avaliação a cada 25 mil transições.
- `runs/smoke_final/REPORT.md`: na avaliação final, **0/6 sucessos, 0/6 colisões, 6/6 timeouts**. Isso verifica a execução, não aprendizado de navegação. Um episódio por cenário não estima qualidade estatística. Não há comparação com DWB.
- `runs/lifecycle_verified/lifecycle_result.json`: launcher retornou enquanto Python treinava em sessão Linux própria; afinidade restrita a um CPU. Parada segura em **264 transições**, retomada do checkpoint e conclusão em **1.288** para meta acumulada 1.280. O excedente é o bloco de rollout PPO. `latest.json` e modelos periódicos foram preservados.
- Avaliador manual pelo script solicitado, PNGs, CSVs, resumo JSON, gráficos e replay GIF executados em `runs/smoke_final`.
- Os seis scripts passaram em `bash -n`; módulos Python passaram na compilação.
- Snapshots de Nav2 e filtro LiDAR copiados para assets continuam byte a byte iguais às fontes ROS lidas. Não houve escrita em workspace ROS, firmware, BTs, collision monitor ou ferramentas de estações.

## Limitações da verificação

O executor de comandos do Codex usa namespaces PID temporários. Não é possível deixar um treinamento duradouro nesse executor: seu ciclo de vida encerra processos. A integração de início/parada/retomada foi validada dentro de um único namespace, com launcher encerrado e Python independente dele. O uso definitivo precisa começar em terminal Linux do usuário. Os scripts não dependem de manter Codex aberto.

Não há tmux instalado. O mecanismo equivalente implementado usa sessão Linux separada, stdin fechado, stdout/stderr em arquivo, SIGHUP ignorado, prioridade nice e afinidade de CPU. Não sobrevive a desligamento/suspensão ou a políticas de encerramento dos processos do usuário no logout; o checkpoint permite retomar após isso.

TensorBoard não está instalado e o pip não conseguiu resolver a rede. Os arquivos escalares foram gravados no formato compatível e decodificados com protobuf/CRC, mas a interface TensorBoard não foi aberta nesta máquina. A instalação opcional do painel está no README. O restante funciona sem rede.

As dependências locais de PPO foram copiadas para o venv V2, sem alterar a V1. NumPy 1.26.4 e SciPy do sistema são utilizados por compatibilidade binária. Há um aviso de Axes3D devido à mistura de pacotes matplotlib do sistema/venv; os gráficos 2D e GIF foram gerados normalmente. As versões estão em `runtime.json`.

O mapa raster importado é de 04/11 09:17; há posegraph posterior de 12:35 sem YAML/PGM. O mapa é parcial, com desconhecido bloqueado, e as estações atuais não foram transportadas entre referenciais sem confirmação. A máscara LiDAR e os parâmetros de ruído, atrito, atraso e odometria não constituem calibração do hardware.

Retomada restaura pesos, otimizador e contagem; um novo episódio começa. Não há restauração bit a bit do rollout/RNG anterior. A progressão de currículo é por orçamento e não por taxa de sucesso. Não há garantia de convergência ou de transferência ao robô real.

## Arquivos para inspeção

- `arena_real.png`, `arena_ws_registradas.png`, `level_*.png`.
- `map_inventory.json`: oito mapas disponíveis, metadados, contagens e hashes.
- `asset_manifest.json`, `delivery_manifest.json`: hashes de assets e código entregue.
- `../runs/smoke_final/training_metrics.png`, `evaluation_history.png`, `arena_episode.gif`.
- `../runs/smoke_final/evaluations/1024/summary.json` e `episodes.csv`.
- `../runs/lifecycle_test.log`.
