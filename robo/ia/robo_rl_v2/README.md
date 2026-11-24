# Robo-WORK RL V2 — treinamento independente em simulação

Projeto isolado em `~/robo_rl_v2`. Não usa ROS, não publica comandos e não chama OpenAI, Codex ou qualquer serviço durante episódios. O treinamento é um processo Python local, com PPO/Stable-Baselines3 em CPU.

## Visualizar enquanto o treinamento continua

```bash
cd ~/robo_rl_v2
./visualizar_treino.sh
```

Janela separada de **avaliação dos checkpoints**, com arena real, robô, LiDAR, decisões e métricas. Espaço pausa, R reinicia, N troca cenário e C aplica um novo checkpoint pendente. A velocidade é ajustável; por padrão usa uma thread, um CPU e orçamento aproximado de 20% de um núcleo. Não interrompe nem escreve no treinamento.

[Controles, atualização automática e validação do visualizador](docs/VISUALIZADOR.md). Execute no terminal da sessão gráfica; o executor Codex não consegue acessar o display local.

## Começar agora, no terminal Linux normal

O ambiente Python já foi preparado neste computador. Não precisa reinstalar para executar.

```bash
cd ~/robo_rl_v2
./iniciar_treino.sh
./acompanhar_treino.sh
```

O início retorna quando o Python está pronto. Pode fechar o terminal e o Codex. O script usa `start_new_session` (equivalente a `setsid`), ignora SIGHUP, redireciona entrada para `/dev/null` e escreve a saída em arquivo. Não depende de tmux instalado. Fechar a sessão gráfica que mata processos do usuário, suspender ou desligar o computador interrompe o treino; retome do checkpoint depois.

Configuração padrão: 2 milhões de transições, 2 CPUs permitidos, 2 threads PyTorch, prioridade nice 10, checkpoints a cada 10 mil passos, avaliações a cada 25 mil, 3 episódios por cenário. CPU em execução não significa consumo de tokens: nenhum modelo de linguagem participa do treinamento.

```bash
# Reduzir carga; use diretório novo para uma experiência nova
./iniciar_treino.sh --run runs/economico --cpus 1 --threads 1 --steps 2000000

# Acompanhamento pontual e log ao vivo (Ctrl+C aqui fecha só o tail)
./acompanhar_treino.sh
tail -f runs/principal/terminal.log

# Interromper com salvamento seguro
./parar_treino.sh
./acompanhar_treino.sh

# Retomar automaticamente o modelo apontado por latest.json
./continuar_treino.sh

# Aumentar a meta total, preservando o currículo original
./continuar_treino.sh --steps 4000000

# Avaliação manual: prefere best.zip, senão último checkpoint
./avaliar_modelo.sh --episodes 10

# Gráficos após haver episódios completos
./relatorio.sh
```

Para qualquer execução com `--run`, repita esse argumento nos outros scripts. `--steps` é meta total acumulada, não passos adicionais. A retomada sem `--steps` usa a meta original. PPO completa blocos de rollout, portanto pode ultrapassar a meta em até `n_steps-1` transições. O currículo muda no próximo reset de episódio.

Parar solicita SIGTERM apenas ao PID correspondente ao comando e instante de criação registrados. Aguarde `process.json` desaparecer e `status.json` indicar `stopped: true`; durante uma avaliação, a parada pode aguardar seu término. Evite `kill -9`: ele perde o trabalho desde o último checkpoint, mas preserva os anteriores. O script não reinicia sozinho após reboot; `continuar_treino.sh` faz a retomada.

## Resultados e visualização

- `docs/arena_real.png`: raster do mapa importado, sem caixas sintéticas adicionais.
- `docs/arena_ws_registradas.png`: anotações do arquivo associado ao mapa; não são novas medições.
- `docs/level_1.png` a `level_5.png`: exemplos dos níveis. Caixas sintéticas aparecem em marrom, separadas do mapa medido.
- `runs/<nome>/episodes.csv`: métricas dos episódios de treino.
- `runs/<nome>/optimizer_*/progress.csv`: perdas e métricas PPO.
- `runs/<nome>/evaluations/<passo>/`: CSV, resumo JSON, PNGs, trajetórias NPZ e gráfico de taxas.
- `runs/<nome>/checkpoints/`: modelos periódicos com pesos e estado do otimizador.
- `latest.json`: ponteiro atualizado atomicamente após gravação completa do checkpoint.
- `best.zip` e `best.json`: melhor média de sucesso menos colisão nos cenários de validação.
- `runs/smoke_final/REPORT.md`: relatório do treino curto de verificação.

```bash
# Reproduzir um episódio como animação GIF, sem conexão externa
.venv/bin/python -m robo_rl_v2.replay \
  runs/smoke_final/evaluations/1024/level_4_episode.npz \
  --output runs/smoke_final/arena_episode.gif

# Avaliar explicitamente um checkpoint, inclusive em diretório separado
.venv/bin/python -m robo_rl_v2.evaluate \
  --model runs/principal/best.zip \
  --output runs/teste_final --episodes 30
```

O modelo acima existe após a primeira avaliação. Para avaliar o último checkpoint em vez do melhor, use o caminho registrado em `latest.json`.

## TensorBoard

O treinamento grava escalares no formato Event protobuf/TFRecord em `runs/<nome>/tensorboard`, com CRC32C. Esse escritor local foi necessário porque TensorBoard não está instalado e a rede do executor não permite baixar pacotes. Ele não requer TensorFlow ou internet para registrar métricas. O esquema protobuf e CRC foram testados; a interface web do TensorBoard não pôde ser executada aqui.

Para abrir o painel, instale TensorBoard uma vez quando houver rede:

```bash
.venv/bin/python -m pip install tensorboard
.venv/bin/python -m tensorboard.main --logdir runs --host 127.0.0.1 --port 6006
```

Acesse `http://127.0.0.1:6006`. CSV, PNG, JSON e GIF funcionam sem essa instalação.

## Arena real e limites conhecidos

Foi inventariado `~/mapas`, incluindo `backup_2210`: 8 pares YAML/PGM. O mais recente disponível como raster é `arena_04-11_0917`: resolução **0,05 m**, origem **[-5,426; -2,729; 0]**, **208 × 219 células** (10,40 × 10,95 m de extensão raster, incluindo desconhecido). Tem 2.343 células ocupadas, 18.920 livres e 24.289 desconhecidas. A extensão do raster não é uma medida de uma sala retangular.

Há `arena_04-11_1235.posegraph/.data`, mais recente e usado no fluxo atual do robô, mas não há seu YAML/PGM correspondente em `~/mapas`. Não convertemos esse posegraph por ROS e não apresentamos o raster 09:17 como mapa atual exato. O mundo importado preserva paredes, corredores e lacunas do raster; não fecha buracos por estimativa. Desconhecido e exterior são bloqueados para colisão e retornos LiDAR. As duas categorias continuam separadas no armazenamento e visualização.

A origem yaw é aplicada em SE(2), inclusive quando diferente de zero. Os limiares `occupied_thresh`, `free_thresh` e `negate` são lidos do YAML. O carregador recusa modos diferentes de trinary, em vez de interpretá-los silenciosamente. `docs/map_inventory.json` guarda metadados e hashes.

O arquivo `service_areas_arena_04-11_0917.yaml` foi preservado com o mapa correspondente e suas faces são anotadas. Os arquivos de WS atuais têm coordenadas divergentes e pertencem a revisões posteriores; não foram misturados. Larguras registradas são históricas, profundidades/alturas padrão não foram consideradas medidas verificadas. Nenhum retângulo de bancada foi inventado a partir desses defaults; a colisão física das estações é apenas aquela presente no raster. Para atualizar, exporte futuramente um YAML/PGM do mapa desejado e forneça `map_path` ao ambiente, com anotações compatíveis.

## Robô e observações

Footprint completo `[[.365,.315],[.365,-.315],[-.205,-.315],[-.205,.315]]`, LiDAR `[.33,.005]`, 72 raios a 6,25 Hz. O deslocamento lateral do sensor vem da V1; a posição aproximada e os setores de oclusão ainda precisam de calibração física. Os intervalos bloqueados são copiados do filtro LiDAR ativo. A aplicação em ângulos de `base_link` é uma hipótese explícita; não reconstitui extrínsecos 3D nem garante a equivalência do `invert_angles` do filtro real.

Ações contínuas normalizadas `(vx, vy, wz)` permitem frente, ré, lateral, diagonais e rotação simultâneas. Referência: configuração copiada de `~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml` (workspace identificado no script de inicialização). Limites do controlador: 0,20 m/s frontal, 0,08 lateral, 0,16 rad/s angular, norma translacional ≤0,20 m/s; ré até 0,16 m/s vem do velocity_smoother, pois o DWB atual restringe ré. Aceleração `[.2,.2,.4]`, desaceleração `[.5,.5,.2]`. São parâmetros configurados, não uma identificação dinâmica dos motores.

Há quatro ticks de dinâmica por ação, atraso de comandos, ganhos de atrito, perda lateral por escorregamento e ruído. Sem encoders: a pose estimada integra comandos atrasados com viés; velocidade física nunca é observação. Esse modelo é aproximado, não um modelo de rodas/PWM nem uma reprodução calibrada da localização LiDAR real.

Observação: 72 distâncias normalizadas, 72 flags válidos, vetor ao alvo em pose estimada, último comando e taxa de odometria estimada. Leituras bloqueadas são zero com máscara zero. A observação não contém pose verdadeira, mapa, caixas, distância verdadeira ou velocidade física. `info` e recompensas contêm métricas privilegiadas que PPO não recebe como observação. O sensor de destino pressupõe disponibilidade de localização estimada no referencial do mapa.

Colisão: SAT do polígono completo contra cada célula bloqueada candidata. Para cada intervalo, o footprint é expandido por um limite superior de deslocamento translacional + raio × rotação; isso cobre poses intermediárias, inclusive giros rápidos, com possível rejeição conservadora perto de paredes. Contato conta como colisão. Não usa somente ponto central nem apenas vértices.

## Currículo e recompensa

1. 0–15%: sala sintética aberta, objetivos e orientações variados, sem caixas.
2. 15–35%: caixa randomizada entre início e alvo, diferentes aproximações.
3. 35–55%: múltiplas caixas, inclusive próximas às paredes.
4. 55–80%: raster real com caixas sintéticas adicionais e destinos variados.
5. 80–100%: salas sintéticas novas, dimensões/caixas variadas, maior ruído, atraso e variação de atrito. Avaliação adiciona layouts procedurais não utilizados no treino.

Partidas e destinos são amostrados em um mesmo componente com folga conservadora para o footprint. A checagem de conectividade é mais restritiva que o necessário para um robô orientável e pode excluir passagens reais estreitas. Não há rota fixa. A progressão é por orçamento de passos, não por competência comprovada; taxas ruins justificam mais treino/ajuste antes de qualquer conclusão.

Recompensa usa progresso euclidiano **verdadeiro**, chegada, custo de tempo, comprimento, proximidade conservadora a obstáculos, giros, inversões de comando e janela sem progresso. Colisão −100, chegada +100; progresso ×15. O erro de odometria não produz recompensa de progresso. Progresso euclidiano pode dificultar desvios que inicialmente afastam do alvo: é uma escolha de baseline a avaliar, não uma solução garantida.

## Avaliação e reprodução

A cada 25 mil passos, usa sementes fixas de validação, separadas das sementes do treino, em níveis 1–5 e no mapa histórico de 22/10 reservado. O mapa histórico é um recorte diferente da mesma arena, não prova generalização a outro prédio. Os layouts sintéticos de validação são diferentes dos layouts de treino.

Como esses cenários selecionam `best.zip`, são validação, não teste final intocado. Para uma conclusão final, altere o conjunto de sementes no avaliador ou use um mapa realmente novo e mantenha-o fora da seleção do melhor modelo.

Registra sucesso, colisão, tempo, comprimento físico, tempo parado (<0,015 m/s), rotação absoluta acumulada, inversões de comando e recuperações. Recuperação significa sair de uma janela de 32 ações com deslocamento líquido <8 cm; não é o recovery do Nav2. Tempo de conclusão apenas dos sucessos vem em campo separado; nenhum sucesso gera null. PNGs, CSVs e NPZ permitem auditar trajetórias. Não há comparação com DWB em condições equivalentes; nenhuma superioridade é alegada.

CPU, seed, hiperparâmetros, currículo e dependências estão registrados. Retomada restaura pesos, otimizador e contador; reinicia episódio e fluxo aleatório. Não é continuação bit a bit: rollout parcial e estado físico/RNG do episódio anterior não são restaurados. Uma interrupção abrupta pode perder até o intervalo de checkpoint, e as amostras de um rollout incompleto não são reaproveitadas. `torch.use_deterministic_algorithms(True)` é usado em CPU; igualdade entre outras plataformas não é garantida.

## Testes e instalação em outro computador

```bash
cd ~/robo_rl_v2
.venv/bin/python -m unittest discover -s tests -v
# Integração de início/parada/retomada; use diretório de teste ainda inexistente
.venv/bin/python tests/integration/lifecycle.py
```

A V1 está em `referencia/robo_rl_2d.zip` e `referencia/v1_workspace.tar.gz`, incluindo o modelo e resultados que já existiam. A V1 passou em 13 testes e no check_env. A V2 usa pacote separado e não tenta carregar modelos V1, pois ações e observações mudaram.

O `.venv` local aproveita pacotes já disponíveis e foi tornado independente do diretório V1. Usa NumPy/SciPy do sistema para compatibilidade binária, sem alterar pacotes do sistema. `docs/runtime.json` registra versões efetivas. Em outra máquina Linux com Python 3.12:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Isso exige rede somente na instalação. Em produção, não mova/remova o `.venv` preparado enquanto treina. Consulte `docs/VALIDACAO.md` para evidências e limitações da verificação desta entrega.

Referências de API: [Gymnasium Env](https://gymnasium.farama.org/api/env/), [SB3 custom environments](https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html). Formato de eventos: [Event](https://github.com/tensorflow/tensorflow/blob/master/tensorflow/core/util/event.proto), [Summary](https://github.com/tensorflow/tensorflow/blob/master/tensorflow/core/framework/summary.proto).
