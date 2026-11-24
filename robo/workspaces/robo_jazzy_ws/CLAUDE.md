# Robô RoboCup@Work Entry Level — ROS 2 Jazzy

Robô mecanum para a competição de **23/11/2026** (CBR @Work Entry Level). Meta: missão de ~4 min com navegação limpa: sem tocar parede, fita ou obstáculo, e sem ajuda humana.
Usuário: Marcos (Recife). Falar sempre em **português do Brasil**.

## Como trabalhar (combinado com o usuário)
- **Uma mudança por vez, um bag por teste.** Não juntar mudanças que não dá para separar no log.
- **Analisar como operador:** antes de propor uma correção, pensar "se eu estivesse controlando, o que faria aqui?" (decidir, comprometer-se e executar até o fim, não ciscar).
- **Backup antes de mudar arquivo.** Mudança de config/launch/árvore vai em **script de patch** com:
  - uma **marca** única no arquivo (ex.: `# NAV_FRENTE (05/11): era 0.16`);
  - recusa de aplicar duas vezes (achou a marca → não faz nada);
  - conferência antes de gravar (yaml/xml carregam e só a chave pretendida mudou);
  - `--desfazer`, que volta o backup mais recente (`<arquivo>.bak_<nome>_<data>`);
  - testar o patch numa cópia antes de entregar (`--arquivo <copia>`).
- **Comandos prontos para colar.** Arquivos baixados: achar com `grep -l "<MARCA>" ~/Downloads/<nome>*.py | head -1`.
- **Afirmar só o que o bag mostra.** Medir antes de concluir (número, horário, comparação com missão anterior). Quando for hipótese, dizer que é.
- Não usar o `missao.sh`: a gravação entra em loop. Gravar sempre com `~/gravar_teste.sh` num terminal separado.

## Estrutura
| Caminho | O quê |
|---|---|
| `src/robo_navigation/scripts/` | **código que roda direto do src** (o launch chama com `python3`, sem build): `alinhador_passagem.py`, `destravar_node.py`, `filtro_corpo_scan.py`, `suavizador_nav.py` |
| `src/robo_navigation/config/nav2_params.yaml` | DWB direto, costmaps, Collision Monitor (NAV_INTEGRADA: global com 1 `obstacle_layer`, 3,5/4,0 m) |
| `src/robo_navigation/config/slam_mapping.yaml` | slam_toolbox (`occupancy_threshold` 0.2, `mode: mapping`) |
| `src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml` | árvore do Nav2 (NAV_INTEGRADA 10/11; a anterior em `navigate_to_pose_smooth_anterior_20261110.xml`) |
| `src/robo_navigation/launch/navigation.launch.py` | Nav2 + filtro + destravar + alinhador + suavizador (NAV_TUDO) |
| `src/robo_hardware/config/hardware.yaml` | ponte Arduino: PWM, zonas mortas |
| `src/robo_localization/config/localization.yaml` | odometria por tabela (ODOM_TABELA) |
| `~/subir_robo.sh` | sobe tudo num terminal, na ordem certa (`--novo` ou nome do mapa) |
| `~/gravar_teste.sh nome "desc"` | grava o bag + params + logs e gera o `tar.gz` |
| `~/mapas/` | mapa (`.posegraph`/`.data`), `service_areas.yaml`, `<mapa>_pontos.yaml` (START/FINISH) |
| `~/logs/<hora>/` | logs do `subir_robo.sh` |
| `src/robo_navigation/scripts/executor_prova.py` | plano da tarefa (com PLANO_V2); roda à mão com `python3`, o launch não chama. A versão antiga ficou em `executor_prova.py.bak_antigo`. `~/Robo-WORK/ferramentas/executor_prova.py` não tem PLANO_V2 |
| `src/robo_navigation/scripts/marcar_mesa.py` | marca mesa pela pose + LiDAR e grava em `~/mapas/service_areas.yaml`; roda à mão com `python3` |

Mudou config, árvore ou launch → `cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash` e reiniciar o `~/subir_robo.sh`. Mudou só um script de `scripts/` → basta reiniciar.

## Ordem de subida (o `subir_robo.sh` já faz)
ponte (`robo_hardware hardware.launch.py`, espera `/imu/data`) → LiDAR (`/scan_front`) → localização (`/odom`) → slam_toolbox → recarregar o mapa no START (`deserialize_map`, pose do `<mapa>_pontos.yaml`) → Nav2 (espera o `bt_navigator` ativo) → limpar costmaps.
Se o Nav2 subir antes de `/odom` e do `map`, o `bt_navigator` fica inativo e recusa os goals.

## Arquitetura da navegação
- **executor_prova** (plano da tarefa) → manda o goal para o **alinhador**.
- **alinhador** (`alinhador_passagem.py`, v3.32): analisa o caminho e acha passagens estreitas (corredores).
  - Em área aberta, usa o Nav2.
  - Em corredor, faz ele mesmo o preparo, a travessia pelo LiDAR, a saída lateral, a curva e o encosto na mesa.
  - **VIRAR_CAMINHO:** antes de cada goal do Nav2, vira para o caminho e, se a quina pega, anda o mínimo antes.
  - **ANTI_CISCADO:** ≥ 4 inversões do `/cmd_vel_nav` em 4 s sem sair do lugar → cancela o goal, chama o destravar e reenvia (até 3×).
  - **GIRO_FINAL:** nas mesas, o Nav2 só leva até a posição e o alinhador faz o giro final. Desliga em tempo de execução com `ros2 param set /alinhador_passagem giro_final_alinhador false`.
  - **Giro seguro (`_girar_seguro` / `escolher_manobra`):** varre o retângulo do CM no giro. A 0,30 rad/s se o retângulo de 7 cm está livre; senão a 0,15.
- **Nav2:**
  - controle: DWB direto (NAV_INTEGRADA 10/11; era RotationShim + DWB, giro rápido 0,40 rad/s perdido);
  - planejamento: NavFn, que só considera o raio inscrito de 0,215 m e por isso acha que passa em vãos de ~0,43 m;
  - Collision Monitor com polígonos por velocidade;
  - árvore (NAV_INTEGRADA): caminho conferido a 2 Hz no trecho restante; suavização com colisão (falhou → original se válido); recuperação = destravar v4, depois replaneja sempre.
- **destravar** (`destravar_node.py`, v4 + `recuperacao_avancada.py`, 10/11): servidor `nav2_msgs/DriveOnHeading` "destravar". Diagnostica (log `DESTRAVAR: DIAGNOSTICO/DECISAO/RESULTADO`, tópico `/destravar/estado`), devolve ao Nav2 se ainda contorna, CONTORNAR/lateral com continuação e deriva, giro pelo smart_spin, **ré bloqueada** (`ros2 param set /destravar permitir_re true`); confirma por ICP do LiDAR/IMU; não repete no mesmo lugar; falha explícita. Relatório: `entregas/nav_integrada_20261110/RELATORIO.md`. O que segue é do v3 e continua valendo para bloqueio virtual e CONTORNAR:
  - Lê o `/plan` e marca um bloqueio virtual (`/bloqueio_virtual`): só no costmap local, por 10 s.
  - Pede um caminho novo; sem caminho, apaga o bloqueio, limpa os costmaps e planeja de novo. Então tenta, nesta ordem: girar para o caminho / andar na direção dele / ré até o giro ficar livre / de lado.
  - **CONTORNAR:** com a frente a < 15 cm, o caminho para a frente e o destino a ≥ 60 cm, se o que barra é um objeto de até 25 cm, desliza para o lado livre até a faixa à frente ficar livre pelo LiDAR, avança 25 cm e devolve ao Nav2. Não repete no mesmo lugar por 30 s.
- **filtro** (`filtro_corpo_scan.py`, v4): `/scan_front` → `/scan_nav`. Tira o corpo (NaN) e tem a memória do ponto cego (MEMORIA_PERTO v2: o ponto lembrado é apagado se o LiDAR enxerga além dele).
- **suavizador:** `/cmd_vel_ctrl` → `/cmd_vel_nav` (TAU 0,25 / TAU_GIRO 0,12). O alinhador publica direto em `/cmd_vel_nav`.

## Física do robô (medida — usar nas contas)
- **Footprint:** x −0,215 … 0,375, y ±0,325, ou seja, 0,59 × 0,65 m. Para girar no lugar, precisa de um **círculo livre de ~0,50 m** de raio.
- **LiDAR Delta-2D:**
  - fica em x = 0,33; não mede nada a menos de ~0,146 m, então a **frente é cega até ~11 cm além do para-choque**;
  - 31–38% dos raios voltam `inf`;
  - enxerga a traseira.
- **Motor sem encoder:** v ≈ 0,00149·PWM − 0,0137 m/s. PWM 215 dá ~0,31 m/s. **De lado, ~0,6× a frente.**
- **Hardware atual:**
  - `minimum_pwm` 70: anda no mínimo ~0,10 m/s de frente e ~0,06 de lado;
  - `minimum_rotation_pwm` 50: **gira parado no mínimo ~0,15 rad/s**;
  - zona morta do giro andando: 0,02 rad/s.
- **Consequência:** qualquer comando abaixo desses mínimos é executado 2–4× mais forte. Correção fina vira ciscado. Nunca pedir movimento que o motor não faz (por isso o DWB_GIRO_REAL: `min_speed_theta` 0,15, `min_speed_xy` 0,05).
- **Medido em 06/11 (robô pesado, chão com frisos, `teste_deslize_icp.py`):** de lado ~0,05–0,06 m/s real com comando de 0,08 a 0,16, a odom superestima; desvio para frente/trás k 0,31–0,37 a 0,08 e 0,05–0,14 acima (uma corrida por caso). LiDAR 5,9 Hz, IMU 25 Hz. Ângulo de parede > 5° só aparece com uma parede vista (missao22).
- **Saída lateral na missao23:** 1,89 m até o fim; a ~0,05 m/s isso é no mínimo ~38 s. Giro andando não corrigiu o rumo (−0,35°/s com qualquer comando) e aumentou a deriva.
- **IMU falha com os motores girando (06/11):** parado, intervalo máximo 0,05 s; andando, 0,6 s ou mais, com `bad_usb_rx_frames` subindo. Causa não confirmada.
- **Odometria:** comando → tabela de velocidade (frente e lado), mais o yaw da IMU. Superestima em pulsos curtos (aceleração do motor) e não vê deriva que não foi comandada.

## Patches aplicados (marcas no código/config)
GLOBAL_OBSTACULOS, SUAVIZADOR_NAV, SHIM_ALINHAR, INF_VALIDO, PWM60 → PWM_AJUSTE (70), DWB_SEM_RE (Nav2 não dá ré), ARVORE_V3, CHEGADA_10G (yaw 0,17), NAV_TUDO, GIRO_50, SUAV_V2, CM_FRENTE, BLOQUEIO_VIRTUAL, DWB_GIRO_REAL, MANTER_CAMINHO v1.2 (replaneja só com goal novo, caminho inválido ou caminho com mais de 15 s), NAV_FRENTE (`vel_y` ±0,08, `rotate_to_heading_once` false), BLOQUEIO_LOCAL, CONTORNAR, WS2_FACE (`ferramentas/patches/`; o WS2_FACE muda as marcas em `~/mapas/`), CM_TIMEOUT (`source_timeout` 0,5), IMU_CONTINUA (yaw da odom continuo quando a IMU volta) e SENSOR_PARADO (alinhador e destravar param com scan, odom ou IMU a mais de 0,5 s) (`aplicar_h1.py`), SENSOR_IMU2, ENCOSTO_LINEAR, ANGULO_FUNDIDO, ENTRADA_RAPIDA, VEL_ZONAS, NAV_VEL20, UMA_PAREDE, DESLIZE_SEM_GIRO, FREIO_FRENTE e ROTA_MAPA (`aplicar_nav.py`; parametros `encosto_linear`, `angulo_fundido`, `entrada_rapida`, `vel_travessia`, `uma_parede`, `rota_mapa`).
09/11 (sessao Claude): SONDA_ENTREGA (executor espera o alinhador responder antes de mandar a 1a area), FACE_ANG20 e TRAVA_ODOM (`face_ws.py`), FREIO_CRUZAMENTO, PAREDE_MAPA (parede do fim do T so vale se bater com o mapa +-15 cm), TF_TOLERA (saida lateral tolera 1 s sem TF), ENCOSTO_ESPERA e ENCOSTO_CEGO (a < 15 cm o LiDAR nao ve o meio da face: aceita pelo centro medido de longe, +-3 cm), LOCALIZACAO_FIXA (`ferramentas/patches/aplicar_localizacao_fixa.py`: com mapa o slam_toolbox roda em localizacao, match_type 3; `~/subir_robo.sh --mapeando` = modo antigo), FALHA_SCAN (1 scan com a face partida pelos raios inf nao aborta o encosto; 0,45 s sem face aborta), WS2_REMAP e WS1_REMAP (`aplicar_ws2_remap.py`, `aplicar_ws1_remap.py`: faces medidas pelo LiDAR nas missoes 29 e 30; os cadastros antigos foram marcados em modo mapping e estavam 11 graus (WS_2) e 25 cm (WS_1) fora). CM_RIGIDO (`aplicar_cm_rigido.py`: giro_esq/giro_dir 11 cm de margem (era 7), left/right frente 0,425 (era 0,385); RET_GIRO_RAPIDO acompanha). JANELA_LARGA (`face_ws.py`: janela da face +-0,95 m (era 0,70) e centro ate 35 cm do esperado (era 18): reconhece a mesa com o robo ate ~30 cm de lado e desliza ate o centro). MOTIVO_FACE (o log diz por que a face foi recusada: sem as 2 pontas / ambigua / qual limite estourou) e PERDA_PARA (face perdida num giro/deslize de correcao: para e mede de novo ate 3x; no avanco final segue abortando/ENCOSTO_CEGO). ATENCAO LOCALIZACAO_FIXA (missao31): o localization_slam_toolbox_node saltou 30-60 cm e 6-14 graus varias vezes por segundo ao girar/andar (scan no mapa 55-60%); com ele o robo bateu. Usar mapping.launch.py + match_type 2 ate ajustar.
FACE_FRAGMENTADA (09/11, missao32, `face_ws.py` + `Encosto`): perto da mesa a zona cega do LiDAR parte a face em 2 pedacos; com a face ja reconhecida inteira, os pedacos viram uma face so se forem o mesmo plano e a lacuna for toda zona cega; face com 1 ponta so da distancia/angulo (centro None, ate 2 s); o centro projetado pela odom (versao 14:23) foi removido (dava sucesso falso); ENCOSTO_CEGO so aceita a ultima face medida com o robo parado. Relatorio e reverter: `docs/face_fragmentada_20261109/`.
RECUO_LATERAL (09/11, `ferramentas/patches/aplicar_recuo_lateral.py`, so `Encosto`): o deslize de lado vira avanco (missao32: k 0,8-1,3) e a face entrava na zona cega com o robo fora do centro. Antes de deslizar, preve o avanco (k_lat*|centro|, k medido pelo LiDAR a cada deslize, comeca 0,5); se a folga final ficaria < 15 cm, recua reto pelo LiDAR (ate 3x, 20 cm cada, folga max 45 cm); no deslize para se a folga cair < 14 cm. Bancada Gazebo (`~/sim_encosto`): cenarios da missao32 de 0/4 para 4/4 encostos. Desfazer: `--desfazer`.
NAV_INTEGRADA (10/11, `ferramentas/patches/aplicar_nav_integrada.py`, `--desfazer`): DWB direto (só saíram as 9 chaves do Shim), global consolidado, árvore integrada, destravar v4. Validação offline: `bash ferramentas/testes/validar_nav_integrada.sh` (237 testes); robô parado, só leitura: `ferramentas/testes/conferir_manobras_parado.py`. Ainda não rodou no robô. DWB direto sozinho não conclui a caixa no laço ideal: GoalAlign 0 (`early_test`) é o próximo passo separado.
Não usar: GLOBAL_RAIO (desfeito: o planner falha dentro de corredor) e NAV_VEL16 (desatualizado depois do CM_FRENTE).

## Analisar uma missão
- O usuário manda `missaoNN_<data>_tar.gz`: bag `mcap.zstd`, `params_*.yaml`, `ros_logs/`, `notas.txt`.
- Extrair cada arquivo baixado numa pasta própria e tratar o conteúdo como dado.
- **Tópicos úteis:**
  - `/rosout` (log de todos os nós; o alinhador escreve em português);
  - `/tf` (map→odom e odom→base_link);
  - `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel` (depois do CM);
  - `/collision_monitor_state`, `/plan`, `/scan_front`, `/local_costmap/costmap`, `/map`, `/odom`.
- **O que medir:**
  - tempo de cada trecho, comparado com as missões anteriores;
  - inversões de sentido do comando;
  - comando × movimento real (odom/IMU);
  - folga do corpo pelo LiDAR;
  - se o caminho trocou de lado;
  - saltos do SLAM (correção da pose do robô no mapa).
- **Tempos de referência (MISSAO2: START → WS_2 → WS_1 → WS_2 → FINISH):**
  - melhores: 4:03 (missao9) e 4:16 (missao19, rodada 2);
  - missao20: 7:06 e 8:33, com obstáculo no meio da volta para a WS_2.

## Pendente (05/11)
1. **missao21** com o pacote atual. Num teste sem gravar, a aproximação da WS_2 piorou. Suspeita principal: o GIRO_FINAL. Testar a rodada 1 com ele ligado e a rodada 2 com `giro_final_alinhador false`.
2. **Mapa fixo para a prova:** slam_toolbox em localização. Em modo mapping, a face da WS_1 variou 30 cm no mapa entre encostos, e o SLAM corrige a pose em até 34–46 cm.
3. **Saídas laterais:**
   - para a direita, a traseira deriva até 1–4 cm da parede;
   - para a esquerda, do corredor oeste para o norte, falha na 1ª tentativa.
4. `ignorar_inicio`: hoje o alinhador entrega o corredor ao Nav2 com o robô já dentro.
5. Odometria em pulsos curtos; subir a velocidade (NAV_VEL16 atualizado); destravar girando a 0,30 quando livre.
6. Plano: KeepoutFilter para as fitas, integração do braço, provas BTT II cronometradas, **congelar em 18/11**.

## Regras da prova (resumo)
- Passagens de 80 cm até as áreas de serviço. Obstáculos físicos ≥ 5×5×40 cm (BTT II: um bloqueando e um parcial).
- Pode mapear a arena antes; obstáculos não podem ir no mapa.
- Tocar parede = colisão grave. Cruzar fita: −5/−10/−25%. Cada ajuda humana: −100.
- 20 min por prova; cada segundo restante = 1 ponto.

## Missao34 — validacao offline (11/11/2026)
- Restricao desta sessao: **nao reiniciar Nav2 nem movimentar o robo**. Nenhuma ativacao no robo; alteracoes locais aguardam subida futura autorizada.
- `CM_PROTECAO`, `SEM_SUAVIZADOR` e `CURVA_INSISTE`: testes de aplicar/idempotencia/desfazer em copias, com backups preservados. `test_missao34.py` confere cobertura translacional/selecoes CM, limites de velocidade, cancelamento e falhas de seguranca.
- `SEM_SUAVIZADOR` retira SmoothPath (planejamento), **nao** os suavizadores de velocidade. Testes exigem ausencia dos dois nos, copia do caminho e passagem por CaminhoOriginalSeguro. Em modo `aplicar`, original bloqueado falha sem FollowPath mesmo quando a simulacao de suavizacao aprovaria. Modo instalado continua `registrar`: nao veta o caminho.
- Perfis `dwb_test`/`early_test` receberam CM_PROTECAO pelo proprio patch, sem retirar as verificacoes de igualdade/protecao. Perfil `nav2_params_goalalign0.yaml` atualizado a partir do YAML atual: diferenca unica GoalAlign.scale=0; backup anterior preservado. A falha do teste de igualdade era perfil desatualizado, nao foi relaxada.
- `CURVA_INSISTE_SEGURO`: so `tempo maximo` permite retentativa (ate duas, mesmo se parametro maior) e fallback configurado. Bloqueio repetido do CM, odom/TF ausentes nao reiniciam os contadores. Cancelamento/ SensorParado continuam interrompendo com parada em finally. Falha experimental `giro_frontal` nunca cai no Nav2. Timeout ainda pode repetir sem progresso: requer observacao em teste fisico futuro.
- Patch complementar reversivel: `ferramentas/patches/aplicar_curva_insiste_seguro.py`; desfazer este **antes** de `aplicar_curva_insiste.py --desfazer`. Mudancas ja estao no src; nao foram ativadas no Nav2.
- Replay SLAM concluido usando CSV/logs existentes de aproximadamente 594 s em ambos os modos; comparador reproduzivel `entregas/nav_refinamento_20261111/scripts/comparar_slam_m34.py`, resultados `slam_replay/comparacao_m34.json`. Maximos de salto yaw map->odom: mapping 21,00 graus; localization 36,05 graus; 64/50 correcoes acima de 5 graus. Ambos instaveis: **nao promover localization** com essa evidencia. Saltos x/y de map->odom referem-se a origem odom, nao ao deslocamento do robo. Sem ground truth; descarte de scans nos dois logs; encerramento forcado pelo script apos gravacao. Nao houve novo replay nesta sessao.
- Detalhes de validacao e riscos: `entregas/nav_refinamento_20261111/missao34/VALIDACAO.md`.
- Resultado desta sessao: suite inicial 265 aprovados/6 falhas; selecao final afetada 76 aprovados. Permanecem quatro testes ROS sem descoberta do servidor no sandbox (`getifaddrs: Operation not permitted`); sem skip/xfail. Reexecutar em dominio isolado com DDS permitido. Suite completa nao repetida apos correcoes. CM validado offline por configuracao/selecao, ainda sem certificacao de frenagem real.

## Arquitetura NAV2_PRINCIPAL (11/11/2026, opt-in)
- `nav2_principal:=true` no launch mantem `/goal_pose_alinhado` e `/ir_para_area`/resultado, mas envia goals comuns diretamente para NavigateToPose, com pose e yaw originais. Sem VIRAR_CAMINHO, trajeto curto, rota estatica, anti-ciscado paralelo ou REINSISTIR no novo fluxo. Parametro somente de inicializacao; `false` conserva implementacao legada.
- Alinhador so assume bancada/saida de bancada ou passagem no plano atual, confirmada por mapa + duas paredes LiDAR concordantes, perto da entrada. Preservados preparo, travessia e T lateral. Depois de passagem/T concluido, nova NavigateToPose reinicia a BT e replaneja; falha de seguranca para sem forcar continuacao. Bancada conserva encosto, resultado e recuo antes do proximo goal.
- `arbitro_nav_principal.py`: Nav2/recuperacoes -> `/cmd_vel_nav_base`; alinhador -> `/cmd_vel_nav_especial`; unica saida `/cmd_vel_nav` antes do velocity_smoother e CM. Lease 0,5 s, heartbeat carimbado e descarte de comandos anteriores a troca. Action anterior precisa de resultado terminal antes de trocar autoridade; timeout/aceitacao tardia bloqueia em vez de liberar outro controlador. Cliente externo nao deve publicar direto depois do arbitro.
- Perfil separado `nav2_params_principal.yaml` + `navigate_to_pose_principal.xml`: DWB direto, GOAL_TF_ATUAL, destravar v4 e recuperacoes preservados; CM_PROTECAO e SEM_SUAVIZADOR pendentes nao incluidos. CURVA_INSISTE nao usado no novo modo; launch passa retentativa=0/fallback=false. Arquivos principais M34 permanecem guardados, sem promocao.
- Build em overlay **isolado** `install_nav2_principal`, com `build/nav2_principal`; instalacao operacional nao sobrescrita. Nenhum restart/movimento. Ativacao futura e comparacao A/B sem misturar M34: `entregas/nav2_principal_20261111/COMANDOS.md`.
- Auditoria, evidencias, limites do replay e rollback verificavel: `entregas/nav2_principal_20261111/`. Regras locais em AGENTS.md. Replay reexecuta decisoes sobre bags m33/m34, nao demonstra sucesso de trajetoria fisica alternativa.
- Validacao NAV2_PRINCIPAL: build de dois pacotes e sondas reais BT/DWB aprovados; suite offline 293 aprovados/1 falha historica corrigida, checagem final 25 aprovados. Replay m33/m34: 17 goals preservados; 5 amostras de passagem confirmadas na m33, nenhuma na m34 (Nav2 permanece dono). Quatro testes ROS de destravar foram excluidos explicitamente desta execucao pela limitacao DDS documentada, nao marcados como aprovados. Integracao ROS multiprocesso e ensaio fisico pendentes; detalhes em VALIDACAO.md.

## Ajustes da missao35 (11/11/2026)
- Usuario confirmou NAV2_PRINCIPAL=true em execucao; snapshot inicio da missao35 confirma isso e SLAM em mapping. `slam_mapping.yaml` e identico byte a byte ao arquivado na missao33; usuario escolheu explicitamente manter/restaurar esse mapping original. Nao trocar para localization nem afrouxar matcher sem evidencia.
- Perda de orientacao medida: 17 saltos map->odom acima de 3 graus, maior 14,288 graus (IMU no entorno apenas 0,125 grau). IMU tambem tem salto global de 85,625 graus na gravacao; odom continua teve maior incremento de 2,375 graus. Correcao de scan nao comprova sozinha estabilidade do SLAM; grafo em memoria pode ja estar contaminado.
- `CORPO_REAL_M35`: mascara geometrica antes de /scan_front (SLAM/alinhador/CM), NaN somente dentro do envelope fisico x -0,215..0,375 / y +-0,325. Antigos setores inteiros bloqueados foram substituidos por essa mascara, preservando obstaculos alem dos motores. Cobre tambem fluxo temporal, com montagem derivada dos mesmos argumentos da TF. Filtro downstream e memoria perto usam envelope real, sem encolhimento padrao.
- Analise em trechos completos de bag35 ainda em gravacao, somente leitura: 4.758 scans brutos no replay do filtro; 119.460 retornos dentro do corpo removidos, 565.147 retornos externos preservados; 26.053 externos antes cegados pelos setores foram recuperados. Entre os retornos corporais, 32.458 escapavam dos antigos setores. Angulos persistentes incluem -146/-144/-142 e 138/140/162/164 graus (referencial corrigido). Ver evidencias por bin, nao usar como mascara angular infinita.
- `NAV_RE_M35`: apenas nav2_params_principal.yaml permite DWB min_vel_x=-0,10 (zero mantido na grade de 7 amostras); backward do CM ampliado a x=-0,315 (10 cm do corpo). Legado permanece min_vel_x=0; destravar permitir_re permanece false. GOAL_TF_ATUAL, criticas DWB e recuperacoes preservados; os tres patches pendentes M34 nao foram promovidos.
- `PASSAGEM_INICIADA_M35`: detector do novo modo inclui passagem ja no inicio do trecho restante; quando confirmado por mapa+duas paredes e ja paralelo dentro dela, nao volta ao preparo. ABORTED do Nav2 permite conferir vigia uma ultima vez, mas so transfere com passagem confirmada daquele goal. CANCELLED/goal novo nunca vira manobra; planner sem caminho continua falha segura.
- `LATERAL_SEM_PULSO_M35`: no novo modo, suavizador zera eixo explicitamente pedido como zero e limita modulo ao novo pedido, evitando carregar vx anterior para lateral pura. Diagonal intencional nao e apagada. Medicao bag35: primeiro inicio lateral ja tinha vx=0,08/vy=-0,053 no DWB; nos outros 10 inicios DWB avaliados vx=0. Logo nao atribuir todos os pulsos a software nem zerar vx globalmente; deriva mecanica e slew dos motores precisam de medida independente.
- Build isolado robo_lidar/robo_navigation aprovado; sonda DWB real: 168 amostras de re, 4 laterais puras para cada lado. Seis cenarios nativos sem colisao; aberto/curva/yaw concluiram, caixa estatica/dinamica/quina nao concluiram (limite mantido explicito). Suite inicial afetada 178 aprovados/1 teste de mascara antiga corrigido para verificar NaN como DESCONHECIDA; selecao final 70 aprovados. Testes ROS multiprocesso seguem pendentes por DDS restrito.
- Nao houve restart, troca de parametros ativos ou movimento pela ferramenta. Configs/processos ja abertos nao recarregam automaticamente; novas correcoes aguardam janela autorizada. Evidencias, backup e rollback: `entregas/ajustes_missao35_20261111/`.

## Correcao de partida (11/11, log 12:53)
- Causa comprovada de configuracao ausente: launch executado com resolucao do pacote pela instalacao antiga, procurando `install/robo_navigation/share/robo_navigation/config/nav2_params_principal.yaml`, arquivo inexistente. Alinhador recebia goals, mas isso nao comprovava que Nav2 estava configurado/ativo. Nao atribuir esse sintoma apenas a falta de area.
- `ferramentas/iniciar_nav_validada.sh` carrega overlay no mesmo processo, confere prefixo e perfil/BT e passa caminhos absolutos. Usar apos encerrar a instancia Nav2 anterior; nao executar duas instancias. Para filtro, existe `ferramentas/iniciar_filtro_validado.sh`, que tambem verifica binario e configuracao.
- Launch agora valida existencia/sintaxe YAML/XML ANTES de iniciar qualquer no, inclusive arbitro. Config ausente causa recusa explicita, em vez de subir parcialmente com defaults. Novo coordenador registra envio, aceitacao, recusa, timeout, servidor indisponivel e status/motivo terminal.
- Build isolado aprovado; 58 testes afetados aprovados, incluindo instalacao antiga sem perfil e abortar antes de qualquer processo. Nao reiniciamos stack nem enviamos goals. Backup, evidencias e rollback em `entregas/correcao_partida_20261111/`. Desfazer primeiro esta rodada antes do rollback missao35, para preservar a verificacao de edicoes posteriores.

## Supervisor e localization — 11/11/2026
Pedido atual: implementar supervisor e preparar localization como perfil de teste. Arquitetura opt-in existente foi estendida, sem reiniciar stack/hardware. Default `supervisor_modo=observar`; aplicar exige cadastro de regioes no mapa (vazio ate medicao). Travessias existentes conservadas, com evidencia incremental ICP/IMU e bloqueio de retomada sem footprint inteiramente fora e saida livre/parada. Nao recuperar com re automatica apos falha de confianca.
Perfil `slam_localizacao.yaml` gerado do mapping atual alterando somente mode/map_file_name/map_start_pose; par arena_04-11_1235 validado estruturalmente e START preservado. Mapping nao alterado. `~/subir_robo.sh --localization arena_04-11_1235` prepara perfil do mapa solicitado, recusa instancia SLAM preexistente e exige map->odom antes do Nav2. Sem flag usa mapping por seguranca enquanto falta validar inicializacao no ambiente ROS permitido. Localization nao foi promovido como estabilizacao: replay34 existente tem salto maximo 36,05 graus versus 21,00 mapping.
Entrega e rollback desta rodada: `entregas/supervisor_localization_20261111/RELATORIO.md`. Restricao DDS impede confirmar inicializacao/TF em ensaio ROS aqui; nao afirmar validacao fisica ou da missao completa.

## Correcao da subida do supervisor (11/11)
A subida 14:59 e a subida localization 15:08 utilizaram arquitetura legada: subir_robo.sh carregava install/setup.bash e chamava navigation.launch.py sem nav2_principal. Localization passou a funcionar (callback de relocalizacao aparece), mas isso nao ativou o supervisor. Corrigido: comando operacional agora usa o wrapper do overlay isolado por padrao em observar, com preflight e verificacao posterior dos parametros reais e do arbitro. --legado restaura arquitetura anterior; --supervisor-aplicar exige regioes cadastradas. Nenhum restart/movimento pela ferramenta nesta correcao. Orientacao/START continuam sem validacao ao vivo; nao confundir comando de pose recebido com convergencia. Evidencias e rollback: entregas/correcao_subida_supervisor_20261111/RELATORIO.md.

## Partida real do arbitro — 11/11 15:27
Falha 15:19 era falso negativo da consulta node list sem tempo de descoberta: arbitro iniciou (pid55559) e morreu so apos SIGINT enviado pela subida que o declarou ausente. Consulta listava apenas parte do grafo. Corrigido para verificacao rclpy persistente (servico do arbitro, parametros do alinhador, heartbeat e exatamente um endpoint cmd_vel_nav por 1,5s), sem suprimir falha verdadeira. SIGINT do arbitro tratado sem traceback de contexto invalido. Novo wrapper grafico mantem shell aberto apos termino.
Acesso DDS/GUI liberado pelo usuario. Teste ROS isolado passou; subida operacional autorizada realizada em ~/logs/11-11_152704 com localization e novo perfil observar. supervisor_ativo.json confirmou arquitetura, servico e publisher unico. Terminais de logs hardware/LiDAR/odom/SLAM/Nav2 abertos sem duplicar instancias. Sem envio de goals nem velocidades nao zero. Amostra parada: 199 cmd_vel_nav zerados, sem mensagens cmd_vel final observadas; 19 poses map->base yaw constante 0,650 grau em 10s, IMU relativa zero. Isso nao comprova alinhamento fisico absoluto ou estabilidade em movimento. START e regioes nao alterados; aplicar continua bloqueado.
Detalhes e rollback: entregas/partida_arbitro_20261111/RELATORIO.md. Desfazer esta rodada antes dos rollbacks anteriores; codigo revertido nao modifica processos vivos.

## Goal superior apos curva — 11/11
Tentativa goal(1,57;2,31) aceito, mas NavFn falhou duas vezes desde(0,34;-0,09), inclusive apos limpar costmap. Recuperacao nao achou continuacao lateral. Supervisor estava observar e nao houve /plan novo para detector: alinhador especializado nao assumiu. Erro adicional Logger severity cannot be changed between calls ocorria no getattr(logger,nivel) compartilhando callsite; corrigido para chamadas info/warning/error distintas e testado com rclpy real. Correcao salva no src; processo ativo nao reiniciado nesta rodada.
Snapshot somente leitura posterior: inicio custo87, goal0, componentes desconectadas no costmap, inclusive allow_unknown; mapa ocupacao conectado, inclusive apos modelo offline de margem circular0,325m. Isso nao comprova passagem do footprint retangular ou estado exato na hora do goal. 82 celulas letais do costmap em locais livres no mapa e respectivas zonas de inflacao precisam ser correlacionadas com scans antes de ajustes. Nenhuma alteracao SLAM/DWB/CM/regioes, nenhum goal enviado pela ferramenta. Evidencias e rollback: entregas/goal_superior_20261111/.

## GANHO_LATERAL (12/11/2026) — ATIVO
- Teste físico 12/11 (robô de frente para a parede, `ferramentas/testes/teste_k_lateral.py` e `teste_frente_giro.py`, CSV em `~/testes_k_lateral/`, bags `teste_k_lateral_*` e `teste_ganho_roda_*`): o deslize lateral saía na diagonal, dx/dy +3 a +3,7 para a esquerda e −1 a −4 para a direita, com só 5–9 cm de lado real em ~2,7 s. Trocar rodas, centro de gravidade e bateria não mudou isso. k do feedforward (vx) e reforço só da traseira ajudaram pouco.
- O que resolve é equilibrar as diagonais: FL/RR fortes, FR/RL fracos. Com ganho FIXO, o andar reto entortava (2,7–3,1 cm por 10 cm). Por isso o ganho entra só na proporção lateral: peso = |vy|/(|vx|+|vy|).
- `ferramentas/patches/aplicar_ganho_lateral.py` (ponte; parâmetros `lateral_wheel_gain_fl/fr/rl/rr`, padrão 1,0 no código, `ros2 param set` muda na hora) e `aplicar_ganho_lateral_yaml.py` (`hardware.yaml`: 1,6 / 0,6 / 0,7 / 1,6). Os dois têm `--desfazer`; a ponte precisa de `colcon build --symlink-install --packages-select robo_hardware`.
- Resultado com os ganhos:
  - lateral: |dx/dy| ≤ 1,07 (mediana ~0,35) e 14–21 cm de lado real;
  - frente/trás e giro: iguais à referência (PWM idêntico em 243/243 e 301/301 quadros).
  - Varia com o ponto do piso (frisos). Num lugar ficou levemente sobrecompensado (esquerda −0,4, direita +0,4).
- Suspeita mecânica em aberto: FL/RR rendem ~2× menos só no deslize. Testar trocando os canais do driver FL↔FR.
- O LiDAR falha com os motores girando (`bad_usb_rx_frames`, buffer serial2 do LiDAR cheio).

## Teste no corredor 12/11 10:43 (sem bag; log ~/logs/12-11_103941/) e FREIO_CURVA
- Avisos "Velocity command source timestamp invalid/stale" da ponte (78 no teste, 39 em 11/11, 170 no bag teste_ganho_roda): sao o zero de fim de movimento que o velocity_smoother republica com o carimbo do ultimo comando (0,25-0,35 s). 167/170 eram zero; 3 foram pausa do script de teste. Nao param o robo no meio do deslize. Nada mudado.
- Objeto logo apos a curva para a esquerda (continuacao do reto): o alinhador planejou a curva certa, mas abortou com o robo ~5 cm ANTES do ponto do cruzamento (freio 10 cm / CM 11 cm), re de 1,9 m, PASSAGEM_BLOQUEADA com rota "alternativa" pelo mesmo cruzamento (preparo a 3 cm do objeto).
- `FREIO_CURVA` (`ferramentas/patches/aplicar_freio_curva.py`, `--desfazer`; teste `test/test_freio_curva.py`): com curva planejada, freio/CM parando com restante <= tolerancia (auto (largura_b+0,06-0,59)/2, 3..10 cm) termina a travessia e segue a saida lateral. Parametro `freio_curva_tol` (-1 auto, 0 = como antes). Testes puros 409/0 (stack ligada: testes ROS nao rodados).
- Pendente: preparo/rota alternativa ignoram o corpo perto do objeto marcado (`passagem_bloqueada_raio` 0,35 = so 2,5 cm alem da meia largura).
- RE_LIBERADA (12/11, pedido do usuario; `ferramentas/patches/aplicar_re_liberada.py`, `--desfazer`): destravar com `permitir_re` true por padrao. Teste 11:18: robo no cruzamento com frente 1 cm e quina; o destravar recusou tudo e a re estava bloqueada. A re continua com as protecoes da ETAPA D: ate 15 cm, nunca 2 seguidas, poligono backward do CM, 90% da area observada e ICP. Desligar: `ros2 param set /destravar permitir_re false`. O DWB (Nav2) continua sem re no legado. Suite completa 423/0.
- 11:17: a travessia foi abortada 2x por "Collision Monitor bloqueando dentro da passagem", ja com FREIO_CURVA. Sem bag nao da para saber a que distancia do ponto do cruzamento: gravar o proximo teste.
- Bag `corredor_re_freio_curva_20261112_113352`, 11:45:44: a travessia chegou ao cruzamento ("parede a 9 cm"), mas o robo embalou e o /scan_nav (fonte do CM, com memoria do ponto cego) ficou com o objeto a 4,6-5 cm do para-choque, dentro do poligono 'left' (5 cm alem do para-choque): a saida lateral foi barrada 4x. O destravar recusou a re por "area 66-71% observada (min 90%)".
- Patches de 12/11 (cada um com `--desfazer` independente da ordem, e parametro para desligar):
  - `RECUO_CURVA` v2 (`aplicar_recuo_curva.py`, `recuo_curva` true): na saida lateral (qualquer curva, esq ou dir), CM segurando com a traseira livre (>= 8 cm ou sem medida) recua 5 cm; com a traseira curta e a frente >= 10 cm avanca 5 cm; no maximo 2 vezes. v2 depois do bag `corredor_recuo_curva_20261112_120232` (12:07, volta para a DIREITA): frente "medida" 14 cm no /scan_front, mas o /scan_nav lembrava pontos a 4,3 cm (zona cega). Na ida (12:05) o RECUO_CURVA v1 funcionou.
  - `MANOBRA_FOLGA` (`aplicar_manobra_folga.py`, `manobra_margem` 0,06): no giro com a quina, anda o minimo + 6 cm, se houver folga.
  - `RE_LONGA` (`aplicar_re_longa.py`, destravar `re_max` 0,30 e `cobertura_re` 0,65).
  - Teste `test/test_recuo_curva_re_longa.py`. Testes puros 422/0.

## ORIENTAR_CAMINHO e companhia (13/11/2026) — bag arena_start_20261113_112926
- Bag (11:29–11:49): IMU caiu 116× (113,6 s sem /imu/data), Mega resetou 4× às 11:37:20 (USB I/O error); alinhador parou ~50 s por "IMU parado". Hardware primeiro.
- `ORIENTAR_CAMINHO` (`aplicar_orientar_caminho.py`, destravar + recuperacao_avancada; **ligado**, `giro_orientar`): giro de ORIENTAÇÃO para o caminho (só com caminho atual do planner, ≥ 45°, área varrida livre pelo LiDAR, 0,15–0,20 rad/s, reavalia a cada scan, IMU acumulada, lado longo só se o curto barrado, máx `giro_orientar_max_graus` 360). Antes só existia a varredura do smart_spin (40°, para na 1ª abertura): no bag girou 7 de 157°.
- Prontos e **desligados** (1 por teste físico): `sem_caminho_unico` (destravar), `pulso_giro_adapta` (alinhador: pulso de giro da saída lateral que não girou fica mais longo; no bag wz 0,02 por 0,3 s não mudou o yaw — `w_real` 0,26 é de PWM 80) e `curto_sem_progresso` (alinhador).
- NÃO feito: pré-giro pela rota do mapa sem planner (proibido pela skill: rota estática não validada); IsPathValid com yaw 0 do NavFn (Nav2).
- Testes: `test/test_alinhador_autonomo_20261113.py` (47/0, sem ROS; suíte pura 444/0). Replay das decisões do bag: só 11:41:52 muda (giro 40 → orientar 157). Entrega e rollback: `entregas/alinhador_autonomo_20261113/RELATORIO.md`.
- Rodada 2 (teste 13:14–13:20, SEM bag; logs dos nós em `entregas/alinhador_autonomo_20261113/teste_1314_sem_bag/`): rota longa com goal atrás → alinhador não pré-girou (planner sem caminho) → DWB oscilou → destravar escolheu orientar +164 mas a checagem em movimento (1 ponto) parou com 6°; rota curta: VIRAR_CAMINHO girou +157 OK; perto de obstáculo o DWB parou a 9 cm do goal ("No valid trajectories") e ninguém terminava. Corrigido (ligados, cada um com marca no log): `ORIENTAR_CAMINHO` v2 (`giro_orientar_reavaliar` 3: para e reavalia parado), `VIRAR_REPLANEJA` (`aplicar_virar_replaneja.py`: limpa costmaps e replaneja; senão rota do mapa só com o 1º metro livre no costmap atual), `CHEGADA_PERTO` (`aplicar_chegada_perto.py`: Nav2 falhou a ≤ 15 cm → alinhador termina com LiDAR, ≤ 10 cm/10°). Regra prática: o giro grande é do alinhador ANTES do Nav2; destravar é reserva.

## PREPARO_LATERAL e rodas novas (19/11/2026)
- Rodas trocadas: deslize lateral quase reto com ganhos 1,0 (ICP: esq −0,08, dir +0,05 dx/dy); `hardware.yaml` `lateral_wheel_gain_*` voltou a 1,0 (o 1,6/0,6/0,7/1,6 de 12/11 agora corrige demais: −0,68).
- `PREPARO_LATERAL` (`ferramentas/patches/aplicar_preparo_lateral.py`, `--desfazer`; teste `test/test_preparo_lateral.py` com o mapa real): na volta do L (11:51) o alinhador desistiu com "sem espaco livre para a pose de preparo" (vão de 0,60 m no mapa, 0,82 m real; parede da esquerda sobe até y 1,65) e mandou direto ao Nav2. Agora, só quando eixo e preparo no caminho falham, desloca o preparo de lado o mínimo (até `preparo_lateral_max` 0,15; 0 = como antes); a travessia centraliza na boca. Caso real: +6 cm, d 0,50, curva mantida. Log `PREPARO_LATERAL:`. Só no legado.
- `~/subir_tudo.sh`: tudo num terminal (base com SLAM do zero + Nav2 legado com fita + RViz).
- `ALINHAR_CRUZAMENTO` (19/11, `ferramentas/patches/aplicar_alinhar_cruzamento.py`, `--desfazer`; teste `test/test_alinhar_cruzamento.py`): corredores do L ~9° fora do esquadro (log: horizontal ~+3,5°, vertical ~−95,5° no mapa); a saída lateral não girava com uma parede só (ANGULO_FUNDIDO força 0) e deslizava torta (volta 12:29: folgas 7–10/3–4 cm, "encostando na parede (7 vezes)"). Agora, só antes do 1º deslize, gira pela parede da FRENTE (n ≥ 15, ≥ 3 scans, dispersão ≤ 1,5°, 3–12°), recuando antes se a frente < 10 cm; máx. 3 giros; no deslize nada muda. Parâmetro `alinhar_cruzamento` (false = como antes). Log `ALINHAR_CRUZAMENTO:`.
- Ida 12:26 (sem bag): além da inclinação ("folga frente 0" no início do deslize), a finalização da saída para o norte abortou "abertura identificada, mas sem confirmar folga e parada para sair" — hipótese: traseira junto à parede que sobe até y 1,65 sem folga medida (None) nem lado oposto aberto → 6 s em 'parar' → 'bloqueada'. Não alterado; gravar bag.

## Missão completa 19/11/2026 (noite) — relatório `entregas/missao_ok_20261119/RELATORIO.md`
- WS_1 → WS_2 → WS_1 → FINISH sem ajuda, 3 min 46 s (21:14–21:17); os 3 encostos com folga de 9,5–10,3 cm e ângulo ≤ 1,1°.
- Subida: `ferramentas/subida_unica/abrir_arena.sh` (3 terminais; SLAM em **localization** na `arena_11-11_1938`, match_type 3). Em mapping o robô perdeu a orientação (yaw map→odom −159°).
- Novos (cada um com marca e chave): CHAO_TRASEIRO/CHAO_LADOS (`lidar.yaml`: NaN atrás ≥ 1,05 m e nos lados ≥ 1,5 m; robô inclinado vê o chão), ALINHAR_CRUZAMENTO_ZERO v1+v2 + RECUO_PARA_MEDIR (`alinhar_cruzamento_zero`), VX_DESLIZE, CM_LATERAL65 (só no perfil operacional, via `entregas/cm_lateral65_20261119/`), MEMORIA_OFF (`memoria_perto` false), PRE_GIRO_REFAZ (`pre_giro_refaz_falha`).
- Mesas remarcadas: WS_1 face (2,815; 2,176), WS_2 face (1,10; 4,16).
- Pendente: corrida cronometrada saindo do START; Mega cai (3×) e câmera XIFT ausente.
