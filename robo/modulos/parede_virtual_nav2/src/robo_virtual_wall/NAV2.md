# Interfaces e Nav2

| Interface | Conteúdo |
|---|---|
| /virtual_wall/image | Imagem anotada |
| /virtual_wall/detections | JSON com timestamp, segmentos base_link e domínio observado |
| /virtual_wall/segments | Marcadores instantâneos |
| /virtual_wall/keepout_mask | OccupancyGrid 0/100 persistente, transient local |
| /virtual_wall/filter_info | CostmapFilterInfo, tipo 0 |
| /tape_keepout/clear | std_srvs/Empty, limpa a memória inteira da sessão |

## Adição e retirada

Três observações distintas e compatíveis confirmam a parede. A transformação usa
TF no timestamp da imagem; dados repetidos, antigos, futuros ou sem calibração
métrica válida não confirmam paredes.

remove_visible_missing é false por padrão no nó, para preservar compatibilidade.
Os launches observation_tape e operational_tape habilitam a retirada. O robô deve
ficar parado um segundo; depois exige dois segundos sem observação compatível e
ao menos seis quadros, com os extremos da parede no domínio calibrado. Odom deve
estar recente. Movimento, falta de imagem/TF válido ou saída da região reiniciam
a contagem. Não há expiração de paredes fora da câmera ou durante deslocamento.

Perda breve de TF usa fila limitada, sem substituir por transformação mais recente.
Imagem escura ou sem textura não fornece evidência de retirada. O módulo não
classifica oclusores: objeto claro cobrindo a fita pode parecer retirada.

## Integração opcional com o perfil principal do Robo-WORK

Com hardware, LiDAR, odom, SLAM e TF disponíveis e Nav2 anterior encerrado,
carregue o overlay que contém robo_navigation principal, e depois o deste módulo:

```bash
source ~/robo_jazzy_ws/install_nav2_principal/setup.bash
source install/local_setup.bash
tape_share="$(ros2 pkg prefix robo_virtual_wall)/share/robo_virtual_wall"
ros2 launch robo_virtual_wall operational_tape.launch.py \
  homography_file:="$tape_share/calibration/front_base_link_expanded_validated_20261113.json" \
  preview_homography_file:="$tape_share/calibration/front_diagnostic_registered_20261113_212518.json"
```

Não executar simultaneamente com observation_tape ou outro virtual_wall/tape_keepout.
O launch inclui o navigation.launch.py do robo_navigation resolvido no overlay,
com nav2_principal=true, supervisor observar e BT principal. Mescla filtros em
uma cópia temporária dos parâmetros; não regrava a configuração operacional.
params_file, bt_xml e navigation_launch permitem indicar caminhos explicitamente.
O perfil principal e seus componentes não fazem parte deste módulo; devem estar
instalados e conferidos previamente. Não inicia hardware, SLAM, goals ou teleop.

O YAML config/virtual_wall_filters.yaml acrescenta KeepoutFilter e inflação após
o filtro nos dois costmaps. Não retirar as camadas normais de LiDAR ou mapa.
Conferir parâmetros reais, supervisor e árbitro antes de movimento. Os goals
do perfil principal usam /goal_pose_alinhado. As rotinas especiais que comandam
movimento fora do Nav2 não passam a respeitar o keepout automaticamente.

## Parâmetros principais do keepout

| Parâmetro | Padrão do nó |
|---|---|
| confirmations | 3 |
| confirmation_tolerance_m | 0,025 |
| min_segment_length_m | 0,08 |
| wall_half_width_m | 0,05 |
| remove_visible_missing | false; launches do módulo usam true |
| removal_timeout_s | 2,0 |
| removal_min_frames | 6 |
| odom_topic | /odom |
| max_observation_age_s | 0,5 |

Memória é da sessão e reinicia ao encerrar o nó. Não há persistência em disco,
remoção durante marcha ou correção global das paredes após loop closure.
