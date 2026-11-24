## Alternativa para o source
rosst

> O `rosst` (função no `~/.bashrc`) substitui as 3 linhas de cada terminal:
> carrega o ROS, entra em `~/robo_work_teste/robo` e faz o source do
> `install/`. Funciona de qualquer pasta. Em terminais abertos antes da
> mudança, rode `source bashrc` uma vez (ou abra um terminal novo).

## Terminal 1 — Bridge / comunicação com o robô
source /opt/ros/jazzy/setup.bash
cd ~/robo_work_teste/robo
source install/setup.bash
ros2 run robo_bridge bridge_node

> A bridge acha o Arduino sozinha (ttyUSB0, ttyUSB1, ...) e reconecta se o
> cabo sair. Para forçar uma porta:
> `ros2 run robo_bridge bridge_node --ros-args -p porta_serial:=/dev/ttyUSB1`

## Terminal 2 — Filtro do LiDAR
source /opt/ros/jazzy/setup.bash
cd ~/robo_work_teste/robo
source install/setup.bash
ros2 run robo_lidar scan_filter_node --ros-args --params-file src/robo_lidar/config/lidar.yaml

## Terminal 3 — Descrição do robô + TF + RViz
(antes da odometria: ela precisa do TF base_link -> laser publicado aqui)
source /opt/ros/jazzy/setup.bash
cd ~/robo_work_teste/robo
source install/setup.bash
ros2 launch robo_description display.launch.py

> Este launch já abre o RViz com a config salva em `~/.rviz2/default.rviz`
> (File → Save Config salva nela). **Não feche este terminal para reabrir o
> RViz**: ele também publica o TF do robô (base_link -> laser) e a
> odometria para sem ele. Se o RViz fechar ou travar, reabra só ele:

## RViz (só se precisar reabrir)
source /opt/ros/jazzy/setup.bash
rviz2 -d ~/.rviz2/default.rviz

## Terminal 4 — Odometria LiDAR-only
source /opt/ros/jazzy/setup.bash
cd ~/robo_work_teste/robo
source install/setup.bash
ros2 launch robo_lidar_odometry lidar_odometry.launch.py

## Terminal 5 — SLAM Toolbox
source /opt/ros/jazzy/setup.bash
cd ~/robo_work_teste/robo
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/robo_work_teste/robo/slam_test/slam.yaml \
  use_sim_time:=false

## Terminal 6 — Nav2 (salvando o log)
source /opt/ros/jazzy/setup.bash
cd ~/robo_work_teste/robo
source install/setup.bash
ros2 launch robo_navigation navigation.launch.py 2>&1 | tee ~/nav_log.txt

> Configurações do Nav2 (em `src/robo_navigation/config/`):
> - `nav2.yaml` (padrão): NavFn — usar durante o MAPEAMENTO.
> - `nav2_mapa_fixo.yaml`: igual, mas o costmap global usa só o mapa salvo
>   — usar com o SLAM em LOCALIZAÇÃO (ver "Modo competição" abaixo).
> - `nav2_smac.yaml`: Smac Lattice (testado; pior que o NavFn nesta base).

> Depois de (re)iniciar SLAM, bridge ou odometria: clicar em **Reset** no
> RViz (canto inferior esquerdo).

> ⚠️ **Reiniciou a odometria (Terminal 4)? Reinicie também o SLAM (5) e o
> Nav2 (6), nessa ordem.** A odometria volta para (0, 0) ao reiniciar; com o
> SLAM antigo rodando, o robô "salta", o mapa é corrompido (paredes no lugar
> errado, mapa crescendo) e o Nav2 dá erros de TF e `collision ahead`.

---

## Modo competição: mapear uma vez, depois navegar com o mapa fixo

Com o SLAM mapeando, cada passagem pelo corredor "engrossa" as paredes no
mapa e o corredor estreito acaba fechando para o planejador. Com o mapa
salvo e congelado, isso não acontece.

**1. Mapear** (Terminais 1–6 normais, `slam.yaml` + `nav2.yaml`): percorrer a
arena devagar, sem pessoas perto do LiDAR. Anotar onde o robô começou.

**2. Salvar o mapa** (com o SLAM ainda rodando):
```
mkdir -p ~/robo_mapas
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/marcos/robo_mapas/arena'}"
```
(gera `arena.posegraph` e `arena.data` em `~/robo_mapas/`)

**3. Navegar com o mapa fixo** — colocar o robô no MESMO lugar e direção em
que o mapeamento começou, subir Terminais 1–4 normais e trocar o 5 e o 6:

Terminal 5 (SLAM em localização — `use_sim_time:=false` é obrigatório):
```
ros2 launch slam_toolbox localization_launch.py slam_params_file:=$HOME/robo_work_teste/robo/slam_test/slam_localizacao.yaml use_sim_time:=false
```
Terminal 6 (Nav2 com mapa fixo):
```
ros2 launch robo_navigation navigation.launch.py config:=nav2_mapa_fixo.yaml 2>&1 | tee ~/nav_log.txt
```
Se o robô aparecer fora do lugar no RViz, corrigir com **2D Pose Estimate**
(clicar onde o robô está e arrastar na direção para onde ele aponta).

---

## Quando precisa recompilar

- Mudou `.cpp` (ex.: `robo_bridge`): `colcon build --packages-select robo_bridge`
  dentro de `~/robo_work_teste/robo` e reiniciar o nó.
- Mudou `.yaml` de `src/` ou `slam_test/`: não precisa compilar
  (symlink-install), só reiniciar o nó que usa o arquivo.
- **Exceção — odometria (`robo_lidar_odometry`)**: está instalada no modo
  normal. Mudou o código Python ou o `lidar_odometry.yaml` dela? Recompile
  SEM `--symlink-install` (com ele o nó não inicia: `PackageNotFoundError`):
  `colcon build --packages-select robo_lidar_odometry`
- Mudou o firmware: gravar pela Arduino IDE.

## Comandos úteis

Recomeçar o mapa sem derrubar o SLAM (robô parado, sem goal ativo):
```
ros2 service call /slam_toolbox/reset slam_toolbox/srv/Reset "{pause_new_measurements: false}"
```

Salvar o mapa do SLAM:
```
mkdir -p ~/robo_mapas
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/marcos/robo_mapas/arena'}"
```

Limpar os costmaps:
```
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

Conferir se a odometria por LiDAR está válida (útil durante giros):
```
ros2 topic echo /lidar_odometry/valid
```

---

## Mapeamento com controle (protótipo, opcional)

Terminais 1 a 5 normalmente, **sem o Terminal 6 (Nav2)** — os dois
publicariam em /cmd_vel ao mesmo tempo. Detalhes em
`~/robo_work_teste/prototipo/robo_controle/README.md`.

source /opt/ros/jazzy/setup.bash
cd ~/robo_work_teste/prototipo
colcon build   # só na primeira vez ou quando mudar o protótipo
source install/setup.bash
ros2 launch robo_controle controle_mapeamento.launch.py

(sem controle: `ros2 run robo_controle teclado_node` num terminal próprio)
