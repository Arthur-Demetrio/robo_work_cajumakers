# Robô completo: base, garra e câmera

Como subir o robô inteiro (base móvel + garra + câmera) para uma prova com coleta automática de cubos.

Tudo fica num repositório só: [Arthur-Demetrio/robo_work_cajumakers](https://github.com/Arthur-Demetrio/robo_work_cajumakers).

| Pasta | O que é | Origem |
|---|---|---|
| `garra/` | garra (MoveIt, driver, firmware) e os scripts da coleta | antigo [Arthur-Demetrio/garra](https://github.com/Arthur-Demetrio/garra) |
| `robo/` | base mecanum: hardware, LiDAR, SLAM, Nav2, executor da prova | antigo [OttavioSilva/Robo-WORK](https://github.com/OttavioSilva/Robo-WORK) |
| `camera/` | visão dos cubos (`cube_vision`, AprilTag 36h11) | antigo [niviton/-WORK](https://github.com/niviton/-WORK) |
| `teleop/` | teleop por controle Xbox/PS5 para mapear e marcar pontos (seção 0.5) | novo |

O `executor_prova.py` do `robo/` já chama a garra no PEGAR (patch em `garra/integracao_robo/`).

## Visão geral da ordem

```text
1. Base (robo)   ponte → LiDAR → odometria → SLAM → mapa no START → Nav2 + árbitro
2. Garra         MoveIt + driver dos servos (garra_real.launch.py)
3. Câmera        camera_cubo.py (padrão) ou cube_vision + TF da câmera
4. Coleta        pegar_cubo.py + ajuste_base.py + alcance_cubo.py (coleta.launch.py)
5. Prova         executor_prova.py tarefa.yaml
```

Cada etapa depende da anterior:

- a base precisa estar **PRONTA** antes de tudo: o Nav2 recusa goals se subir antes de `/odom` e do mapa;
- o `ajuste_base.py` só consegue mover a base com o **árbitro** do robô no ar (`/navegacao/especial`), e
  o árbitro só sobe no perfil **supervisor** (`--supervisor`);
- o `pegar_cubo.py` espera o MoveIt da garra;
- o executor só pede a coleta depois de chegar na mesa, e o `alcance_cubo.py` só age nesse pedido.

Use o mesmo `ROS_DOMAIN_ID` em todos os terminais (ou nenhum).

## 0. Preparação (uma vez por computador)

### 0.1 Dependências

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox ros-jazzy-behaviortree-cpp ros-jazzy-tf-transformations \
  ros-jazzy-usb-cam ros-jazzy-vision-msgs
```

Conferir: `rosdep check --os=ubuntu:noble --from-paths <pasta> --ignore-src -r` em cada pasta (`garra/`, `robo/`, `camera/`)
(no Linux Mint é preciso o `--os=ubuntu:noble`).

### 0.2 Caminhos que o robô espera

Os scripts do robô usam caminhos fixos na home: `~/robo_jazzy_ws`, `~/subir_robo.sh` e `~/mapas/`.
Aponte-os para esta pasta com links (não altera os arquivos do repositório):

```bash
ln -s ~/work/robo/workspaces/robo_jazzy_ws ~/robo_jazzy_ws
ln -s ~/work/robo/operacao/subir_robo.sh ~/subir_robo.sh
mkdir -p ~/mapas
```

Em `~/mapas/` ficam o mapa (`<mapa>.posegraph` e `.data`), o `<mapa>_pontos.yaml` (START/FINISH) e o
`service_areas.yaml` (mesas). Os `.yaml` estão em `robo/mapas_operacionais/`; os mapas completos
(`.posegraph`/`.data`) estão nos arquivos da release `snapshot-20261112` do Robo-WORK.

### 0.3 Compilar

```bash
source /opt/ros/jazzy/setup.bash

# Garra
cd ~/work/garra && colcon build

# Base: instalação operacional (install/) e overlay do supervisor (install_nav2_principal/),
# que o perfil --supervisor exige (veja robo/workspaces/robo_jazzy_ws/AGENTS.md)
cd ~/robo_jazzy_ws && colcon build
colcon build --build-base build/nav2_principal --install-base install_nav2_principal

# Câmera (só para o cube_vision): ambiente virtual com OpenCV >= 4.8
cd ~/work/camera && python3 -m venv --system-site-packages .venv
source .venv/bin/activate && pip install "opencv-contrib-python>=4.8,<4.12" "numpy<2"
cd ros2_ws && python -m colcon build --packages-select cube_vision
deactivate
```

Compile o `cube_vision` com `python -m colcon` e o `.venv` ativo, para o nó usar o OpenCV novo. O resto
usa o Python do sistema (OpenCV 4.6 do ROS); não instale o OpenCV novo fora do `.venv`.

### 0.4 Antes de cada sessão de mapa

Com o robô parado no START, grave o START/FINISH uma vez:

```bash
python3 ~/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova.py --marcar-start
```

### 0.5 Mapear e marcar pontos com o controle (Xbox Series X ou PS5)

O [teleop/teleop_mapeamento_joy.py](teleop/teleop_mapeamento_joy.py) dirige o robô pelo controle durante o
mapeamento e marca START, FINISH, WS (workstation), SH (shelf) e PP (precision) pela pose `map→base_link`.
Ele substitui o teleop de teclado e o `--marcar-start` acima.

Antes: base no ar (etapa 1, com SLAM ou localização) e **nenhum goal do Nav2 ativo**. Conecte o
controle por USB ou Bluetooth; ele aparece em `/dev/input/js0` e o modelo é detectado sozinho.

```bash
python3 ~/work/teleop/teleop_mapeamento_joy.py
```

**Para dirigir**

| Xbox / PS5 | Ação |
|---|---|
| **LB / L1** (segurar) | Homem-morto: o robô só anda com ele apertado e para assim que você solta |
| **Analógico esquerdo** | Anda para frente/ré e de lado (lateral do mecanum) |
| **Analógico direito** (só horizontal) | Gira o robô |
| **RB / R1** (segurar) | Deixa 2× mais rápido |
| **B / Círculo** | Parada: zera a velocidade e trava até você soltar todos os botões e analógicos |

**Para marcar pontos**

| Xbox / PS5 | Ação |
|---|---|
| **D-pad ↑ ↓** | Escolhe o tipo: START → FINISH → WS (workstation) → SH (shelf) → PP (precision) |
| **D-pad ← →** | Escolhe o número (WS_1, WS_2, … até 9). Não vale para START/FINISH |
| **A / Cruz** | Marca a pose atual do robô no item escolhido |
| **Y / Triângulo** | Apaga o item escolhido. É preciso apertar duas vezes em até 2 s |
| **Menu / Options** | Grava os arquivos em `~/mapas/` |
| **View / Create** | Lista no terminal tudo o que já foi marcado |

Exemplo: D-pad ↓ até `WS`, → até `WS_2`, leve o robô de frente para a mesa a uns 60 cm e aperte **A**;
no fim, **Menu** para gravar. O terminal mostra sempre o item escolhido e se ele já foi marcado.

- START/FINISH vão para `~/mapas/pontos.yaml`; WS/SH/PP para `~/mapas/service_areas.yaml` (as áreas que já
  estão no arquivo são mantidas). Outros caminhos: `--pontos` e `--service-areas`.
- Para marcar uma mesa, pare o robô **de frente para ela, a uns 60 cm**: a pose do robô vira a
  `aproximacao` e a `face` é estimada 0,60 m à frente (`origem: teleop`, ajuste com `--dist-face`). Se o
  alinhador precisar da face exata, remarque com o `marcar_mesa.py`, que mede pelo LiDAR.
- Velocidades padrão: 0,10 m/s frente/ré, 0,08 m/s lateral, 0,30 rad/s giro (`--linear`, `--lateral`,
  `--angular`). Publica em `/cmd_vel_nav`; o Collision Monitor continua ativo.
- Ctrl+C sai publicando velocidade zero e pergunta se grava o que ainda não foi gravado.
- Botões trocados? Rode com `--mostrar-eventos` para ver a numeração. Xbox por Bluetooth sem o driver
  xpadneo: `--controle xbox_bt`.

## 1. Base móvel (terminal 1)

Robô parado no START, virado como quando o START foi marcado:

```bash
~/subir_robo.sh --supervisor --localization arena_04-11_1235
```

O script sobe, cada parte esperando a anterior:
ponte (`robo_hardware`, espera `/imu/data`) → LiDAR (`/scan_front`) → localização (`/odom`) →
slam_toolbox → carrega o mapa no START (`deserialize_map`) → confere `map → odom` →
Nav2 com supervisor e árbitro (espera o `bt_navigator` ativo) → limpa os costmaps.

Espere aparecer **`ROBO PRONTO`** e a linha **`CONFIRMADO: nav2_principal=true ... arbitro presente`**.
Confira no RViz se o scan bate com o mapa. Logs em `~/logs/<hora>/`.

- `--supervisor` é **obrigatório** para a coleta: sem ele (arquitetura legada, o padrão do script)
  não há árbitro e o `ajuste_base.py` responde `falhou: sem autoridade especial no arbitro`.
- `--localization <mapa>` usa o mapa salvo sem alterá-lo. Sem essa opção o SLAM roda em modo
  mapping (o README do robô relata saltos de localização nos dois modos; confira no RViz).
- `~/subir_robo.sh --novo` sobe sem mapa, para mapear.

## 2. Garra (terminal 2)

```bash
source /opt/ros/jazzy/setup.bash && source ~/work/garra/install/setup.bash
export GARRA_PORTA=/dev/ttyUSB0      # porta do Arduino Mega da garra
ros2 launch garra_moveit_config garra_real.launch.py use_rviz:=false
```

Espere os controladores e o `move_group` subirem.

Cuidado com a porta: a base usa `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (conversor CH340).
Com as duas placas ligadas, os nomes `/dev/ttyUSB*` podem trocar entre reconexões; prefira o caminho
fixo da garra em `ls /dev/serial/by-id/` no `GARRA_PORTA`.

## 3. Câmera

**Opção A: `camera_cubo.py` (padrão).** Não precisa de terminal próprio: sobe junto com a coleta (etapa 4)
e abre uma janela com a imagem.

**Opção B: `cube_vision` do -WORK (terminal 3).**

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/camera/.venv/bin/activate
cd ~/work/camera/ros2_ws && source install/setup.bash
export PYTHONPATH=$(realpath ..):$PYTHONPATH
ros2 launch cube_vision cube_vision.launch.py video_device:=/dev/video0
```

Ligue a webcam direto no computador (não em hub USB). Na opção B, a TF da câmera sobe com a coleta.

## 4. Coleta (terminal 4)

```bash
source /opt/ros/jazzy/setup.bash && source ~/work/garra/install/setup.bash
ros2 launch garra_coleta coleta.launch.py                 # opção A (camera_cubo.py)
ros2 launch garra_coleta coleta.launch.py camera:=vision  # opção B (cube_vision)
```

Sobe o `pegar_cubo.py`, o `ajuste_base.py`, o `alcance_cubo.py` e a câmera (A) ou a TF da câmera (B).
Cada parte também tem launcher próprio; veja a seção 7.5 de `garra/README.md`.

A coleta fica parada até o executor pedir (`/coleta/pedido`): com o robô navegando, a câmera é ignorada
e a base não se mexe por conta da garra.

## 5. Prova (terminal 5)

```bash
source /opt/ros/jazzy/setup.bash && source ~/robo_jazzy_ws/install/setup.bash
cd ~/robo_jazzy_ws/src/robo_navigation/scripts
python3 executor_prova.py tarefa.yaml --so-plano     # confere o plano, não mexe no robô
python3 executor_prova.py tarefa.yaml                # executa
```

Em cada mesa de coleta, o executor publica `/coleta/pedido` e espera `/coleta/resultado` (até 120 s,
`--tempo-coleta`). O `alcance_cubo.py` ajusta a base (até 4 vezes, no máximo 15 cm cada) e chama a garra
(até 2 tentativas). Se a coleta falhar, o executor registra e segue a prova. O SOLTAR ainda é simulado.
`--sem-garra` volta ao PEGAR simulado.

## Desligar

Na ordem inversa: Ctrl+C no executor (5), na coleta (4), na câmera (3), na garra (2) e por último no
`subir_robo.sh` (1), que para a base na ordem inversa da subida.

## Testes por partes

- **Coleta sem o robô:** etapas 2–4 e
  `ros2 topic pub --once /coleta/pedido std_msgs/msg/String "{data: 'teste'}"` com um cubo à vista.
  Sem a base, use `coleta.launch.py base:=false` (cubo já ao alcance) ou o `ajuste_base` com
  `sem_arbitro:=true` em simulação.
- **Ajuste da base sozinho** (o robô ANDA): etapa 1, `ros2 launch garra_coleta ajuste_base.launch.py`
  e `ros2 topic pub --once /robo/mover geometry_msgs/msg/Pose2D "{x: 0.03}"`, com alguém pronto para parar.
- **Garra em simulação:** `coleta.launch.py garra:=demo camera:=nenhuma base:=false sempre_ativo:=true` e
  publicar a posição em `/cubo_detectado`.

## Pendências conhecidas

- A integração completa (executor → coleta → garra) ainda não rodou no robô; foi conferida com mensagens
  publicadas à mão.
- O frame `robo_base` da garra ainda não está ligado ao `base_link` do robô na árvore TF.
- A garra pega o cubo mais perto da base: os nomes dos objetos da tarefa ainda não são ligados ao ID do AprilTag.
- O `cube_vision` nunca foi rodado com a câmera neste computador.
