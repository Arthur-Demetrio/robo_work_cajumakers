# robo_lidar 1.1.0

Pacote ROS 2 Jazzy para o LiDAR Delta-2D do robo mecanum.

## Interfaces

- Entrada: `/scan_raw` (`sensor_msgs/msg/LaserScan`)
- Saida filtrada: `/scan_front` (`sensor_msgs/msg/LaserScan`)
- TF estatica: `base_link -> laser_raw`

## Correcao angular do Delta-2D

O diagnostico fisico mostrou que a frente ja correspondia a `0 grau`, mas um
objeto a direita aparecia com angulo positivo. A versao 1.1.0 corrige esse
espelhamento no proprio `LaserScan`:

- reverte `ranges` e `intensities`;
- publica `angle_min = -angle_max` da entrada;
- publica `angle_max = -angle_min` da entrada;
- mantem `angle_increment` positivo.

Com `invert_angles: true`, o resultado esperado segue a convencao ROS:

- frente: aproximadamente `0 grau`;
- esquerda: aproximadamente `+90 graus`;
- direita: aproximadamente `-90 graus`.

## Geometria inicial medida em 18/09/2026

- `x = 0.330 m`
- `y = 0.005 m`
- `z = 0.000 m`, adotando `base_link` na altura dos eixos das rodas
- `yaw = 0.000 rad`, ainda sujeito a verificacao visual no RViz

Todos os valores da TF podem ser substituidos na linha de comando sem editar o
arquivo de lancamento.

## Compilacao

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robo_lidar --executor sequential
source install/setup.bash
```

## Execucao

O hardware deve estar publicando `/scan_raw`.

```bash
ros2 launch robo_lidar lidar_filter.launch.py
```

Exemplo de ajuste temporario da posicao:

```bash
ros2 launch robo_lidar lidar_filter.launch.py \
  lidar_x:=0.330 lidar_y:=0.005 lidar_z:=0.000 lidar_yaw:=0.000
```

## Filtro angular

Os quatro setores validados que continham rodas e motores foram espelhados para
o novo sistema angular e estao configurados em `config/lidar.yaml`. O formato e:

```yaml
blocked_intervals_deg: "-162:-146,-140:-110,100:138,142:160"
```

Os intervalos sao aplicados depois da correcao angular.

## Validacao antes do SLAM

Inicie primeiro o hardware e depois este pacote. Confirme os metadados:

```bash
ros2 topic echo /scan_front --once --qos-reliability best_effort | \
grep -E 'frame_id|angle_min|angle_max|angle_increment|range_min|range_max'
```

Depois teste um objeto separadamente a esquerda e a direita. Somente inicie o
SLAM quando esquerda aparecer positiva e direita negativa.
