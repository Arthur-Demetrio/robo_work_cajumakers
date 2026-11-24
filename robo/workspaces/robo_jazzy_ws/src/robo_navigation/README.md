# robo_navigation 1.0.0

Primeira configuracao de mapeamento do robo mecanum com ROS 2 Jazzy e
`slam_toolbox`.

## Entradas esperadas

- `/scan_front` (`sensor_msgs/msg/LaserScan`)
- TF `odom -> base_link`
- TF `base_link -> laser_raw`

## Saidas principais

- `/map`
- TF `map -> odom`

## Iniciar o mapeamento

Com `robo_hardware`, `robo_lidar` e `robo_localization` ja executando:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
ros2 launch robo_navigation mapping.launch.py
```

No RViz, use `map` como Fixed Frame e adicione o display `Map` com o topico
`/map`.

## Observacao importante

A posicao linear ainda e estimada a partir de `/cmd_vel`, pois o robo nao tem
encoders. O BNO055 fornece a orientacao real, e o scan matching do
`slam_toolbox` corrige parte do erro linear. Patinagem e deslocamentos manuais
nao sao medidos pela odometria e podem deformar o mapa.
