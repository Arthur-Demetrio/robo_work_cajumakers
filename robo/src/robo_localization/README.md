# robo_localization 1.0.0

Odometria provisoria para o robo mecanum sem encoders.

## Funcionamento

- recebe velocidades da base em `/cmd_vel`;
- recebe orientacao e velocidade angular do BNO055 em `/imu/data`;
- integra as velocidades lineares no referencial indicado pelo BNO055;
- publica `/odom` (`nav_msgs/msg/Odometry`);
- publica a TF dinamica `odom -> base_link`;
- interrompe a integracao quando o comando ou a IMU ficam antigos;
- oferece o servico `/reset_odometry`.

## Limitacao importante

Sem encoders, `x` e `y` representam deslocamento comandado, nao deslocamento
medido. Patinacao, piso irregular, bateria e diferencas entre motores causam
erro acumulado. O BNO055 fornece uma orientacao muito melhor, mas nao corrige a
distancia linear sozinho.

## Compilacao

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robo_localization --executor sequential
source install/setup.bash
```

## Execucao

Primeiro inicie o hardware. Depois:

```bash
ros2 launch robo_localization localization.launch.py
```

## Verificacao

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Para zerar a pose na posicao atual:

```bash
ros2 service call /reset_odometry std_srvs/srv/Trigger '{}'
```

Os parametros de escala `linear_scale_x` e `linear_scale_y` comecam em 1.0 e
serao calibrados medindo o deslocamento real do robo.
