# robo_hardware 1.2.0

Ponte serial ROS 2 Jazzy para:

- Arduino Mega 2560/CH340;
- quatro motores mecanum e quatro BTS7960;
- LiDAR Delta-2D;
- BNO055 em `0x29`;
- operação sem encoders nesta etapa.

## Interfaces

Entradas:

- `cmd_vel` — `geometry_msgs/msg/TwistStamped`;
- `emergency_stop` — `std_msgs/msg/Bool`.

Saídas:

- `scan_raw` — `sensor_msgs/msg/LaserScan`;
- `imu/data` — `sensor_msgs/msg/Imu`;
- `imu/yaw_deg` — `std_msgs/msg/Float64`;
- `diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`.

O bridge utiliza `linear.x`, `linear.y` e `angular.z`. A cinemática mecanum
gera quatro PWMs independentes na ordem FL, FR, RL e RR.

## Controle de direção com o BNO055

Durante um comando linear em `x` ou `y`, o bridge registra o yaw atual e
executa um controle PD a 20 Hz. Se o robô desviar para a esquerda, a correção
acrescentada é para a direita, e vice-versa.

O deslocamento lateral usa ganhos próprios e um pequeno feedforward com sinal
oposto ao de `linear.y`. Isso compensa a tendência física observada nos testes:
à esquerda o robô guinava à esquerda; à direita, guinava à direita. Em comandos
diagonais os ganhos frontal e lateral são interpolados.

A autoridade da correção também é limitada pela velocidade linear. Assim, em
um comando lento, a parcela angular não pode dominar a translação nem inverter
o sentido de rodas apenas para corrigir alguns graus.

Não existe espera de 500 ms. A correção começa no ciclo seguinte à detecção do
desvio. Quando `angular.z` ultrapassa `0.02 rad/s`, o controle é desativado e o
giro solicitado passa diretamente para a cinemática. Ao voltar ao movimento
linear, uma nova direção de referência é capturada.

Parâmetros iniciais:

- `heading_hold_kp: 0.8`;
- `heading_hold_kd: 0.05`;
- `heading_hold_max_correction_rps: 0.15`;
- `heading_hold_lateral_kp: 1.2`;
- `heading_hold_lateral_kd: 0.08`;
- `heading_hold_lateral_feedforward_rps: 0.025`;
- `heading_hold_error_deadband_deg: 0.5`;
- `heading_hold_linear_authority_ratio: 0.75`;
- `heading_hold_imu_timeout_s: 0.20`.

Os diagnósticos mostram `heading_hold_active`, modo (`longitudinal`, `lateral`
ou `diagonal`), referência, erro, feedforward, feedback, correção efetiva e
idade da última amostra da IMU. Se a IMU ficar desatualizada, o bridge desativa
a correção e mantém apenas o comando recebido.

Os parâmetros `heading_hold_*` podem ser alterados com `ros2 param set` durante
o teste; nesta versão a alteração passa a atuar de fato no controlador e uma
nova referência angular é capturada com segurança.

O protocolo USB binário usa 500000 baud, cabeçalho `A5 5A`, versão 2 e
CRC-16/CCITT-FALSE. O bridge envia comandos a 20 Hz; o firmware para os motores
após 300 ms sem comando válido.

```bash
ros2 launch robo_hardware hardware.launch.py
```

Esta versão ainda não publica odometria. A odometria continua no pacote
`robo_localization`. Valide o controle lateral antes de avançar ao Nav2.
