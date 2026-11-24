# Odometria por LiDAR

Alinha cada `/scan_front` ao scan anterior e publica `/odom` e
`odom -> base_link`. Os encoders não são assinados. `/cmd_vel` é usado somente
como estimativa inicial durante o intervalo entre scans; a pose é corrigida
pela correspondência geométrica dos pontos.

O tópico `/lidar_odometry/valid` fica verdadeiro apenas enquanto há scans e
correspondências recentes aceitas. O gate físico cancela a navegação quando
esse sinal é perdido.
