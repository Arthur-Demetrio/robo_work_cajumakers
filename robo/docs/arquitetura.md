# Arquitetura — visão inicial

## Plataforma
- Base omnidirecional mecanum, quatro motores, Arduino Mega 2560
- Sensores: LiDAR e IMU BNO055
- Computador com ROS 2 Jazzy
- Navegação: Nav2; mapeamento: SLAM Toolbox

## Fluxo conceitual
LiDAR / IMU → ROS 2 (sensoriamento e localização) → planejamento / Nav2 → comando de velocidade → ponte com Arduino → motores.

Este é um resumo conceitual, **não um diagrama validado dos tópicos e TF atuais**. Confirmar pelo workspace antes de detalhar.
