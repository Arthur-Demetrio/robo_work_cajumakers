# Especificações técnicas — referência

## Base e processamento
- Base mecanum com quatro motores e quatro pontes H.
- Arduino Mega 2560, interface USB CH340.
- ROS 2 Jazzy em Linux Mint 22.2, workspace relatado: `~/robo_jazzy_ws`.
- IMU BNO055 por I2C (endereço anteriormente informado: 0x29).
- LiDAR com frame histórico `laser_raw`.

## Mapeamento dos motores (relatado)
| Motor | Posição | RPWM | LPWM |
|---|---|---:|---:|
| M1 | dianteiro direito | 12 | 11 |
| M2 | dianteiro esquerdo | 10 | 9 |
| M3 | traseiro direito | 8 | 7 |
| M4 | traseiro esquerdo | 6 | 5 |

## Geometria
Veja [geometria.md](./geometria.md). Dimensões e TF são referências históricas, não confirmação de medições atuais.

## Software conhecido das sessões
- Hardware/interface Arduino, integração LiDAR, localização/odometria, Nav2, SLAM Toolbox.
- Recuperação lateral mecanum e spin progressivo foram discutidos/testados.
- **Sem encoders** nas configurações discutidas: a odometria exige atenção especial para deriva e estimação de deslocamento.

## Pontos a confirmar no equipamento
Tensão da alimentação sob carga, pinagem efetiva, modelo do LiDAR, portas, versão de cada firmware, referência de eixos TF e proteção elétrica. Não usar este documento como diagrama de ligação de potência.
