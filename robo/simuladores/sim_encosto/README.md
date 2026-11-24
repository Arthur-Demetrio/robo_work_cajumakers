# Bancada de encosto no Gazebo

Roda o `_encostar` **real** do `alinhador_passagem.py` (lido direto do `~/robo_jazzy_ws/src`, sem cópia nem
edição) em malha fechada no Gazebo Harmonic. Depois compara o que o alinhador **disse** com a pose **real**
do robô simulado.

Não fala com o robô: `ROS_DOMAIN_ID=77`, só localhost, `GZ_PARTITION=bancada` e nenhum acesso à serial.
Nada no `robo_jazzy_ws` é alterado.

## Rodar
```bash
cd ~/sim_encosto
python3 bancada.py                          # todos os cenários de cenarios.yaml
python3 bancada.py --so centrado lado_p10   # só alguns
python3 bancada.py --so lado_m15 --rep 3 --gui   # 3 repetições, com a janela do Gazebo
```
Precisa da sessão gráfica aberta (`DISPLAY=:0`): nesta Intel o `gpu_lidar` não funciona sem tela.
Resultado em `resultados/<data>/`: `relatorio.md`, `resultados.json` e os logs de cada nó por cenário
(`<cenario>_alinhador.log` é o log normal do alinhador).

## O que roda
| Nó | Origem |
|---|---|
| `alinhador_passagem.py` | **real**; `alinhador_bancada.py` só acrescenta o gatilho `/sim/encostar` |
| `filtro_corpo_scan.py` | **real** |
| `command_imu_odometry` | **real** (`localization.yaml`): o erro de odometria aparece sozinho |
| `velocity_smoother`, `collision_monitor` | **reais** (`nav2_params.yaml`, overrides do `navigation.launch.py`) |
| `ponte_sim.py` | simula o Arduino (modelo do motor), o Delta-2D, a IMU e o `map→odom` |
| Gazebo | mesas do `~/mapas/service_areas.yaml`, robô cinemático 0,59 × 0,65, `gpu_lidar` de 2° |

## Modelo do motor e sensores (`ponte_sim.py`, `MODELO_PADRAO`)
- **Frente:** a velocidade real segue a `speed_table_x`. Um comando de 0,016 dá ~0,10 m/s, o mínimo do PWM 70.
- **Lado:** `speed_table_y` × 0,60. Medido: 0,05–0,06 m/s real; a odometria superestima.
- **O lado vira avanço:** `vx += 0,33·|vy|`. Deriva de rumo de −0,35°/s andando de lado.
- **Giro parado:** mínimo de 0,15 rad/s, zona morta de 0,02 rad/s. Atraso de 0,10 s e timeout de comando de 0,25 s.
- **LiDAR:** nada abaixo de 0,146 m, ruído de 4 mm, 3% de raios `inf` e rajadas de 3–8 raios, 5,9 Hz.
- **Localização ideal**, atualizada a cada 0,2 s. O `erro_mapa` injeta um erro fixo.

Cada cenário pode sobrescrever o modelo (`modelo:` em `cenarios.yaml`).

## Critério
- **Honesto:** folga de 6 a 12 cm (para-choque no meio até a face), centro dentro de ±3 cm, ângulo dentro de ±3°, tudo pela pose real.
- **Classes:**
  - `SUCESSO_OK`;
  - `SUCESSO_FALSO`: o alinhador disse SUCESSO fora do critério;
  - `FALHA_EXPLICITA`;
  - `COLISAO`: o footprint encostou na mesa ou no obstáculo;
  - `SEM_RESPOSTA`.
- **Avaliação:** `ruim` = colisão, sucesso falso ou sem resposta.

## Limites (o que a simulação NÃO mostra)
- O robô é cinemático. Não há patinada real nem roletes; o deslize vem do modelo acima.
- O scan é instantâneo, sem a distorção de movimento da varredura de 1/5,9 s.
- Sem SLAM real: não aparecem os saltos do slam_toolbox (missao31/32).
- A mesa tem um degrau só, de 1,7 cm, entre as metades. A forma real da face da WS_1 não foi medida.
- Não substitui a validação física (roteiro em `docs/face_fragmentada_20261109/RELATORIO.md`).
