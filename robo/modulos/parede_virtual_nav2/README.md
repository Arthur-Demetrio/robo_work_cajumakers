# Parede virtual por fita para Nav2

Módulo independente do Robo-WORK com o funcionamento conferido em **13/11/2026**:
detecção de fita vermelha/branca, projeção métrica na região calibrada, máscara
keepout nos costmaps e retirada da parede quando a fita é removida do chão visível.

- Confirma a parede após três observações compatíveis.
- Com robô parado e trecho visível, retira após dois segundos sem fita.
- Durante deslocamento ou fora da imagem, conserva a parede em memória.
- Não utiliza máscara fixa ou rastreamento dinâmico da garra; posicionar a garra
  fora da área observada durante o mapeamento da fita.

O operador confirmou o funcionamento da retirada no ensaio real. Isso não
equivale a validar todas as iluminações, poses ou navegação em movimento.
O protótipo dinâmico da garra foi abandonado nesse ensaio e não está incluído.

## Construir somente este módulo

Requer ROS 2 Jazzy, OpenCV, NumPy e dependências ROS declaradas no package.xml.

```bash
cd modulos/parede_virtual_nav2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --packages-select robo_virtual_wall
source install/local_setup.bash
```

Use explicitamente --base-paths src. O COLCON_IGNORE na raiz do módulo evita
que uma compilação do repositório inteiro descubra duas cópias do pacote
robo_virtual_wall, pois src/ da raiz do repositório preserva a versão histórica.
As instalações ficam dentro deste módulo. Não substituem instalações operacionais.

## Abrir apenas a prévia da câmera

Feche qualquer outro processo que esteja usando a câmera. Depois de construir:

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
tape_share="$(ros2 pkg prefix robo_virtual_wall)/share/robo_virtual_wall"
ros2 launch robo_virtual_wall vision.launch.py \
  device:=/dev/video2 show_windows:=true roi_top:=0.0 \
  preview_homography_file:="$tape_share/calibration/front_diagnostic_registered_20261113_212518.json"
```

A prévia não publica paredes métricas. Clique numa marca para conferir a distância.
R salva imagem/JSON; M registra a medida da trena; q fecha. As capturas vão para
~/.ros/virtual_wall/captures. O nó aceita capture_directory para mudar esse local.
HSV incluído em config/hsv_fita.json; para ajustar uma cópia, use o parâmetro
hsv_file do executável virtual_wall com calibrate=true e show_windows=true.

## Visão métrica e memória de fita, sem iniciar Nav2

Com base, odom, mapa e TF já disponíveis:

```bash
ros2 launch robo_virtual_wall observation_tape.launch.py \
  homography_file:="$tape_share/calibration/front_base_link_expanded_validated_20261113.json" \
  preview_homography_file:="$tape_share/calibration/front_diagnostic_registered_20261113_212518.json"
```

Não executar ao lado de outro virtual_wall/tape_keepout. Este comando inicia
somente visão e keepout. Não inicia base, SLAM ou servidores Nav2 e não manda
goals ou velocidades. Veja a [integração Nav2](src/robo_virtual_wall/NAV2.md).

RViz de teste, com máscara, costmaps, /plan e goal pela interface alinhada:

```bash
rviz2 -d "$tape_share/config/tape_nav2_test.rviz"
```

## Calibração e alcance

A calibração incluída é específica da câmera, foco e montagem usados em
13/11/2026, com imagem raw **640×480** e foco automático desligado. Mudança de
montagem, enquadramento, foco ou resolução exige nova conferência/calibração.
O módulo não altera os controles físicos da câmera automaticamente.

As marcas de ajuste cobrem aproximadamente 39–110 cm à frente da borda branca,
com largura variável. As coordenadas base_link acrescentam o deslocamento
medido de 0,365 m dessa borda. A região válida é o polígono medido, não um
retângulo inteiro nem todo o campo visível. Sem extrapolação além do domínio.

As três conferências independentes deram erros de 1,26 / 0,55 / 1,05 cm.
O resíduo máximo do ajuste das 13 marcas foi 2,40 cm; não foi escondido ou
convertido em precisão garantida de toda a região. Não há retificação de lente.
JSONs de 06/11 permanecem apenas como dados de regressão dos testes.

## Testar

```bash
bash scripts/validar.sh
```

Testes usam domínios ROS isolados e imagens de referência; não movimentam o robô.
Resultados desta cópia e evidências do ensaio estão em [docs](docs/VALIDACAO.md).

## Limites da remoção

Odometria precisa estar recente. Um segundo parado antecede a contagem de dois
segundos de ausência, com ao menos seis imagens válidas. Se o robô se mover ou
o trecho sair da câmera, a contagem é cancelada. A parede não expira ao se afastar.
Uma observação parcial compatível conserva o segmento maior já confirmado.

Ausência visual não distingue fita retirada de fita coberta por um objeto claro.
Manter chão e câmera desobstruídos no ensaio. Falha da câmera não gera comando
de parada; a máscara não substitui Collision Monitor ou saúde da percepção.
Parede é guardada só na memória da sessão, sem associação após loop closure.

## Organização

| Caminho | Conteúdo |
|---|---|
| src/robo_virtual_wall | Pacote ROS 2, separado do restante do robô |
| config/ dentro do pacote | HSV, filtros dos costmaps e RViz |
| calibration/ dentro do pacote | Calibrações atuais e dados históricos de teste |
| test/ dentro do pacote | Regressões de detecção, projeção, memória e remoção |
| docs/ | Validação, evidências e procedência |

Para desativar, encerrar os processos deste módulo. As fontes e instalações
operacionais não são modificadas por construir ou testar esta cópia.
