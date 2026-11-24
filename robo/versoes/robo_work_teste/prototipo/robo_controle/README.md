# Protótipo `robo_controle`: controle remoto para a base mecanum

> Índice de todos os protótipos: [../README.md](../README.md)

> **Status: protótipo standalone, adaptado ao sistema Robo-WORK.** Não faz
> parte de nenhum workspace do robô e não altera nada nele. Conversa só por
> interfaces ROS 2: publica `geometry_msgs/TwistStamped` em `/cmd_vel_nav`
> e `std_msgs/Bool` em `/emergency_stop`.
>
> **Referência:** código em `~/Robo-WORK` (git) e parâmetros em uso em
> `~/robo_jazzy_ws` (workspace operacional, mais novo; conferido em
> 19/10/2026).

## A ideia

Na etapa de **mapping**, é mais fácil e mais seguro **dirigir o robô na mão**
do que mandar goals para o Nav2. O comando entra no mesmo pipeline do Nav2,
então o **Collision Monitor continua protegendo**, como no teleop da equipe
(`~/Robo-WORK/scripts/teleop_geometry_wasd_qe.py`):

```
 controle BT/USB ─▶ joy_node ─/joy─▶ controle_node ─┐
 teclado ─────────────────────────▶ teclado_node ──┤  TwistStamped
                                                    ▼
   /cmd_vel_nav ─▶ velocity_smoother ─▶ /cmd_vel_smoothed ─▶ collision_monitor
                                                                   │
                                     robo_hardware ◀── /cmd_vel ◀──┘
                                     (mecanum + IMU) ─▶ Arduino ─▶ motores

 controle_node ─ emergência ─▶ /emergency_stop (true) ─▶ robo_hardware para TUDO
```

> ⚠️ O comando passa pelo `velocity_smoother`, que vem no
> `navigation.launch.py` (terminal 05). **Sem a navegação rodando, o robô
> não anda.** É de propósito: dirigir sem Collision Monitor seria um caminho
> sem proteção.

### O que mudou em relação à base antiga

| | Base antiga (`robo/`) | Robo-WORK (mecanum) |
|---|---|---|
| Mensagem | `Twist` em `/cmd_vel` | `TwistStamped` em `/cmd_vel_nav` |
| Movimento | frente + giro, curva limitada pela física | frente, **lateral** e giro independentes (diagonal também) |
| Limites | faixa física de cada lado (0,132–0,292 m/s) | os do `velocity_smoother`: **0,08 m/s** (x e y) e **0,14 rad/s** |
| Rumo | sem correção | o `robo_hardware` mantém o rumo com a IMU (BNO055) |
| Parada | só parava de publicar | o botão de emergência aciona o `emergency_stop` do hardware |

Os máximos são os do *smoother* porque, acima deles, ele reduz o vetor
inteiro na mesma proporção (`scale_velocities: true`), e a direção mudaria
sem aviso. Cada eixo começa em 0,016, logo acima da *deadband* do smoother
(0,015), para um toque leve no analógico não virar zero. Detalhes em
[comando.py](robo_controle/comando.py).

## Controles

O padrão do [config/controle.yaml](config/controle.yaml) é o controle do
notebook, um **DragonRise "PC TWIN SHOCK" (USB `0079:0006`)**, mapeado com
o `mapear_controle` em 21/10/2026. Os números abaixo são os índices que o
`joy_node` publica para cada função:

| Função | Índice no DragonRise | Como usar |
|---|---|---|
| **Homem-morto** | botão 8 | **segurar** para o robô andar |
| Modo lento | botão 9 | segurar: metade da velocidade, para encostar em bancadas |
| **Parada de emergência** | botão 0 | apertar: `robo_hardware` para os motores, **inclusive o Nav2**, e fica travado |
| Liberar emergência | botão 2 | **segurar 2 s** |
| Salvar local | botão 1 | apertar: local novo no `robo_locais` (nome automático `C1`, `E1`, `Y1`…) |
| Salvar mapa | botão 3 | apertar: `serialize_map` do slam_toolbox |
| **Finalizar mapeamento** | botão 4 | **segurar 2 s**: salva mapa e locais e, se o objetivo tiver plano, libera a missão autônoma |
| Frente / ré | eixo 1 | analógico (para cima = frente) |
| Lateral (mecanum) | eixo 0 | analógico (para a esquerda = esquerda) |
| Giro no lugar | eixo 3 | analógico (para a esquerda = anti-horário) |

Para saber qual botão físico é cada índice, olhe o `/joy` apertando um de
cada vez (`ros2 topic echo /joy`) ou rode o `mapear_controle` de novo
escolhendo os botões que preferir.

Outras proteções:
- **Sem `/joy` por mais de 0,5 s** (o controle desconectou): para.
- **Cede ao Nav2:** se outro nó (controller do Nav2, `mecanum_escape`)
  estiver publicando em `/cmd_vel_nav`, o controle **não disputa**. Ele
  ignora o homem-morto e avisa no log. Para tomar o controle, cancele o
  objetivo ou use o botão de emergência.

> ⚠️ A parada de emergência do `robo_hardware` **fica ativa até alguém
> publicar `false`**, mesmo se o `controle_node` for fechado. Para liberar
> sem o controle:
> `ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}" --qos-durability transient_local --qos-reliability reliable`

### Usando outro controle

O `controle.yaml` vem com os índices do **DragonRise do notebook**. Cada
modelo de controle numera os botões de um jeito: controles que o SDL
reconhece como *gamepad* (Xbox, PS4/PS5, 8BitDo) usam o layout Xbox
(anotado no YAML); controles genéricos usam a numeração própria
(`ros2 run joy joy_enumerate_devices` mostra `GamePad: false`). Com os
índices errados, uma função perigosa pode cair num botão inesperado (ex.:
"finalizar" no botão que você acha que é o homem-morto).

Antes de usar outro controle, mapeie-o (sem o `controle_node` rodando):

```bash
ros2 run joy joy_node
```

Em outro terminal:

```bash
ros2 run robo_controle mapear_controle
```

A ferramenta pede, um por um, cada botão (homem-morto, lento, emergência,
liberar, salvar local, salvar mapa, finalizar) e cada movimento (frente,
lateral, giro), detecta o índice e se o eixo está invertido, e grava
`~/robo_testes_prototipos/controles/<nome_do_controle>.yaml`. Ela só lê o
`/joy` e se recusa a rodar com o `controle_node` aberto. Depois:

```bash
ros2 launch robo_controle controle_mapeamento.launch.py botoes:=$HOME/robo_testes_prototipos/controles/<nome_do_controle>.yaml
```

O arquivo só sobrepõe os índices; o resto vem do
[config/controle.yaml](config/controle.yaml). Em controles com botão
**ANALOG**, deixe o LED aceso, senão o analógico esquerdo vira direcional.

## Como usar

### 1. Parear o controle Bluetooth (uma vez)

```bash
bluetoothctl
# dentro do bluetoothctl:
#   power on
#   scan on            (coloque o controle em modo de pareamento)
#   pair XX:XX:XX:XX:XX:XX
#   trust XX:XX:XX:XX:XX:XX
#   connect XX:XX:XX:XX:XX:XX
```

Confira se ele aparece com `ls /dev/input/js*`. Com cabo USB, basta
conectar.

### 2. Compilar (fora do workspace do robô)

```bash
source /opt/ros/jazzy/setup.bash
cd /home/marcos/robo_work_teste/prototipo
colcon build
source install/setup.bash
```

### 3. Mapear

Suba o robô pelos terminais da equipe (`~/Robo-WORK/terminais/`), na ordem:
01 hardware → 02 LiDAR → 03 localização → 04 mapeamento → 05 navegação. Em
cada terminal do protótipo, carregue **os dois** ambientes:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
source ~/robo_work_teste/prototipo/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch robo_controle controle_mapeamento.launch.py
```

Sem controle, dá para usar o teclado, **num terminal separado**:

```bash
ros2 run robo_controle teclado_node
```

`w`/`s` frente e ré, `q`/`e` lateral, `a`/`d` giro no lugar, `t` modo
lento, espaço para parar (mesmas teclas do teleop da equipe).

**Dicas para um bom mapa:**
- sem encoders, a odometria (`robo_localization`) **integra o comando**:
  patinar ou empurrar o robô com a mão deforma o mapa. Ande devagar e sem
  trancos;
- passe perto das paredes e cantos e feche voltas (volte a lugares já
  vistos), para o SLAM corrigir o erro acumulado;
- a lateral é ótima para encostar de frente numa bancada sem girar;
- no meio do caminho, aperte **salvar mapa** (botão 3) (seguro, não libera
  nada).

### 4. Marcar locais e finalizar, só com o controle

Com o `robo_locais` rodando
(`ros2 launch robo_locais locais.launch.py autonomo:=true modo:=nav2 permitir_movimento:=true`):

1. Defina o tipo dos próximos pontos (vale até você mudar):
   `ros2 param set /locais tipo_atual coleta`. Se for marcar pontos dentro
   de uma área: `ros2 param set /locais area_pai_atual Y`.
2. (Opcional) anote o que tem e o que vai para o próximo ponto:
   `ros2 param set /locais objetos_atual "caixa,caixa"`.
3. Pare o robô no ponto, virado para onde ele deve ficar, e aperte **salvar local** (botão 1). O
   log mostra o nome gerado (ex.: `local 'C1' (nome automatico)`). Para
   bancadas, prefira marcar pela borda no RViz (`/locais/marcar_bancada`,
   ver README do `robo_locais`), que já calcula a pose de aproximação.
4. Terminou a arena: **segure finalizar (botão 4) por 2 s**. O mapa e os locais são
   salvos sempre. Se o objetivo tiver plano e a pegada do robô couber em
   cada local, o `autonomo_node` começa a missão 3 s depois. Se faltar
   algo, o log mostra o que corrigir e o robô não se move.

> ⚠️ Depois de finalizar, **solte o homem-morto**. Durante a missão, o
> **botão de emergência** pausa tudo (a missão espera) e **liberar
> segurado** retoma.

## Registro dos testes

Cada vez que o `controle_node` roda, ele grava uma **sessão de controle**
em `~/robo_testes_prototipos/controle/AAAA-MM-DD_HH-MM-SS_controle.jsonl`
(um evento JSON por linha, gravado na hora: se o nó cair, nada se perde).

| Evento | Quando | Dados |
|---|---|---|
| `inicio` | ao subir | tópicos, limites e eixos em uso |
| `trecho` | ao parar de dirigir | duração, maior `vx`/`vy`/`w`, movimentos usados (frente/lateral/giro), modo lento |
| `emergencia_acionada` / `emergencia_liberada` | emergência / liberar segurado | se estava dirigindo; quanto tempo ficou parado |
| `cedeu_ao_nav2` / `parou_de_ceder` | homem-morto com outro nó comandando | duração do episódio |
| `controle_conectado` / `controle_desconectado` | `/joy` voltou / parou | se estava dirigindo |
| `salvar_local`, `salvar_mapa`, `finalizar` | salvar local, salvar mapa, finalizar | sucesso e mensagem (só a linha principal e os PROBLEMAS/AVISOS) |
| `erro` | o nó caiu com exceção | a mensagem do erro |
| `resumo` | ao fechar (Ctrl+C) | totais: trechos, tempo dirigindo, máximos, emergências, desconexões, locais/mapas/finalizações |

Para ler a última sessão (ou um arquivo/pasta específico):

```bash
ros2 run robo_controle ver_registro
```

Sem a linha `resumo` no fim, a sessão não terminou normalmente (o nó
caiu): o `ver_registro` avisa. Outra pasta: `pasta_registro:=<pasta>`;
desligar: `pasta_registro:=''`.

## Estrutura

```
robo_controle/
├── README.md
├── robo_controle/
│   ├── comando.py         # analógicos -> (vx, vy, w) mecanum; botão segurado (sem ROS)
│   ├── controle_node.py   # nó do controle: homem-morto, emergência, ceder ao Nav2, extras
│   ├── registro.py        # registro da sessão (JSONL) + ver_registro (sem ROS)
│   ├── mapear.py          # mapear_controle: descobre os botões/eixos do seu controle
│   └── teclado_node.py    # alternativa pelo teclado, mesma conversão
├── config/controle.yaml   # botões, eixos e limites
├── launch/controle_mapeamento.launch.py   # joy_node + controle_node
└── test/                  # test_comando.py, test_registro.py, test_mapear.py (sem ROS)
```

Testes:

```bash
cd /home/marcos/robo_work_teste/prototipo/robo_controle
python3 -m pytest -q test/
```

## Pontos de extensão

| Futuro | Como |
|---|---|
| Limites diferentes no Nav2 | ajustar `limites.*` no YAML para bater com `velocity_smoother.max_velocity` do `~/robo_jazzy_ws` |
| Prioridade do controle sobre o Nav2 | um `twist_mux` na frente do `velocity_smoother`, com o controle em prioridade maior (hoje o controle só cede) |
| Mais botões (ex.: `smart_spin`, cancelar objetivo) | novo índice no YAML + ação em `ControleNode.on_joy` |

## Limitações conhecidas

- Não testado com o robô andando. A lógica foi testada com mensagens
  `/joy` simuladas e checando o que sai em `/cmd_vel_nav` e
  `/emergency_stop`; os índices do DragonRise vieram do `mapear_controle`
  com o controle físico.
- Os índices padrão valem só para o DragonRise do notebook. Outro controle
  precisa do `mapear_controle` (o `joy_node` não traduz controles genéricos
  para um layout comum).
- "Ceder ao Nav2" compara o que chega em `/cmd_vel_nav` com o último
  comando enviado pelo próprio nó. Comandos zerados de outros nós não são
  detectados (e não precisam ser: não movem o robô).
- O teclado depende da repetição de tecla do sistema. Soltou a tecla, ele
  para depois de ~0,6 s.
