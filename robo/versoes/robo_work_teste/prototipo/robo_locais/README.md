# Protótipo `robo_locais`: locais de interesse e missões

> Índice de todos os protótipos: [../README.md](../README.md)

> **Status: protótipo standalone, adaptado ao sistema Robo-WORK.** Não faz
> parte de nenhum workspace do robô, não importa nenhum pacote dele e não
> altera nada. Conversa só por interfaces ROS 2: TF `map → base_link`,
> tópicos do RViz, `/global_costmap/costmap`, `/emergency_stop` e as ações
> `NavigateToPose`/`ComputePathToPose` do Nav2. Do `robo_station_manager`,
> só **lê** o arquivo de bancadas salvo por ele.
>
> **Referência:** código em `~/Robo-WORK` (git) e parâmetros em uso em
> `~/robo_jazzy_ws` (workspace operacional, mais novo; conferido em
> 19/10/2026).

## Encaixe no sistema Robo-WORK

| Peça do robô | Como o protótipo usa |
|---|---|
| SLAM (`mapping.launch.py`, terminal 04) + Nav2 (`navigation.launch.py`, terminal 05) rodando **juntos** | locais no frame `map` do SLAM; a missão roda na mesma sessão, logo após o mapeamento |
| `NavigateToPose` com a *behavior tree* do robô (recuperações `mecanum_escape`, `smart_spin`) | o protótipo só manda o objetivo; nunca publica velocidade, então o **Collision Monitor** continua no caminho |
| Velocidades baixas (≤ 0,08 m/s) | o prazo de cada trecho cresce com a distância (`velocidade_media_mps`, `fator_timeout`) |
| `/emergency_stop` do `robo_hardware` | a missão **pausa** enquanto estiver ativo e continua quando liberar |
| `robo_station_manager` (bancada = borda frontal + lado livre) | mesma geometria em `~/marcar_bancada`; `~/importar_bancadas` lê o cadastro dele |
| `/global_costmap/costmap` | ao planejar/finalizar, confere se o robô **cabe** em cada local (mesma regra do `costmap_guard` do station_manager) |
| `allow_motion` do station_manager | equivalente aqui: `permitir_movimento` (padrão `false`) |

## A ideia

Na RoboCup @Work o robô precisa ir a lugares específicos da arena e fazer
algo em cada um, por exemplo:

- no **ponto A**, se alinhar com um objeto;
- na **área X**, pegar um objeto de interesse;
- levar esse objeto até o **ponto Z**, dentro da **área Y**.

Para isso, o protótipo separa o problema em três camadas:

```
 ┌───────────────────────────┐    ┌──────────────────────────────┐
 │ 1. LOCAIS (locais_node)   │    │ 2. MISSÃO (missao_node)      │
 │ marcar pontos e áreas     │───▶│ lista de passos sobre locais │
 │ no mapa durante o mapping │YAML│ ir_para / alinhar / pegar /  │
 └───────────────────────────┘    │ soltar / transportar         │
                                  └──────────────┬───────────────┘
                                                 │ interfaces plugáveis
                          ┌──────────────────────┴───────────────────┐
                          │ 3. EXECUÇÃO                               │
                          │ Navegador: simulado | Nav2                │
                          │ Acoes:     simuladas | (futuro) visão+braço│
                          └───────────────────────────────────────────┘
```

1. **Locais:** durante o mapeamento, a equipe marca os pontos e as áreas de
   interesse. Cada local tem um nome, um tipo, uma **pose de aproximação**
   (onde o robô deve parar e para onde deve estar virado) e, se precisar,
   um **polígono** delimitando a área. Tudo é salvo num YAML **junto com o
   mapa**.
2. **Missão:** uma lista de passos em YAML que faz referência aos locais
   pelo nome. Não tem coordenadas, então a mesma missão serve para qualquer
   mapa que tenha locais com esses nomes.
3. **Execução:** o executor não sabe *como* o robô anda ou pega objetos.
   Isso fica em implementações plugáveis. Hoje existe a versão simulada
   (roda sem robô) e a navegação real pelo Nav2. Alinhar, pegar e soltar
   são ganchos para a visão e o braço preencherem no futuro.

> ⚠️ **Os locais só valem para o mapa em que foram marcados.** Se o SLAM
> recomeçar do zero, a origem do mapa muda e as coordenadas perdem o
> sentido. No Robo-WORK o SLAM e o Nav2 rodam juntos, então o fluxo é:
> mapear, marcar e **executar na mesma sessão**, sem reiniciar o SLAM. O
> robô não tem encoders: a odometria integra o comando, então empurrar o
> robô com a mão entre a marcação e a missão desloca tudo.

## Missão autônoma: o robô decide sozinho

Escrever a missão passo a passo continua possível, mas não é mais
necessário. Durante o mapeamento você anota **o que tem** em cada local e
**o que vai para onde**; ao finalizar o mapeamento o robô planeja e cumpre
o objetivo sozinho.

```
 MAPEAMENTO                       FINALIZAR                 AUTÔNOMO
 marca locais + anota             confere se há plano,      planeja, executa e
 objetos / recebe        ───────▶ salva locais e mapa ────▶ replaneja nas falhas
 (locais_node)                    (~/finalizar_mapeamento)  (autonomo_node)
```

**O que o planejador decide** (`planejador.py`, Python puro):

| Decisão | Como |
|---|---|
| De onde pegar | locais cujo `objetos` contém o objeto; havendo vários, o mais conveniente |
| Onde soltar | o ponto de destino; se for uma **área**, os pontos marcados dentro dela (`area_pai`), espalhando um objeto por ponto |
| Ordem | menor caminho total (busca com poda; a 1ª solução já é a gulosa "vai no mais perto") |
| Quantos de uma vez | parâmetro `capacidade` (com 2, junta coletas e economiza viagens) |
| Custo do caminho | simulado: linha reta; **Nav2: comprimento do caminho do planner**, que contorna paredes |

**Quando algo dá errado**, o executor atualiza o que sabe e replaneja com o
que sobrou:

- não conseguiu **pegar** `max_tentativas` vezes → assume que o objeto não
  está lá e procura em outro local que o tenha;
- não conseguiu **chegar/alinhar/soltar** `max_tentativas` vezes → bloqueia
  o local e usa outro ponto (ex.: outro ponto da mesma área);
- sem alternativa → abandona só aquela entrega, cumpre as outras e volta ao
  `inicio`. O relatório final lista o que foi e o que não foi entregue.

### Anotando o objetivo durante o mapeamento

Além de `nome_atual`/`tipo_atual`, defina (separado por vírgula; nomes
repetidos são unidades diferentes):

```bash
ros2 param set /locais objetos_atual "caixa,caixa,caixa_azul"   # o que há aqui
ros2 param set /locais recebe_atual  "caixa_azul"               # o que vem para cá
```

Vale para a **próxima** pose marcada (botão Y do controle, serviço ou
clique) e depois é limpo, para não passar para o ponto seguinte. Para
corrigir um local já marcado **sem mexer na pose**, defina os parâmetros e
chame `ros2 service call /locais/atualizar_atributos std_srvs/srv/Trigger`
(age no `nome_atual` ou, se vazio, no último local marcado). Campo vazio
mantém o que já estava; `"-"` apaga a lista.

O RViz mostra `tem:` e `recebe:` embaixo do nome de cada local.

Exemplo: na mesa `X` há `[caixa_azul, caixa, caixa]`; o ponto `Z` recebe
`[caixa_azul]`; a área `Y` (com os pontos `Y1` e `Y2`) recebe
`[caixa, caixa]`. Isso é tudo. Ver `config/locais_exemplo.yaml`.

> Alternativa: um YAML de objetivo (ex.: tarefa convertida da RefBox),
> passado em `objetivo:=` / `arquivo_objetivo:=`. Ele só diz o que entregar;
> origens, ordem e caminho continuam com o planejador. Ver
> `config/objetivo_exemplo.yaml`.

### Do mapeamento à execução

```bash
# ambiente: ROS + workspace do robô + protótipos
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
source ~/robo_work_teste/prototipo/install/setup.bash
export ROS_DOMAIN_ID=0

# sobe os dois nós. modo:=nav2 + permitir_movimento:=true para andar de verdade
ros2 launch robo_locais locais.launch.py autonomo:=true modo:=nav2 permitir_movimento:=true

# a qualquer momento: o plano atual faz sentido?
ros2 service call /locais/planejar std_srvs/srv/Trigger

# terminou de mapear:
ros2 service call /locais/finalizar_mapeamento std_srvs/srv/Trigger
```

O `finalizar_mapeamento` **sempre salva** os locais e o mapa do
slam_toolbox (`~/robo_mapas/<mapa>`), mas **só libera a missão se o
objetivo inteiro tiver plano e o robô couber em cada local usado** (pegada
conferida no costmap global). Se faltar algo (objeto sem origem, destino
não marcado, local dentro de área inflada ou desconhecida…), ele responde
`success=False` com a lista de problemas e o robô não sai do lugar;
corrija e finalize de novo. Sem a navegação rodando, o costmap não existe:
a pegada não é conferida e a resposta traz um `AVISO`. Com tudo certo, avisa o
`autonomo_node`, que espera `atraso_inicio_s` (3 s) e começa.

Pelo controle (`robo_controle`): o botão **salvar local** marca um local
novo e **segurar finalizar por 2 s** finaliza. Dá para mapear e disparar a missão sem tocar
no teclado (só os `ros2 param set` de tipo/anotação).

Parâmetros do `autonomo_node`: `iniciar` (`ao_finalizar_mapeamento` |
`imediato` | `manual` via `~/iniciar`), `modo`, `permitir_movimento`,
`capacidade`, `max_tentativas`, `atraso_inicio_s`, `localizador`
(`slam_toolbox`), `timeout_navegacao_s` (mínimo por trecho, 60 s),
`velocidade_media_mps` (0,04), `fator_timeout` (3), `topico_emergencia`.

**Durante a missão:** a parada de emergência (`/emergency_stop`, botão de emergência do
`robo_controle`) cancela o trecho atual e **pausa**; ao liberar (A
segurado), o mesmo trecho é reenviado. O tempo parado não conta como
falha nem no prazo. Ctrl+C no `autonomo_node` cancela o objetivo do Nav2.

> ⚠️ Suba o `autonomo_node` **antes** de finalizar. O aviso não fica
> guardado de propósito: um nó aberto depois não sai andando por causa de
> um mapeamento antigo (para isso use `~/iniciar`).
>
> ⚠️ Durante a missão, o `robo_controle` cede ao Nav2 (não publica mesmo
> com o homem-morto). Para intervir, use o **botão de emergência**.

## Registro dos testes

Cada missão grava um arquivo em `~/robo_testes_prototipos/missao/`:
`AAAA-MM-DD_HH-MM-SS_missao.jsonl` (`autonomo_node`) ou
`…_missao_manual.jsonl` (`missao_node`). Um evento JSON por linha, gravado
na hora: se o nó cair ou for interrompido, tudo até ali está no disco.

| Evento | Dados |
|---|---|
| `inicio` | o que disparou (finalizar, `~/iniciar`, imediato), arquivos de locais/objetivo, mapa, entregas pedidas, modo, `permitir_movimento`, capacidade |
| `plano_inicial` | entregas, passos, custo estimado, PROBLEMAS e AVISOS |
| `passo` | ação, local, objeto, **sucesso**, mensagem de falha e **duração** |
| `plano` | cada (re)planejamento: rodada, ações, custo |
| `objeto_ausente` / `local_bloqueado` | por que o plano mudou (objeto não achado após N tentativas, local que falhou N vezes) |
| `pausa_emergencia` | quanto tempo a missão ficou parada pelo `/emergency_stop` |
| `timeout` / `objetivo_recusado` | trecho do Nav2 que estourou o prazo ou não foi aceito |
| `entregue` / `abandonada` | cada entrega concluída ou desistida, com o motivo |
| `erro` | exceção ou configuração inválida |
| `fim` | situação (`concluida`, `incompleta`, `interrompida`, `nao_iniciada`, `erro`), duração, tempo em emergência, replanejamentos, entregues/abandonadas e, **por tipo de ação**, quantas vezes, falhas e tempo total |

Para ler a última missão (ou um arquivo/pasta específico):

```bash
ros2 run robo_locais ver_registro
```

Sem a linha `fim`, a missão não terminou normalmente: o `ver_registro`
avisa. Outra pasta: `pasta_registro:=<pasta>`; desligar:
`pasta_registro:=''`. Uma missão que nem começou (ex.: modo nav2 sem
`permitir_movimento`) também gera registro, com `situacao: nao_iniciada`
e o motivo.

## Estrutura

```
robo_locais/                     # pacote ROS 2 (ament_python)
    ├── README.md
    ├── robo_locais/
    │   ├── locais.py            # modelo dos locais + YAML (sem ROS)
    │   ├── missao.py            # executor e interfaces plugáveis (sem ROS)
    │   ├── planejador.py        # objetivo -> missão, replanejamento (sem ROS)
    │   ├── pegada.py            # o robô cabe no costmap nessa pose? (sem ROS)
    │   ├── registro.py          # registro da missão (JSONL) + ver_registro (sem ROS)
    │   ├── locais_node.py       # nó: marcar locais/anotações, finalizar mapeamento
    │   ├── missao_node.py       # nó: executa uma missão escrita à mão
    │   └── autonomo_node.py     # nó: cumpre o objetivo sozinho
    ├── config/
    │   ├── locais_exemplo.yaml
    │   ├── objetivo_exemplo.yaml
    │   └── missao_exemplo.yaml
    ├── launch/locais.launch.py
    └── test/                    # test_locais_missao.py, test_planejador.py, test_robo_work.py, test_registro.py
```

`locais.py`, `missao.py`, `planejador.py` e `pegada.py` são Python puro, testáveis sem ROS. Os nós são
só uma casca fina por cima deles.

## Formatos

**Locais** (gerado pelo `locais_node`):

```yaml
frame_id: map
mapa: arena
locais:
  A:
    tipo: alinhamento            # alinhamento | coleta | entrega | passagem | inicio
    pose: {x: 1.2, y: 0.4, yaw: 1.571}
  X:
    tipo: coleta
    pose: {x: 2.1, y: -0.3, yaw: 0.0}
    area: [[2.3, -0.7], [2.9, -0.7], [2.9, 0.1], [2.3, 0.1]]
  Z:
    tipo: entrega
    area_pai: Y                  # Z é um ponto dentro da área Y
    pose: {x: 0.5, y: 1.9, yaw: 3.142}
```

**Missão:**

```yaml
missao:
  - {acao: ir_para, local: A}
  - {acao: alinhar, local: A}
  - {acao: transportar, objeto: M20, de: X, para: Z}
  - {acao: ir_para, local: inicio}
```

`transportar` é expandido automaticamente em: ir_para(de) → alinhar(de) →
pegar(de) → ir_para(para) → alinhar(para) → soltar(para).

## Como usar

### Compilar (fora do workspace do robô)

```bash
source /opt/ros/jazzy/setup.bash
cd /home/marcos/robo_work_teste/prototipo
colcon build
source install/setup.bash
```

### Testes (sem ROS rodando)

```bash
cd /home/marcos/robo_work_teste/prototipo/robo_locais
python3 -m pytest -q test/
```

### Marcar locais durante o mapeamento

Com o robô e o SLAM rodando:

```bash
ros2 launch robo_locais locais.launch.py arquivo:=~/robo_mapas/arena_locais.yaml mapa:=arena
```

1. **Escolha o tipo (e, se quiser, o nome) do local:**
   ```bash
   ros2 param set /locais tipo_atual alinhamento
   ros2 param set /locais nome_atual A      # opcional
   ```
   Para um ponto dentro de outra área: `ros2 param set /locais area_pai_atual Y`.
   `tipo_atual` e `area_pai_atual` valem até você mudar. Já o `nome_atual`
   vale para **uma** marcação e depois é limpo. Sem nome, o local recebe
   um nome automático pelo tipo: `A1` (alinhamento), `C1` (coleta),
   `E1` (entrega), `P1` (passagem), `inicio`; ou `Y1`, `Y2`… se tiver
   `area_pai` `Y`. Assim, apertar Y várias vezes nunca sobrescreve um
   local. Para **reposicionar** um local existente, defina `nome_atual`
   com o nome dele antes de marcar.
2. **Marque a pose**, de uma destas formas:
   - **Robô no lugar (mais preciso):** leve o robô até o ponto, virado
     para onde ele deve ficar, e chame:
     ```bash
     ros2 service call /locais/salvar_pose_atual std_srvs/srv/Trigger
     ```
   - **Clique no RViz:** em *Tool Properties*, troque o tópico do
     **2D Goal Pose** para `/locais/marcar_pose` e clique e arraste no mapa.
     O tópico é outro de propósito: o `/goal_pose` padrão faria o Nav2 sair
     andando.
3. **(Opcional) Desenhe a área:** com a ferramenta **Publish Point** do
   RViz (`/clicked_point`), cada clique vira um vértice do polígono do
   último local marcado (ou do `nome_atual`, se definido). Para recomeçar: `ros2 service call /locais/limpar_area std_srvs/srv/Trigger`.
4. **Visualize:** adicione um display **MarkerArray** no tópico
   `/locais/marcadores` (Durability: Transient Local). Aparecem uma seta,
   o nome, o contorno da área e a borda das bancadas, com uma cor por tipo.

#### Bancadas: marcar pela borda (recomendado)

Para mesas/bancadas, em vez de levar o robô até lá, marque **a bancada** e
deixe a pose de aproximação ser calculada, igual ao `robo_station_manager`:

```bash
ros2 param set /locais tipo_atual coleta     # e, se quiser, nome_atual/objetos_atual
ros2 service call /locais/marcar_bancada std_srvs/srv/Trigger
```

Depois, com o **Publish Point** do RViz, clique: (1) e (2) os extremos da
**borda frontal**; (3) um ponto no **lado livre**, de onde o robô vai
chegar. O robô para de frente para a bancada, com o `base_link` a
`frente_robo_m + folga_bancada_m` (0,365 + 0,20 m) do meio da borda. Uma
borda com menos de 25 cm ou um lado livre colado nela são recusados.

Bancadas **já cadastradas no station_manager** (`station_cli save` grava em
`~/.ros/robo_station_manager/stations_teste.json`) podem ser trazidas sem
redesenhar:

```bash
ros2 service call /locais/importar_bancadas std_srvs/srv/Trigger
```

Cada bancada vira um local com o mesmo nome (`A`, `B`…). Se o local já
existia, a pose e a borda são atualizadas, e tipo e anotações são mantidos.
Só funciona se o cadastro foi feito **no mesmo mapa**.

Outros serviços: `~/listar`, `~/remover` (remove o `nome_atual` ou, se
vazio, o último local marcado) e
`~/salvar_arquivo`. O arquivo também é salvo automaticamente a cada
alteração (`auto_salvar: true`).

### Executar uma missão

Simulado (não precisa de robô nem de Nav2):

```bash
ros2 run robo_locais missao_node --ros-args \
  -p arquivo_locais:=install/robo_locais/share/robo_locais/config/locais_exemplo.yaml \
  -p arquivo_missao:=install/robo_locais/share/robo_locais/config/missao_exemplo.yaml
```

Com o Nav2 (o robô anda de verdade; alinhar, pegar e soltar continuam
simulados):

```bash
ros2 run robo_locais missao_node --ros-args \
  -p modo:=nav2 -p permitir_movimento:=true \
  -p arquivo_locais:=~/robo_mapas/arena_locais.yaml \
  -p arquivo_missao:=<sua_missao>.yaml
```

## Pontos de extensão (para quando o robô estiver completo)

| O que falta | Onde ligar |
|---|---|
| Alinhar com a bancada | implementar `Acoes.alinhar()` em `missao.py`. Candidato: a lógica de `~/Robo-WORK/scripts/geometry_approach_align.py` (alinhamento com checagem de LiDAR, TF e costmap) |
| Achar bancadas sozinho | o `robo_station_detector` publica candidatas em `/station_detector/approach_pose`; daria para criar locais a partir delas (hoje: station_manager `auto` + `~/importar_bancadas`) |
| Pegar e soltar (braço) | implementar `Acoes.pegar()` / `Acoes.soltar()` |
| Novas ações (ex.: escanear a mesa) | `ExecutorMissao.registrar_acao("escanear_mesa", func)` |
| Outra forma de navegar | nova subclasse de `Navegador` |
| Receber tarefas da RefBox da @Work | converter a tarefa recebida em lista de `Passo` e chamar `ExecutorMissao.executar()` |
| Salvar/carregar mapa junto com os locais | usar o campo `mapa` do YAML de locais para casar com o mapa serializado do slam_toolbox |
| Usar áreas na navegação (ex.: zonas proibidas) | `Local.area` + `MapaDeLocais.locais_que_contem()` já existem; falta publicar para um *keepout filter* do Nav2 |

## Limitações conhecidas

- É um protótipo: tratamento de erro simples e sem recuperação de falhas
  na missão (`parar_na_falha` só escolhe entre parar ou seguir).
- `alinhar`, `pegar` e `soltar` só registram a ação no log.
- O clique no RViz exige o *Fixed Frame* igual ao `frame_id` (`map`).
- O `missao_node` roda a missão uma vez e termina; não há fila, pausa nem
  cancelamento pela rede.
- O `autonomo_node` pausa só pela parada de emergência; não há pausa
  "suave" pela rede. Ctrl+C cancela o objetivo do Nav2, mas confira se o
  robô parou; se não, use o botão de emergência ou o painel do Nav2 no RViz.
- Não testado no robô físico. Os nós foram testados com ROS rodando e
  entradas simuladas (cliques, costmap falso, `/emergency_stop`), sem Nav2.
- A conferência de pegada usa o footprint retangular documentado
  (+0,365 / −0,205 / ±0,315 m) e reprova célula desconhecida ou com custo
  ≥ 80, como o station_manager. Se for rígido demais para as bancadas,
  ajuste `margem_pegada_m` ou desligue com `conferir_pegada:=false`.
- Com poucas entregas o planejador acha a melhor ordem; com muitas, para
  no limite de busca (`limite_nos`) e usa a melhor encontrada até ali
  (sempre ao menos a gulosa).
- No modo `nav2`, o custo de cada par de locais é um pedido ao planner do
  Nav2: com muitos locais, o primeiro plano leva alguns segundos.
