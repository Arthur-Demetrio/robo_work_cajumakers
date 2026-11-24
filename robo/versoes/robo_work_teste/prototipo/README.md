# Protótipos do robô @Work

> **Pasta só local.** Nada aqui faz parte de um repositório do robô e
> nenhum protótipo importa ou altera o código dele. Cada protótipo é um
> pacote ROS 2 independente, que conversa com o sistema só por interfaces
> padrão (tópicos, TF, serviços e ações). Assim ele pode ser integrado ou
> expandido no futuro sem retrabalho.

**Sistema alvo (desde 19/10/2026): Robo-WORK**, base mecanum com IMU.
Código de referência em `~/Robo-WORK` (git) e workspace operacional em
`~/robo_jazzy_ws` (mesmos pacotes, parâmetros de navegação mais novos; é o
que roda no robô). O sistema antigo em `../robo/` não é mais o alvo.

Para rodar um protótipo junto do robô, carregue os dois ambientes:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
source ~/robo_work_teste/prototipo/install/setup.bash
export ROS_DOMAIN_ID=0
```

| Protótipo | Ideia | Etapa |
|---|---|---|
| [robo_controle](robo_controle/README.md) | dirigir a base mecanum (frente, lateral, giro) com controle Bluetooth/USB ou teclado, pelo Collision Monitor, com parada de emergência | **mapping** |
| [robo_locais](robo_locais/README.md) | marcar locais, áreas e bancadas no mapa, anotar o que há em cada um e o que vai para onde, e o robô **monta e executa a missão sozinho** ao terminar o mapeamento | mapping + prova |

Os dois se completam: durante o mapping, você dirige com o `robo_controle`
e, ao parar num ponto importante, aperta **salvar local** no controle para salvar a
pose como local do `robo_locais`.

## Registros dos testes

Cada protótipo grava o seu tipo de registro (JSONL, um evento por linha):

| Protótipo | Pasta | O que registra | Ler |
|---|---|---|---|
| `robo_controle` | `~/robo_testes_prototipos/controle/` | sessão de direção: trechos, velocidades, emergências, desconexões, salvar local/mapa e finalizar | `ros2 run robo_controle ver_registro` |
| `robo_locais` | `~/robo_testes_prototipos/missao/` | missão: contexto, plano, cada passo com duração e resultado, replanejamentos, pausas, entregas | `ros2 run robo_locais ver_registro` |

Num teste completo (dirigir, marcar, finalizar e executar), os dois se
complementam: o horário no nome dos arquivos liga a sessão de controle à
missão que ela disparou.

## Compilar e testar

Compile **dentro desta pasta**, separado do workspace do robô:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/marcos/robo_work_teste/prototipo
colcon build
source install/setup.bash
```

Testes (não precisam de ROS rodando):

```bash
cd /home/marcos/robo_work_teste/prototipo/robo_controle && python3 -m pytest -q test/
cd /home/marcos/robo_work_teste/prototipo/robo_locais && python3 -m pytest -q test/
```

## Mantendo local

- Não mova esta pasta para dentro de `robo/`.
- Não rode `git init` aqui nem em `robo_work_teste/`. Se quiser histórico
  de versões, pode ter um git **sem** `remote`.
