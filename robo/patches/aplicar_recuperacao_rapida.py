#!/usr/bin/env python3
"""Recuperacao rapida do Nav2 (26/10): nova arvore + PoseProgressChecker.

O que muda:
  1) behavior_trees/navigate_to_pose_smooth.xml (a arvore que o bt_navigator usa):
     - 4 tentativas de recuperacao (eram 6);
     - ordem: escape lateral -> re -> limpar mapas -> smart_spin (12 s no maximo, eram 24 s);
     - sem o Wait de 5 s.
  2) config/nav2_params.yaml, bloco progress_checker do controller_server:
     - nav2_controller::PoseProgressChecker (conta giro como progresso, o Simple nao conta);
     - 6 s (eram 12 s) sem andar 3 cm nem girar 10 graus = "travou".
  Resultado esperado: a 1a recuperacao sai em ~12 s (era ~24 s) e o Nav2 desiste em ~1 min
  (era ~2,5 min), devolvendo a decisao para o alinhador.

Uso:
    python3 aplicar_recuperacao_rapida.py            # aplica (backup antes)
    python3 aplicar_recuperacao_rapida.py --desfazer # volta os backups mais recentes
Depois: compile o pacote e reinicie o Nav2 (o script imprime os comandos).
"""
import glob
import os
import re
import shutil
import sys
import time

WS = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation')
ARQ_BT = os.path.join(WS, 'behavior_trees', 'navigate_to_pose_smooth.xml')
ARQ_PAR = os.path.join(WS, 'config', 'nav2_params.yaml')
MARCA = 'RECUPERACAO_RAPIDA'
SUF = '.bak_recup_'

NOVOS = [  # (chave, valor) dentro de progress_checker:
    ('plugin', '"nav2_controller::PoseProgressChecker"  # ' + MARCA),
    ('required_movement_radius', '0.03'),
    ('required_movement_angle', '0.17'),
    ('movement_time_allowance', '6.0'),
]

ARVORE = """<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <!--
    RECUPERACAO_RAPIDA v1 (26/10)
    Mudancas em relacao a arvore anterior:
      - tentativas de recuperacao: 6 -> 4 (depois disso o alinhador decide);
      - ordem: escape lateral (LiDAR) -> re -> limpar mapas -> smart_spin;
      - smart_spin com no maximo 12 s (era 24 s);
      - sem o Wait de 5 s (na arena nao ha ninguem para "sair da frente").
    O tempo para perceber que travou vem do progress_checker do controller_server
    (PoseProgressChecker: 6 s sem andar 3 cm nem girar 10 graus).
  -->
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="4" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector" />
        <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector" />
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <Sequence name="ComputeAndSmoothPath">
              <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" />
              <SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}" smoother_id="simple_smoother" max_smoothing_duration="2.0" check_for_collisions="false" error_code_id="{smoothing_path_error_code}" />
            </Sequence>
            <Sequence>
              <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}" />
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap" />
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{smoothed_path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" />
          <Sequence>
            <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}" />
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap" />
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <Sequence>
        <Fallback>
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}" />
          <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}" />
        </Fallback>
        <ReactiveFallback name="RecoveryFallback">
          <GoalUpdated />
          <RoundRobin name="RecoveryActions">
            <!--
              Recovery holonomica customizada: o servidor /mecanum_escape interpreta
              dist_to_travel como deslocamento lateral e escolhe esquerda/direita pelo
              /scan_front. Saida: /cmd_vel_nav -> velocity_smoother -> collision_monitor.
              Vem primeiro porque e a mais rapida (recusa em ~0,1 s se nao ha lado livre).
            -->
            <DriveOnHeading server_name="mecanum_escape" dist_to_travel="0.03" speed="0.02" time_allowance="6.0" error_code_id="{mecanum_escape_code_id}" />
            <BackUp backup_dist="0.10" backup_speed="0.03" time_allowance="6.0" error_code_id="{backup_code_id}" />
            <Sequence name="ClearingActions">
              <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap" />
              <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap" />
            </Sequence>
            <Spin server_name="smart_spin" spin_dist="0.70" time_allowance="12.0" error_code_id="{spin_error_code}" />
          </RoundRobin>
        </ReactiveFallback>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""


def desfazer():
    ok = True
    for arq in (ARQ_BT, ARQ_PAR):
        baks = sorted(glob.glob(arq + SUF + '*'))
        if not baks:
            print('Sem backup para %s' % arq)
            ok = False
            continue
        shutil.copy2(baks[-1], arq)
        print('Restaurado %s  <-  %s' % (arq, os.path.basename(baks[-1])))
    return 0 if ok else 1


def patch_params(txt):
    """Devolve o texto novo ou levanta ValueError (nada e gravado)."""
    linhas = txt.split('\n')
    idx = [i for i, l in enumerate(linhas) if re.match(r'^\s*progress_checker:\s*(#.*)?$', l)]
    if len(idx) != 1:
        raise ValueError('esperava 1 bloco "progress_checker:", achei %d' % len(idx))
    i0 = idx[0]
    ind0 = len(linhas[i0]) - len(linhas[i0].lstrip())
    fim = i0 + 1
    while fim < len(linhas):
        l = linhas[fim]
        if l.strip() and (len(l) - len(l.lstrip())) <= ind0:
            break
        fim += 1
    bloco = list(range(i0 + 1, fim))
    filhos = [j for j in bloco if linhas[j].strip() and not linhas[j].strip().startswith('#')]
    if not filhos:
        raise ValueError('bloco progress_checker vazio')
    ind = linhas[filhos[0]][:len(linhas[filhos[0]]) - len(linhas[filhos[0]].lstrip())]
    plug = [j for j in filhos if re.match(r'^\s*plugin:', linhas[j])]
    if len(plug) != 1 or 'ProgressChecker' not in linhas[plug[0]]:
        raise ValueError('nao achei "plugin: ...ProgressChecker" dentro de progress_checker')
    ultimo = filhos[-1]
    extras = []
    for chave, valor in NOVOS:
        js = [j for j in filhos if re.match(r'^\s*%s:' % chave, linhas[j])]
        if len(js) > 1:
            raise ValueError('chave %s repetida no bloco' % chave)
        nova = '%s%s: %s' % (ind, chave, valor)
        if js:
            linhas[js[0]] = nova
        else:
            extras.append(nova)
    linhas[ultimo + 1:ultimo + 1] = extras
    return '\n'.join(linhas), '\n'.join(linhas[i0:ultimo + 1 + len(extras)])


def main():
    if '--desfazer' in sys.argv:
        return desfazer()
    for arq in (ARQ_BT, ARQ_PAR):
        if not os.path.isfile(arq):
            print('Nao achei %s. Nada alterado.' % arq)
            return 1
    bt = open(ARQ_BT).read()
    par = open(ARQ_PAR).read()
    if MARCA in bt and MARCA in par:
        print('Ja aplicado (achei a marca %s nos dois arquivos). Nada a fazer.' % MARCA)
        return 0
    if 'mecanum_escape' not in bt or 'smart_spin' not in bt:
        print('A arvore atual nao parece a nossa (sem mecanum_escape/smart_spin). Nada alterado.')
        return 1
    try:
        par_novo, trecho = patch_params(par) if MARCA not in par else (par, '(ja estava)')
    except ValueError as exc:
        print('nav2_params.yaml: %s. Nada alterado.' % exc)
        return 1
    carimbo = time.strftime('%Y%m%d_%H%M%S')
    for arq in (ARQ_BT, ARQ_PAR):
        shutil.copy2(arq, arq + SUF + carimbo)
        print('Backup: %s' % (arq + SUF + carimbo))
    if MARCA not in bt:
        open(ARQ_BT, 'w').write(ARVORE)
        print('Arvore nova gravada em %s' % ARQ_BT)
    open(ARQ_PAR, 'w').write(par_novo)
    print('progress_checker agora:\n' + trecho)
    print('\nAgora compile e reinicie o Nav2:')
    print('  cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash')
    print('Confira depois de subir:')
    print('  ros2 param get /controller_server progress_checker.plugin')
    return 0


if __name__ == '__main__':
    sys.exit(main())
