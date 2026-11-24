# Atualização de 26 a 28/10/2026 — pacote de navegação

**Origem:** arquivo enviado `robo_2026-10-28.zip`. Este documento registra o conteúdo e o estado informado pelo pacote; não substitui os arquivos-fonte do robô nem demonstra que todos os testes foram executados novamente.

## Estado informado no pacote

- Alinhador de passagem **v3.16**, candidato para prova; não presumir estabilidade em todos os cenários.
- `filtro_corpo_scan.py` remove retornos do LiDAR dentro do corpo do robô para produzir `/scan_nav`.
- `nav2_params.yaml` contém configuração de referência após patches; não aplicar os patches sobre ele novamente sem comparar o resultado.
- Árvore `navigate_to_pose_smooth.xml` com recuperação rápida.
- Executor de prova, ferramentas de velocidade e missão, testes e análises de bags.

## Evidências e limitações relatadas

| Data | Resultado | Limitação |
| --- | --- | --- |
| 26/10 | Prova BTT1 com **11/11 etapas** e tempo de **4 min 25 s** para a tarefa; 2/2 passagens LiDAR | Travamento posterior no FINISH, com orientação incorreta e bloqueio de segurança; auxílio manual |
| 26/10 | Medidas mostraram diferença entre velocidade comandada e real; ajustes de recuperação e giro rápido | O novo giro produziu oscilação próxima ao goal; correção do shim foi proposta/testada separadamente |
| 27/10 | Filtro de retornos internos reduziu STOP falso de 98% dos scans gravados para 0% nesse replay | O dado deriva dos scans gravados, não comprova ausência de falsos STOP em toda condição |
| 27/10 | Prova WS_1 → WS_2 → WS_1 concluída em aproximadamente 2 min 50 s | FINISH falhou por estar próximo da parede; executor passou a verificar folga mínima de 0,50 m para START |
| 28/10 | `curva5` (v3.15): duas saídas laterais concluídas | Ida 24 s e volta 33 s; zigue-zague do DWB e giro final bloqueado |
| 28/10 | Simulação v3.16: 112/120 com modelo de deriva; teste do nó em curva: 19/36 | Ainda existem falhas de travessia; `curva4` apresentou deriva e risco de colisão |
| 28/10 | Patch `dwb_sem_lateral.py` testado em `curva6` | **Piorou**, foi desfeito; manter somente em `patches/descartados/` |

## Integração correta no repositório

A importação do ZIP deve ser feita em uma branch separada, comparando os arquivos de destino com o `src/` atual:

- `robo_navigation/scripts/alinhador_passagem.py` → `src/robo_navigation/scripts/alinhador_passagem.py`
- `robo_navigation/scripts/filtro_corpo_scan.py` → `src/robo_navigation/scripts/filtro_corpo_scan.py`
- `robo_navigation/config/nav2_params.yaml` → `src/robo_navigation/config/nav2_params.yaml`
- `robo_navigation/behavior_trees/navigate_to_pose_smooth.xml` → `src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml`
- `patches/` → `patches/` (manter `descartados/` isolado)
- `ferramentas/` → `ferramentas/`
- `testes/` → `tests/navegacao_20261028/`
- `analise/` → `analise/navegacao_20261028/`
- `resultados/RESULTADOS_*.md` e artefatos seletivos → `tests/resultados_20261028/`

**Não** tratar `ferramentas/zonas_velocidade.py` como ativado: o próprio relatório diz que **nunca foi aplicado**.

**Não** executar instaladores ou patches apenas porque fazem parte do pacote. Os scripts operam sobre o workspace real e podem alterar parâmetros de segurança.

**Fora deste ZIP:** arquivos operacionais de outros pacotes (`robo_hardware` etc.), sketch completo e mapas; permanecer com os existentes no Git e não inventar versões novas.

## Próximas validações

1. Repetir casos de corredor com deriva lateral e verificar folgas reais de frente e traseira.
2. Testar estabilidade de parada e orientação, considerando PWM mínimo da ponte e odometria.
3. Confirmar correspondência entre parâmetros efetivamente em execução e arquivos versionados.
4. Avaliar START/FINISH e segurança de giro antes de classificar uma versão estável.

**Atenção:** até a cópia dos arquivos do ZIP ser feita para esta branch, este commit é somente documental.
