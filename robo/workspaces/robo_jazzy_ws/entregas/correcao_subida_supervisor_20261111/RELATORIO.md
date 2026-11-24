# Correcao da subida do supervisor

Causa: ~/subir_robo.sh carregava a instalacao operacional e chamava navigation.launch.py sem nav2_principal. As subidas 14:59 e 15:08 usaram o alinhador legado, mesmo com scripts novos no src. No segundo caso, localization estava ativo, mas supervisor/arbitro nao.

Correcoes:
- Subida padrao chama iniciar_supervisor_validado.sh observar, que carrega install_nav2_principal e passa nav2_principal=true, supervisor_modo e caminhos absolutos do YAML/BT.
- Preflight do wrapper (--validar) verifica prefixo resolvido, DWB e sintaxe da BT, sem ROS nodes nem hardware. Aplicar exige regioes cadastradas antes de iniciar hardware.
- Apos lifecycle ativo, subida exige parametros reais do alinhador e arbitro no grafo antes de declarar sucesso. Ausencia/mismatch aborta a subida.
- EXIT preserva codigo da falha, em vez de mascarar todo erro como sucesso.
- Rollback operacional de arquitetura: --legado. SLAM/matcher, DWB, CM e geometrias nao alterados nesta correcao.

Comando futuro, depois de encerrar a instancia anterior:

```bash
~/subir_robo.sh --localization arena_04-11_1235
```

Esse comando ativa a arquitetura nova em OBSERVAR. Nao habilita transferencias especializadas: cadastro de regioes continua vazio, e pose inicial/estabilidade ainda precisam ser confirmadas. Para aplicar, apos validacao/cadastro: acrescentar --supervisor-aplicar. O proprio comando recusa cadastro vazio. --legado permite restaurar a arquitetura anterior explicitamente.

Nenhum processo vivo foi reiniciado, nenhum parametro ativo foi alterado e nenhum comando de movimento foi enviado pela ferramenta. Codigo corrigido nao se aplica retroativamente aos processos abertos. DDS/GUI bloqueados neste ambiente impedem substituir/validar a instancia ativa. Nao zerado yaw nem gravado START: screenshot e callbacks nao fornecem pose final confiavel.

Testes: preflight observar executado com overlay real e aprovado, sem iniciar nodes. Teste shell de integracao usa executaveis ROS simulados, HOME temporario e os scripts reais, cobrindo selecao principal, rollback legado, parametro incorreto e arbitro ausente. Nao e teste de ROS/hardware. Resultados finais em testes_corrigidos.log. Bash -n e diff --check tambem executados.

Backup inclui scripts tracked/untracked e AGENTS/CLAUDE anteriores a esta rodada. Desfazer com `python3 entregas/correcao_subida_supervisor_20261111/patch.py --desfazer`; recusa edicoes posteriores e nao inicia processos. Desfazer esta correcao antes do rollback supervisor_localization_20261111. Patch permite --arquivo DIR para testar aplicar/idempotencia/desfazer em copia.

Resultado final: 4 testes de integracao shell passaram em 45,66 s. Preflight real em observar aprovado; preflight aplicar recusou cadastro vazio conforme esperado. Bash -n e git diff --check aprovados. Aplicar/reaplicar/desfazer/repetir rollback em copia conferidos byte a byte. Primeiras execucoes do teste falharam por mocks incompletos (servico SLAM nunca ficava disponivel e assercao confundia nome da BT com envio de action); mocks corrigidos sem relaxar verificacoes de selecao de perfil ou de ausencia de comandos/goals.
