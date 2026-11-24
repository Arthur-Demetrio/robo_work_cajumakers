# Histórico consolidado de testes — observações anteriores

> Síntese retrospectiva de conversas e diagnósticos. Datas e métricas precisam ser reconciliadas com os logs originais no notebook. Uma alteração de parâmetro **não equivale** a resultado validado.

| Data (2026) | Cenário | Observação | Estado |
|---|---|---|---|
| 18/09 | Mecanum em piso | Após correção física, deslocamento lateral funcionou; rotação de 90° permaneceu incompleta em algumas tentativas | Parcial |
| 05–07/10 | Nav2/corredores | Trajetos com progresso positivo; houve goal failed e dúvida sobre folga traseira real versus RViz | Misto; rever logs |
| 09/10 | Parâmetros de controle | Giro final aparentou mais rápido; percurso em zigue-zague no corredor | Ajustes pendentes |
| 10/10 | DDS sem Wi-Fi | Uso forçado de UDPv4 associado à perda de tópicos; configuração foi revertida | Reverificar offline |
| 13–14/10 | Recuperação em quinas | Relatados 6 goals concluídos, 2 escapes laterais, 47 'No valid trajectories', 18 'Failed to make progress' e 103 STOP do Collision Monitor | Resultados relatados, não reproduzidos |
| 13–14/10 | Spin progressivo | Comando angular registrado; em testes o robô não girou, com bloqueio VelocityPolygonStop | Falha em investigação |
| 15–16/10 | Bancada e exploração | Discussão de estratégia sem mapa prévio e identificação de bancada por múltiplas observações | Planejamento, **não teste físico validado** |
| 21/10 | Travessia LiDAR com alinhador v3.4 | Ida: 3 medições e alinhamento ~5 s; volta: 1 medição e alinhamento ~1 s; ambas as travessias ~7 s; sem bloqueios observados | Melhor resultado relatado até a data; conferir bags para validação |
| 22/10 | Identificação de bancadas e encosto em WS_1 | Testes de candidatos via `service_areas`; `encostar2` (alinhador v3.6): navegação 16,9 s, encosto 18,0 s com 8 medições, folga LiDAR 6 cm para alvo de 8 cm, erro lateral recalculado ~9,5 cm e travessia 8,8 s | Evolução funcional com imprecisões; correções de validação solicitadas, não consideradas testadas |

## Referência do teste de travessia de 21/10/2026

- Código: commit [`5576380`](https://github.com/OttavioSilva/Robo-WORK/commit/5576380), integrado pelo [PR #7](https://github.com/OttavioSilva/Robo-WORK/pull/7).
- Arquivos alterados: alinhador v3.4, parâmetros Nav2/Collision Monitor, árvore de comportamento, geometria e log do smart_spin, medição de escape por `/odom`.
- Medições e tempos informados pelo responsável pelo teste; bags e logs não foram reanalisados neste registro.
- Próxima validação: repetir ida e volta de diferentes posições e orientações e verificar consistência do alinhamento, folgas e bloqueios.

## Desenvolvimento e testes de 22/10/2026

**Identificação e representação das áreas de serviço**

- O nó `service_areas` foi executado, publicando `/service_areas/marcadores` para visualização no RViz. Em uma inspeção, a mensagem continha apenas `Marker.DELETEALL`, sem candidatos publicados naquele instante.
- Houve falsos positivos em corredores/parede. Em identificações manuais, C6 correspondia à mesa e C7 ao corredor; em outra rodada automática, C3 era a mesa e C1/C4/C5 eram corredores. Esses identificadores são locais às rodadas, não IDs permanentes.
- Foi experimentado um filtro de continuidade de pelo menos 1 m para reduzir falsos positivos; após o ajuste, o corredor deixou de aparecer naquela observação.
- A área de trabalho recebeu o nome **WS_1**. O alinhador era acionado por `/ir_para_area` e usava o arquivo `~/mapas/service_areas.yaml`.

**Encosto e travessia**

- No bag identificado como `encostar2`, relatado para o **alinhador v3.6**: navegação de **16,9 s**, encosto de **18,0 s com 8 medições** e travessia de **8,8 s**.
- A folga LiDAR medida foi **6,0 cm**, diante de uma meta de **8 cm**; o erro lateral recalculado foi de cerca de **9,5 cm**. Portanto, não classificar o encosto como precisão satisfatória apenas porque o movimento prosseguiu.
- Foram solicitadas correções para **parar quando a face da bancada não for encontrada**, **exigir centro válido** e **validar distância, ângulo e centro antes de declarar sucesso**. O escopo foi o **encosto da bancada**, não o centralizador de corredor/travessia nem o detector `service_areas.py`.
- Houve evolução do script do alinhador durante o dia; versões e resultados não devem ser confundidos. **Não há neste registro comprovação de que as correções finais tenham sido testadas fisicamente**.

**Limites deste registro:** observações e métricas provenientes dos relatos e registros da sessão; sem reprocessamento independente dos bags. O código de 22/10 não foi incluído nesta atualização documental e precisa ser comparado com os arquivos locais antes de substituir a versão publicada no GitHub.

## Registros de 26 a 28/10/2026 — pacote recebido

As observações abaixo foram extraídas dos relatórios `RESULTADOS_26-10.md`, `RESULTADOS_27-10.md` e `RESULTADOS_28-10.md` do ZIP `robo_2026-10-28.zip`. Os logs e bags não foram reprocessados aqui; ver [resumo da atualização](../docs/atualizacao-2026-10-28.md).

| Data | Teste e desenvolvimento | Resultado e ressalva |
|---|---|---|
| 26/10 | Executor BTT1 com tarefas simuladas; ajustes Nav2/recuperação | 11/11 etapas e 4m25s da tarefa, mas travamento posterior ao tentar FINISH |
| 27/10 | `filtro_corpo_scan.py`, tópico `/scan_nav` e missão WS_1–WS_2–WS_1 | Replay mostrou eliminação do STOP falso nos scans gravados; missão concluída em ~2m50s, com falha no FINISH |
| 28/10 | Curvas 4–6, alinhador v3.14–v3.16, avaliação de deriva e patch DWB | Saídas laterais de curva5 concluídas; v3.16 obteve 112/120 em simulação de deriva e 19/36 no teste do nó em curva; patch `dwb_sem_lateral` piorou e foi desfeito |

O ZIP descreve a **v3.16 como candidata para prova, não como versão estável**. Os códigos do ZIP precisam ser importados e revisados nesta branch para manter o Git correspondente ao relatório.

## Interpretação
Os números acima foram relatados em sessões passadas e não representam comparativo controlado. Conferir período amostral, parâmetros e condições físicas antes de traçar indicadores. Evitar extrapolar sucesso de um cenário para competição.

## Registro de novos testes
Usar o [modelo](../docs/modelo-teste.md), anexando links para evidências e commit testado. Nome recomendado: `AAAA-MM-DD_cenario_numero.md`.
