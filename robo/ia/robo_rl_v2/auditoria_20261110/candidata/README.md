# Candidata de correção — não treinada e não ativada

Cópia isolada. Nenhum checkpoint foi modificado. Não continuar o treino principal com esta versão sem nova validação.

Correções restritas a problemas demonstráveis:
- `Grid.add_box` rejeita caixas cortadas pelos limites ou sobrepostas a células ocupadas/desconhecidas. O retorno booleano permite auditar colocações rejeitadas. Ainda pode gerar menos caixas que o nível pretendia: adicionar reamostragem com contagem explícita antes de novo treino.
- A inversão de velocidade freia até zero e só então acelera no sentido oposto, respeitando separadamente os limites configurados.

Não altera recompensas nem presume localização perfeita. Revisar localização, protocolo de currículo e domínio físico antes de investir em novo treinamento. Esta candidata não demonstrou ganho de sucesso; corrige contratos locais, não o aprendizado completo.
