# Protocolo de registro de alterações — Robo-WORK

## Compromisso de rastreabilidade
Toda mudança proposta, executada ou testada deve gerar um registro claro e compreensível. **Planejamento não é execução, execução não é validação.** A atualização do repositório será feita durante as sessões com acesso à integração GitHub; caso não haja acesso, produzir registro pronto para sincronização sem afirmar que foi publicado.

## Para cada intervenção
1. **Contexto e objetivo:** qual problema, condição e comportamento esperado.
2. **Estado anterior:** commit/tag, hardware/firmware, arquivos e parâmetros anteriores.
3. **Mudança efetuada:** arquivos, trechos afetados, parâmetros antes/depois, justificativa e explicação técnica.
4. **Teste:** data, cenário, procedimentos, medições, logs e referência de commit.
5. **Resultado:** observado, inferido, não testado; passou, falhou, inconclusivo ou parcial.
6. **Efeitos colaterais e riscos:** inclusive se houve regressões.
7. **Decisão:** manter, reverter ou iterar; próximo teste e método de recuperação.

## Nomenclatura
- Commit: `feat:`, `fix:`, `test:`, `docs:`, `chore:`, com descrição específica.
- Experimento: `exp-AAAA-MM-DD-tema-01` (não é release estável).
- Versão estável: `vX.Y.Z` somente após validação explícita e rastreável.
- Teste: `tests/AAAA-MM-DD_tema_01.md`, baseado em [modelo](modelo-teste.md).

## Regras
- Não apagar histórico de versões falhas; documentar conclusão e apontar commits ou branches.
- Preservar o código real e backups antes de intervenções.
- Toda publicação de código deve especificar o que mudou e como executar/reverter.
- Não publicar segredos, dados pessoais ou logs volumosos.
- Experimentos que não foram realizados devem permanecer como *propostos*.
- Quando a mudança depender de hardware, validar segurança antes do teste físico.

## Exemplo breve
**Hipótese:** limite de giro insuficiente causa falha. **Mudança:** parâmetro X, de A para B. **Teste:** robô parado e percurso curto, com log do CM. **Resultado:** inconclusivo, bloqueio persistiu. **Decisão:** não marcar estável; manter registro e planejar medição do fluxo de comandos.

Esse exemplo é ilustrativo, não relata um teste real.

## Critério de relevância — documentação enxuta
Registrar **somente** mudanças ou descobertas que ajudem a reproduzir, explicar, validar, diagnosticar ou recuperar o robô: código/firmware alterado, parâmetros com impacto real, resultado de teste significativo (inclusive falha útil), decisão técnica importante, configuração funcional e riscos novos.

Não abrir documento, issue ou commit apenas para registrar comandos rotineiros, tentativas idênticas sem aprendizado, conversas exploratórias sem decisão, logs redundantes ou pequenas observações sem consequência. Consolidar várias tentativas relacionadas em **um registro sintético**, com links para evidências quando necessário.

Alterações reais de código continuam sob controle de versão normalmente, mas a documentação narrativa deve ser proporcional à importância. Em caso de dúvida, priorizar um resumo curto no registro existente em vez de criar novos arquivos.
