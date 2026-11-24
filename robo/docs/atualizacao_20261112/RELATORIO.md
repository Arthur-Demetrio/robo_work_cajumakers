# Publicação consolidada — 12/11/2026

Destino: **OttavioSilva/Robo-WORK**. Fontes originais preservados; consolidação realizada em checkout independente.

Captura iniciada: 2026-11-12T11:37:46.044965+00:00. Concluída: 2026-11-12T11:43:06.670483+00:00.

Total capturado: **7,743 arquivos**, **2,865,475,339 bytes**. Caminhos, tamanhos, SHA-256 e exclusões estão em `manifesto.json`. Arquivos grandes e dados de execução ficam na release; os fontes e relatórios menores ficam navegáveis no Git.

A captura é consistente por arquivo, com conferência de tamanho e mtime antes/depois da cópia. Ela não é uma transação global: o treinamento continuou avançando, de modo que logs e checkpoints podem refletir instantes diferentes dentro da janela documentada. Os resultados do piloto são parciais.

## Validação

- 63 testes offline de navegação/supervisão: aprovados (`ros.txt`, `ros.xml`).
- 33 testes de visão/configuração/parede virtual: aprovados (`visao.txt`, `visao.xml`).
- 230 fontes Python atuais: sintaxe válida (`validacao_publicacao.json`).
- Registros históricos de validação do piloto: 39 testes de regressão e 7 testes próprios, aprovados antes do treinamento.
- Scanner de padrões de credenciais nos arquivos menores que 20 MiB: nenhum alerta. Isso é uma verificação por padrões, não uma auditoria completa de segredos em todo histórico binário.
- Nenhuma compilação nova completa de ROS/firmware nem teste de hardware foi executado nesta publicação.

## Histórico e autoria

Os commits e autores anteriores são preservados. Os novos commits de consolidação usam a identidade GitHub de OttavioSilva. Os históricos locais adicionais estão em bundles dentro do arquivo de referência da release; o `robo_jazzy_ws` não tinha remoto configurado.

## Limitações e retomada

A IA não concluiu as nove réplicas no momento desta captura. O relatório publicado não deve ser tratado como recomendação final. A execução interrompida do primeiro lançamento foi preservada no diretório `interrupcoes`, e a bateria relançada de forma independente manteve as condições pareadas de partida.

Para conferir integridade, baixar todos os assets da release e executar `sha256sum -c SHA256SUMS`. O manifesto permite conferir individualmente os arquivos capturados. A atualização não modifica processos ou worktrees operacionais.

Saídas de compilação residuais encontradas entre evidências históricas foram mantidas somente nos arquivos da release, sem objetos C/C++ no Git navegável.

## Integridade dos arquivos completos

Os quatro arquivos da release foram lidos integralmente: todos os **7.743 arquivos** corresponderam ao tamanho e SHA-256 do manifesto, sem ausências ou divergências. Detalhes em `integridade_arquivos.json`.
