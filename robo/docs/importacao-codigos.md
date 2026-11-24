# Plano de importação dos códigos reais

**Status: pendente de acesso ao notebook do projeto.** O repositório atual contém documentação e scripts de referência, não o código operacional auditado.

## Pré-requisitos
1. Fazer backup fora do repositório seguindo [versionamento e backup](./versionamento-backup.md).
2. Verificar `ls -la ~/robo_jazzy_ws`, `find ~/robo_jazzy_ws/src -maxdepth 3 -type f | head -80` e `git status` se já existir histórico Git.
3. Identificar pacote(s) próprios e código de terceiros. Respeitar licenças, preferindo declarar dependências de ROS em vez de copiar pacotes externos.
4. Conferir sketch efetivamente gravado no Arduino, especialmente pinos, PWM e comandos.
5. Revisar chaves, tokens, URLs privadas, dados pessoais, .env e logs.
6. Comparar os launch files reais com os nomes dos scripts em [terminais/](../terminais/README.md); **não assumir que os exemplos estão corretos**.
7. Importar em branch separada e revisar o diff antes de consolidar.

## Áreas sugeridas
- `src/`: pacotes ROS 2 próprios, preservando a estrutura de cada pacote.
- `firmware/`: sketches e dependências de firmware próprios.
- `config/`: arquivos de parâmetros relevantes sem credenciais.
- `tests/`: relatórios e evidências selecionadas.

## Critérios para declarar importação concluída
- [ ] Backup validado
- [ ] Estrutura de código revisada
- [ ] Arquivos sensíveis excluídos
- [ ] Compilação do workspace testada
- [ ] Comandos operacionais conferidos
- [ ] Versão marcada como histórica ou validada de acordo com testes reais

Não é possível recuperar automaticamente arquivos que estão apenas no notebook desligado ou inacessível. Arquivos antigos de conversas podem servir de referência, mas não devem substituir silenciosamente a versão ativa.
