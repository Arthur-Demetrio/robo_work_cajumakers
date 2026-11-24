# Estratégia de versões e backups

## Papéis
- `main`: documentação e código revisados; **não implica estado operacional validado**.
- `feature/<tema>`: alteração isolada; revisar antes de trazer para `main`.
- Tags `vX.Y.Z` **somente para uma versão identificada e validada**; anexar notas do teste, hardware, firmware e parâmetros.
- Para experimentos não validados, usar identificadores `exp-AAAA-MM-DD-<tema>`, explicitando que não são releases estáveis.

## Antes de cada teste
1. Registrar `git rev-parse --short HEAD` no notebook após importar a fonte.
2. Salvar cópia dos parâmetros ativos e identificação do firmware; não incluir segredos.
3. Registrar nome de mapa, cenário, resultado esperado e método de parada.
4. Evitar executar com alterações locais não registradas sem documentá-las.

## Backup inicial do workspace (somente quando estiver com o notebook)
Não execute sem conferir origem, destino e espaço disponível:

```bash
mkdir -p "$HOME/backups-robo"
tar --exclude='build' --exclude='install' --exclude='log' \
  -czf "$HOME/backups-robo/robo_jazzy_ws-$(date +%F-%H%M%S).tar.gz" \
  -C "$HOME" robo_jazzy_ws
```

Este backup é local, independente do GitHub. Validar conteúdo do arquivo com `tar -tzf <arquivo>` antes de alterar qualquer coisa. Manter ao menos uma segunda cópia em mídia distinta, sem expor credenciais, identificadores privados ou dados pessoais.

## Git básico — após auditoria do workspace
```bash
git status
git diff
git add <arquivos-especificos>
git commit -m "fix: descrever alteracao validada"
git push
```
**Não usar `git add .` nem `git push --force` sem revisar.** Nunca versionar `build/`, `install/`, `log/`, credenciais, `.env` ou backups completos.

## Recuperação
Para inspecionar versões: `git log --oneline` e `git show <commit>`. Não usar `reset --hard` nem sobrescrever parâmetros em produção sem backup, checklist e revisão.

## O que deve acompanhar uma versão funcional
Commit/tag, nome do firmware, mapa/SLAM, parâmetros Nav2/CM, TF/footprint, hardware e evidência de teste físico.
