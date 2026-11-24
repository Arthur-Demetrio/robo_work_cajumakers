# Integração experimental do alinhador: resultados e limites

## O que foi conectado

- Um cenário isolado de encosto foi criado com uma parede virtual de 0,92 m na arena v2; a geometria oficial não foi alterada.
- Cada ação do professor veio do código congelado e verificado por SHA-256: `Encosto`, `RastreioFaceWS`, medição de face e guarda de obstáculo. O simulador fornece os mesmos 72 raios/máscara, observação de 152 valores e footprint usado pela política.
- Um mapeamento local de atuadores usa os comentários/testes já existentes do robô para reproduzir a grande resposta a comandos pequenos. É aproximado e não foi recalibrado nesta execução; o restante do PPO conserva sua física original.
- Foram coletados 120 episódios independentes (seeds 71000–71119): 48 concluídos pelo estado `fim`, 72 abortados e zero colisões. O JSONL completo mantém todos os exemplos; a primeira clonagem usou apenas os 48 episódios concluídos.
- A validação supervisionada separou episódios: 9.575 ações de treino e 2.627 de validação, RMSE 0,027 para a primeira MLP sem estado.

## Comparação independente

Nas mesmas 100 seeds para professor e candidato:

| Controlador | Sucessos | Colisões | Observação |
|---|---:|---:|---|
| `Encosto` original + rastreador de face | 36/100 | 0 | 64 abortos; distância média final ao alvo 5,4 cm |
| MLP BC sem estado, sozinha | 0/100 | 100 | Instável em malha fechada; não aprovada |
| MLP BC condicionada ao estado, com FSM/guardas do `Encosto` ativas | 0/100 | 0 | 73 abortos por tempo; restante majoritariamente perdeu face/limite de correções |

Os dois modelos foram preservados para auditoria, mas **não estão aprovados para uso**. A baixa RMSE por amostra não previu desempenho em malha fechada. O segundo modelo não melhorou o professor e parou com frequência sem avançar até a folga.

## Limite de escopo

Isto clonou somente a habilidade de `dock/Encosto` na tarefa de parede virtual. O Behavior Tree do Nav2 não foi executado dentro do Gym: seus nós chamam servidores de planner, controller, recovery e Collision Monitor que não existem nesta simulação. Travessia de corredores e recuperação da árvore ainda não têm um professor ligado ao ambiente nem dados; não foram rotuladas como treinadas. Nenhum robô/ROS foi iniciado, nenhum treino PPO foi alterado, nenhum checkpoint antigo foi sobrescrito e nenhum ator BC foi ativado.

## Interpretação

Há evidência de como a informação pode entrar na política (observação, fase e estado), mas não de que a clonagem atual melhore o sistema. Antes de novas épocas, é necessário obter maior sucesso/reprodutibilidade do professor no simulador, representar o temporizador e a meta interna do estado para o ator ou manter esses controles explicitamente na FSM, e validar a cadência/atuação usada. A discrepância entre 6,25 Hz do Gym e o loop de 20 Hz do alinhador real também pode alterar pulsos de 0,15 s.

## Reavaliação corrigida a 20 Hz

A primeira avaliação longa ainda tinha um teto de 450 passos (22,5 s), curto para o timeout de 60 s da FSM. Essa rodada foi descartada. A avaliação válida usa 1.300 passos, executa o controle a 20 Hz, atualiza o LiDAR simulado a cada 0,20 s e compara 100 seeds (103000–103099):

| Sistema | Sucesso | Colisões | Falhas predominantes |
|---|---:|---:|---|
| Professor original `Encosto` + rastreador | 61/100 | 0 | 24 perda da face, 9 face ausente/desatualizada |
| Ator BC por estado + FSM/guardas reais | 0/100 | 0 | 66 timeouts, 8 perda de face, 7 face ausente |

O desempenho do professor melhorou ao executar seus pulsos curtos na cadência correta. O ator ainda não reproduz o comportamento. Em um replay, no estado `girar`, o professor pedia rotação negativa de magnitude 0,16 (normalizada), mas a rede sugeria rotação positiva de 0,046. A MLP recebe os raios e o rótulo do estado, mas não o alvo angular/lateral interno nem o tempo no estado. O mapeamento observação+estado não é Markov para esta FSM; erro de validação baixo não corrige a falta dessas variáveis.

Conclusão operacional: manter o alinhador atual como controlador. Não usar nenhuma das políticas BC. Para outra tentativa de clonagem, primeiro incluir erro ao alvo angular/lateral e temporizadores da FSM nas entradas (ou manter essas correções no controlador determinístico) e coletar novamente. O resultado não valida a árvore Nav2 completa.
