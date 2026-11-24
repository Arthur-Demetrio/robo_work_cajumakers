# Visualizador 2D do aprendizado por checkpoints

No terminal da sessão gráfica Linux:

```bash
cd ~/robo_rl_v2
./visualizar_treino.sh
```

Abre uma janela Tk separada. O treino que já está rodando continua como está. O visualizador não precisa de Codex, OpenAI, internet ou TensorBoard. Fechar sua janela encerra somente a visualização. Não é necessário reiniciar o treinamento.

## Controles

| Controle | Efeito apenas na avaliação |
|---|---|
| Espaço / Pausar | Pausa ou continua a simulação exibida |
| R / Reiniciar | Recomeça a mesma semente e o mesmo modelo |
| N / Próximo cenário | Descarta um episódio parcial e passa à próxima semente |
| Velocidade | Escolhe de 0,25× a 16×; o limite de CPU prevalece |
| C / Novo checkpoint | Aplica o checkpoint completo já carregado e inicia nova avaliação |
| Fechar janela | Encerra o visualizador; o treino permanece independente |

As informações à direita têm barra de rolagem para telas menores.

## O que aparece

- Raster verdadeiro importado de 04/11 às 09:17, com origem, orientação e escala física preservadas. A extensão desconhecida é cinza e permanece bloqueada.
- Caixas sintéticas adicionais em marrom, separadas da ocupação do mapa em preto.
- Footprint completo do robô, orientação, origem do LiDAR e raios válidos. Raios ocluídos não são desenhados como espaço livre.
- Objetivo e trajetória física da avaliação.
- Seta ciano para translação solicitada; arco magenta para o sentido da rotação. A seta mostra uma decisão, não uma previsão da trajetória. Há atraso, aceleração e escorregamento na dinâmica.
- Ação normalizada e velocidades solicitadas em `base_link`, em m/s e rad/s.
- Passos do checkpoint, passos e tempo do episódio, recompensa instantânea e acumulada, colisões e taxa de sucesso.

A faixa superior identifica explicitamente **AVALIAÇÃO DO CHECKPOINT — NÃO É UM EPISÓDIO DO TREINAMENTO**. Os passos globais exibidos pertencem ao último modelo carregado, não ao contador ao vivo do processo de treino. A recompensa, o trajeto e as decisões pertencem exclusivamente à avaliação visualizada.

## Atualizações e comparação

A cada 5 segundos, o visualizador lê `runs/principal/latest.json`. Não varre ZIPs em gravação: segue o ponteiro que o treinador publica atomicamente. Confere o ZIP completo, CRC, dimensão dos espaços e número de passos antes de carregar uma cópia na CPU. Um arquivo incompleto ou inválido mantém o modelo anterior e produz uma mensagem na janela.

O carregamento ocorre em uma thread para preservar os controles da janela. A política é colocada em modo de avaliação, com gradientes desativados, e usa somente `predict()` sob `inference_mode()`. Não chama `learn()`, `save()` ou o controle de processos do treinamento. Não escreve em `runs/principal`.

Cada episódio usa um modelo congelado. Um novo checkpoint fica pendente até o fim do episódio, incluindo uma breve exibição do resultado; então é aplicado automaticamente. Para vê-lo imediatamente, pressione **C**: o episódio parcial é descartado, sem contabilizá-lo como falha. Enquanto pausado no meio de um episódio, você pode aplicar o novo checkpoint por C sem retomar a reprodução. Se o treinamento produzir várias versões nesse intervalo, o visualizador acompanha a mais recente, sem obrigação de reproduzir todas as intermediárias.

São 8 sementes fixas de avaliação no nível 4, reutilizadas entre versões para facilitar a comparação. A taxa exibida considera **cenários únicos concluídos desta versão**; repetir a mesma semente substitui o resultado e não aumenta artificialmente a amostra. Antes do primeiro cenário concluído, a taxa aparece como `—`. O histórico das últimas cinco versões mostra a taxa e o tamanho de amostra; contagens pequenas não comprovam qualidade. As métricas são mantidas na memória da janela e reiniciam quando ela é fechada.

O visualizador sempre testa a arena real, mesmo quando o currículo do treino ainda está no nível 1. Falhas nessa etapa não significam que o treinamento parou. O mapa mais recente em posegraph continua sem raster correspondente; essa limitação do simulador não mudou.

## Consumo e opções

```bash
# Reprodução mais rápida, mantendo o orçamento de CPU
./visualizar_treino.sh --speed 4

# Menor consumo: um CPU permitido, uma thread, menor frequência de desenho
./visualizar_treino.sh --cpu-budget 0.10 --fps 8

# Outro diretório de treino
./visualizar_treino.sh --run runs/economico

# Limitar a duração dos episódios visuais, sem tocar na configuração do treino
./visualizar_treino.sh --max-steps 750 --scenarios 8
```

Padrões: afinidade a **um CPU**, preferindo o último permitido (o launcher de treino escolhe os primeiros); **uma thread** de PyTorch/BLAS, CPU apenas, prioridade **nice 15**, até 15 desenhos por segundo e orçamento aproximado de **20% de um CPU** para os ticks de simulação/desenho. Pausas são introduzidas entre ticks; a velocidade efetiva pode ficar abaixo da solicitada. Importação inicial, carregamento de checkpoints e resets podem causar picos curtos no único CPU permitido; não é uma quota rígida de cgroup. Mais aceleração não remove esse controle de ritmo.

O processo da janela é separado do treino e do Codex. Inicie em um terminal gráfico normal. Para também poder fechar esse terminal:

```bash
nohup ./visualizar_treino.sh > /tmp/robo_rl_visualizador.log 2>&1 &
```

Isso não inicia nem retoma treinamento.

## Validação realizada

- **17 testes automatizados passaram**, incluindo os 13 anteriores. Resultado em `docs/viewer/tests.log`.
- Modelos congelados mantiveram tensores idênticos após inferência; hash do ZIP permaneceu igual.
- Testes cobrem ponteiro completo, rejeição de ZIP inválido, confinamento ao diretório de checkpoints, troca na fronteira do episódio, reset de métricas por versão e contagem por sementes únicas.
- A cena do canvas foi exercitada sem display: raster, polígono, trajetória, LiDAR, decisões e transformação com origem rotacionada. O raster gerado é `docs/viewer/map_preview.png`.
- Avaliação de leitura do checkpoint principal de **60.000 passos**, por 30 passos: pesos e ZIP inalterados. Tempo de CPU 0,145 s em 0,712 s de relógio, aproximadamente 20% de um CPU, sem incluir carregamento. Registro em `docs/viewer/live_checkpoint_validation.json`.
- Fontes de `train.py`, `env.py`, `maps.py` e `control.py` permaneceram idênticas. Não houve sinais enviados ao treino nem alterações em seus scripts ou checkpoints.
- A abertura gráfica não pôde ser verificada pelo executor: a conexão ao display `:0` foi negada. Tk 8.6 está instalado. O script informa esse erro e orienta executar no terminal gráfico normal. Não foi possível validar visualmente os controles em uma janela real nesta sessão.
