# arena_2016_v2 — primeira revisão visual, não aprovada

**Reconstrução aproximada da arena RoboCup@Work 2016 baseada em referência visual e restrições geométricas conhecidas.** A figura é um exemplo de 2016; não é a planta de 2026. A v2 não foi ativada no treino nem no visualizador do treino.

## Inspecionar primeiro

1. [Comparação em quatro painéis](review/comparacao_4_paineis.png): referência com cantos, retificação, vetor e raster candidato.
2. [Geometria reprojetada na imagem](review/reprojecao_sobre_original.png): laranja = paredes; verde = plataformas e círculo; magenta = superfícies elevadas; ciano = suportes, alguns inferidos.
3. [Vetor em PNG](review/vetorizada.png) e [SVG](review/vetorizada.svg).
4. [Regiões de aproximação candidatas](review/areas_servico.png).

O quarto painel é o mapa candidato exportado desta versão, **não um mapa já integrado ao simulador em execução**. O visualizador e o treinamento ativos continuam usando a v1.

## Referência e pendência obrigatória

Fonte exigida: [regulamento CBR 2026](https://cbr.robocup.org.br/wp-content/uploads/2026/07/CBR_RoboCup_Work_Rulebook_2026.pdf), Figura 1, página física 9 (índice 8). O texto do documento foi consultado. A seção 3.2 limita a área a 10–120 m² e estabelece 0,80 m para passagens/acesso; não fixa 12 × 10 m.

**A extração da imagem embutida do PDF não foi concluída.** O download no executor falhou por resolução de host/restrição de rede. As ferramentas de consulta permitiram ler o texto, mas não disponibilizaram os bytes da imagem. Por isso, todos os traçados usam `source/provided_reference.png`, a imagem de **1047 × 565 pixels fornecida pelo usuário**, preservada sem alteração. Não foi feita super-resolução ou geração de conteúdo para simular detalhe inexistente. Os arquivos de revisão têm resolução maior para facilitar a leitura dos vetores, sem aumentar a informação disponível na referência.

`extract_pdf.py --pdf /caminho/local/documento.pdf` já está preparado para extrair a imagem nativa da página 9 com `pdfimages` e renderizar separadamente a página. Essa etapa precisa do PDF local. A extração não substituirá automaticamente as anotações: será necessário registrar as coordenadas na imagem extraída e refazer a inspeção. Ainda não há um arquivo `figure_1_embedded.png` nesta entrega.

## Método e geometria

Os quatro cantos do piso no sistema da imagem são: esquerda (117,265), frente (426,443), direita (890,177), fundo (567,4), com incerteza manual inicial de ±3 pixels. A homografia é armazenada no YAML, com sua ordem de cantos. É uma retificação aproximada de um piso assumido retangular. Não fornece calibração métrica, pose de câmera nem alturas.

A retificação da imagem inteira corrige apenas o **plano do piso**. Tampos e coberturas mantêm paralaxe/altura: não devem ser traçados diretamente na imagem retificada como se estivessem no chão. As posições de contato e projeções no piso foram anotadas separadamente. As coberturas com aparência inclinada têm deslocamentos verticais estimados por vértice; pernas ocultas são marcadas como inferidas. Essas projeções ainda são hipóteses para revisão, não medidas obtidas por reconstrução 3D.

São 40 primitivas: nove conjuntos de paredes com segmentos individuais; quinze plataformas/estruturas baixas; uma mesa vermelha e quatro pés; duas coberturas e oito suportes; um objeto circular. Nenhum template simétrico foi usado. Cada primitiva mantém coordenadas de origem em pixels, geometria métrica estimada e intervalos de altura. Os IDs permitem localizar o mesmo elemento no vetor, na reprojeção e no YAML.

As pequenas peças sobre a mesa de precisão e na estrutura baixa frontal não foram reconstruídas como objetos móveis: não há informação suficiente para identificar formas/alturas. A viga frontal tem papel funcional desconhecido; não foi declarada automaticamente uma estação. A base do círculo e o círculo são partes associadas, sem duplicar a estação.

### Escala configurável

A área de referência **100 m² é uma hipótese de trabalho**, dentro da faixa do regulamento, não uma medida oficial. O aspecto foi estimado assumindo que o elemento circular tem superfície paralela ao piso. Isso resulta em aproximadamente **8.265 × 12.100 m**. A hipótese de circularidade, a leitura da elipse e o deslocamento por altura introduzem incerteza no aspecto.

`annotate.py --area 120` recalcula a geometria a partir das mesmas anotações, aplicando somente escala global e mantendo a topologia. Depois execute `build_review.py` para gerar os derivados. A escala do robô, as alturas e o requisito de 0,80 m não devem ser multiplicados junto com o piso. Não se movem objetos individualmente para eliminar uma violação. O valor de área pode variar de 10 a 120 m²; nem toda escala produzirá uma arena navegável para este robô.

## Camadas separadas

- **Visual:** todas as superfícies, estruturas, plataformas e suportes. Fonte: objetos com `color`, tipo e projeção no piso. A renderização 2D não oculta bases automaticamente.
- **Colisão física:** primitivas cujos intervalos verticais intersectam o volume do robô, exportadas para `review/colisao_fisica.pgm` e YAML de ocupação.
- **LiDAR:** somente primitivas que intersectam o plano de varredura, exportadas para `review/lidar.pgm` e YAML. O tampo elevado não vira automaticamente uma parede no scan.
- **Serviço/aproximação:** `service_regions` em `layout_review.yaml` e `review/service_regions.json`, com pose, largura de 0,80 m e região livre verificada ou motivo da pendência.

**Alturas ainda não calibradas:** LiDAR = 0,20 m; corpo do robô = intervalo 0–0,55 m; paredes = 0–0,28 m; plataformas = 0–0,10 m; tampo vermelho = 0,65–0,70 m; coberturas = 0,31–0,36 m. Esses valores são estimados e estão marcados assim no YAML. O footprint x=[−0,205;0,365], y=[−0,315;0,315] m vem do projeto; a altura do corpo e a altura do sensor não foram confirmadas por medição.

Nessa hipótese, a mesa vermelha colide por seus pés, sem bloquear todo o tampo. Plataformas baixas colidem com o corpo, mas podem ficar abaixo do scan. Coberturas podem colidir com a parte superior do robô mesmo quando o LiDAR enxerga somente os suportes e paredes. **Mudanças nas alturas mudam as duas camadas.** Isso precisa de confirmação antes de considerar a navegação fisicamente válida.

O exterior do piso é proibido para o robô; não foi inventada uma parede no perímetro. O LiDAR não deve ganhar um retorno físico artificial só porque o raio cruzou o limite do campo. O ambiente atual `RoboEnv` usa uma ocupação única: simplesmente carregar um desses mapas no visualizador antigo **não aplica as quatro camadas**. A integração de um ambiente de percepção separado fica para depois da revisão visual e da definição das alturas.

## Navegação: resultados preliminares, sem certificado de fidelidade

O verificador usa o polígono assimétrico completo, margem de 1 cm, 24 orientações (15°), espaços de configuração conservadores e conexões de rotação com limite de volume varrido. A tradução é testada em quatro vizinhos na grade. O teste adicional confere 600 poses livres contra a colisão SAT do simulador original e contra os limites do piso.

O maior componente contém **99.922% dos estados discretos livres**. Há 116 componentes no grafo conservador, muitos com pouquíssimos estados em bolsões estreitos ou orientações isoladas. Esse número não é uma contagem de salas fisicamente desconectadas. Não há comprovação de que todos os bolsões correspondam a espaços realmente úteis nem de ausência de erro de modelagem.

Foram encontradas aproximações candidatas para **10/14 estações candidatas**. As frentes são estimadas, não oficiais. Permanecem sem aproximação validada: P02_base_cobertura_esquerda, P03_fundo_esquerdo, P08_base_cobertura_direita, C01_plataforma_circular. O algoritmo não encontra uma solução para toda estação; não alterou posições para criá-la. É necessário revisar projeção no piso, classificação funcional, escala e faces reais de aproximação.

Há **23 pares** de elementos não pontuais com separação raster conservadora inferior a 0,80 m. A lista em `navigation_validation.json` inclui relações intencionais entre paredes, bases e coberturas da mesma estação e casos ainda ambíguos. Sobreposição entre volumes de alturas distintas não equivale automaticamente a colisão; interseções entre paredes e bases no mesmo plano precisam de revisão. Não se afirma que todas as passagens já atendem 0,80 m. A figura de 2016 não deve ser deformada para fazê-la satisfazer automaticamente cada hipótese de 2026.

A randomização está desligada: **zero caixas aleatórias** foram geradas. Assim não há spawn aleatório dentro de paredes; a coerência dos elementos fixos continua sendo revisada. As verificações automatizadas aprovam contratos de geometria/camadas e transparência das pendências. Não aprovam a fidelidade nem autorizam novo treino.

## YAML, testes e preservação

`layout_draft.yaml` é a anotação parametrizada; `layout_review.yaml` acrescenta os candidatos de serviço. Ambos têm estado `visual_review_candidate_not_approved`. `annotations_px.json` mantém os traçados. `build_review.py` gera as imagens e o diagnóstico. O teste em `tests/test_arena2016_v2_review.py` verifica homografia/proveniência, alturas/camadas, 600 poses com footprint completo e apresentação dos casos não resolvidos.

```bash
cd ~/robo_rl_v2
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m unittest discover -s tests -p test_arena2016_v2_review.py -v
```

Os mapas e `layout.yaml` da v1 foram preservados; o hash do layout ativo coincide com o registrado no início do treino. O modelo inicial coincide com seu SHA-256 de lançamento. Evidência em `preservation.json`. Nenhum comando de treino, parada, controle de janela ou escrita em pesos/logs preexistentes foi executado nesta revisão.

**Próxima decisão:** inspecionar primeiro as imagens e ajustar a correspondência dos elementos. Completar a extração do PDF e confirmar alturas/frentes. Só então resolver as pendências e considerar integração. Nenhum treinamento longo deve usar a v2 antes dessa aprovação.

## Visualizador v2 preparado após solicitação do usuário

O adaptador `robo_rl_v2/arena2016v2.py` carrega as camadas física e LiDAR separadamente. `robo_rl_v2/viewer2016v2.py` desenha também todos os móveis da camada visual e lê os checkpoints do treino transferido na arena anterior. Isso disponibiliza avaliação na v2; não muda o mapa do processo de treinamento existente. As limitações e alturas estimadas acima continuam válidas.

```bash
cd ~/robo_rl_v2 && ./visualizar_2016_v2.sh
```

O comando verifica cena e display, pede fechamento das janelas antigas pelo título exato e abre o visualizador v2. O executor não conseguiu conectar ao display `:0`; portanto não confirmou a abertura ou fechamento de janelas. O comando deve ser executado no terminal da sessão gráfica do usuário. Três testes do adaptador passaram, incluindo API, reprodutibilidade de 30 sementes e distinção entre colisão física e obstáculos do LiDAR.
