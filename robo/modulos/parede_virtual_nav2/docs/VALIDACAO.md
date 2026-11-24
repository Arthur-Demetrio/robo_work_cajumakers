# Validação e procedência — 13/11/2026

## Funcionamento separado

Esta cópia contém apenas o fluxo usado no ensaio aprovado pelo operador:
fita vermelha/branca → projeção calibrada base_link → confirmação em map →
keepout/inflação Nav2 → retirada por ausência visível com o robô parado.
Não inclui máscaras fixa/dinâmica da garra, firmware, controladores, algoritmos
de manipulação ou outras alterações locais do projeto.

O operador confirmou a retirada da fita no ensaio real: “perfeitamente, deu certo”.
Isso é uma confirmação do cenário observado, não um teste automático de hardware
nem prova de estabilidade em todas as condições.

## Testes desta cópia

- Pacote 0.2.0 compilado isoladamente dentro do módulo, com ROS 2 Jazzy.
- **51 testes passaram**, incluindo regressões de visão/projeção, confirmação,
  memória, retirada, falta de odom, movimento, quadros repetidos e TF atrasado.
- A suíte operacional anterior tinha 61 testes; os 10 testes exclusivos das
  máscaras da garra não integram esta edição. Não confundir exclusão de uma
  funcionalidade com testes pulados: esta suíte passa sem skips.
- Nove módulos de lógica central foram copiados byte a byte da versão operacional;
  SHA-256 em [procedencia.json](procedencia.json). O nó de visão foi limpo das
  opções de máscara inativas e recebeu caminhos portáveis para HSV/capturas.
- As cinco imagens de regressão são as mesmas já versionadas no repositório.
- A publicação não inicia hardware, câmera ou navegação e não altera o programa
  que continua rodando no robô. Os testes ROS usam domínios isolados.

## Evidências anteriores da sessão

| Evidência | Resultado |
|---|---|
| [Costmaps com parede](evidencias/costmaps_com_parede.json) | 121/121 células da máscara letais no global e no local |
| [Filtro real Nav2 isolado](evidencias/filtro_real_nav2.json) | 24 células letais e 220 infladas antes; zero de ambas após máscara vazia |
| [Supervisor ativo](evidencias/supervisor_ativo.json) | principal=true, observar, serviço/heartbeat e único publisher do árbitro |

O ensaio isolado usou somente um costmap no domínio 171, sem controladores ou
hardware. O resultado não demonstra que uma imagem qualquer foi corretamente
interpretada como retirada; valida a atualização do filtro e da inflação.

## Calibração

13 marcas medidas, com referência frontal branca e x para frente, y para esquerda.
Origem frontal confirmada a +0,365 m em base_link. A posição estimada dos pixels
foi transferida para o enquadramento atual por registro das marcas do chão.
Os alvos de verificação não entraram nesse ajuste.

| Medida real frente/lateral | Projeção | Erro total |
|---|---|---|
| 90 / +20 cm | 89,46 / +18,86 cm | 1,26 cm |
| 70 / −25 cm | 69,97 / −25,55 cm | 0,55 cm |
| 50 / +30 cm | 49,60 / +30,97 cm | 1,05 cm |

Os dados estão em calibration/checks_20261113.json dentro do pacote; a calibração
validada registra método e limitações. O resíduo máximo de ajuste de 2,40 cm não
foi suprimido. Três pontos abaixo de 2 cm não certificam toda a imagem. Foco
automático foi desligado na sessão; não foi provado que ele causou a mudança de
enquadramento observada. Intrínsecos/retificação de lente não foram calibrados.

## Limites

Manter câmera fixa, raw 640×480, chão visível e garra fora da observação. Sem
segmentação de oclusores, um objeto claro sobre a fita pode simular sua ausência.
Memória é da sessão, não acompanha correções globais de loop closure. Movimentos
especiais externos ao Nav2 não passam a obedecer aos costmaps automaticamente.
Não foi demonstrado comportamento em todos os ângulos, iluminações ou ambientes.
