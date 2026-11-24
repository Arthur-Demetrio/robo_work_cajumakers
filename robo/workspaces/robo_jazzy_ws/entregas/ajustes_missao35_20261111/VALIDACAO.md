# Missao35 — evidencias e limites

## SLAM
- Snapshot inicio/_slam_toolbox.yaml: mapping, scan_front. Config local identica a missao33; mantida conforme resposta explicita do usuario.
- Leitura parcial limitada a chunks completos, sem reindexar/alterar gravacao ativa. Leitor conferido contra rosbag2 em 20 scans de bag34, mesmos bytes/timestamps. Amostras adicionais foram tomadas em instantes diferentes de uma gravacao que crescia; nao comparar contagens como se fossem o mesmo intervalo.
- 38.982 TF map->odom: 17 correcoes angulares >3 graus; maior 14,288 graus com variacao IMU na janela de 0,6 s de apenas 0,125 grau. Ha tambem salto isolado global IMU de 85,625 graus; incremento maximo da odom continua 2,375 graus. Nao atribuir toda deriva a uma unica causa. Dados e scripts em yaw.json/yaw.log e ferramentas/analise/yaw_missao35.py.
- Hipotese concreta para o SLAM: retornos do proprio corpo estavam no scan_front. Correcao na entrada remove-os antes do SLAM e do alinhador. Mantidos parametros originais de mapping, inclusive limites do matcher. Ainda nao foi comprovada reducao do salto angular apos a correcao; grafo ativo nao foi limpo nem recarregado.

## Filtro
- Primeira leitura: 60.342 pontos corporais no raw, 16.927 ainda no scan_front; 558 na borda ainda no scan_nav (bins -84..-74). Motores/bordas persistentes aparecem tambem em -146/-144/-142 e 138/140/162/164 graus.
- Replay maior: 4.758 scans; mascara removeria 119.460 pontos estritamente dentro do envelope real, preservando 565.147 externos; 26.053 externos estavam ocultos pelas antigas mascaras angulares. Nao se filtrou um setor inteiro ate distancia infinita. NaN nao apaga parede atras do motor.
- Envelope conforme medidas do projeto; montagem da mascara compartilha TF do launch tanto no fluxo normal como temporal. Scan bruto original preservado. Falta validar hardware com a montagem atual e repetir medida de orientacao.

## Re, transicao e lateral
- Perfil principal autoriza DWB -0,10 m/s, CM traseiro ampliado; nenhum critico, footprint ou velocidade positiva alterado. Legado sem re e recuperacao propria com re bloqueada conservados.
- Detector inclui corredor que comeca na pose atual; se mapa+duas paredes concordam e robo ja esta alinhado, dispensa retorno ao preparo. Falha ABORTED avalia a confirmacao antes de desistir; nao se inventa rota estatica se planner falhar. Cancelamento/preempcao mantem parada e resultado terminal antes de transferir.
- Suavizador novo preserva eixos explicitamente zerados e nao transfere modulo frontal acumulado para o lado. Regressao reproduzida no filtro legado e corrigida, sem remover diagonais intencionais. Ativacao vinculada ao modo principal.
- Bag: um dos 11 inicios DWB laterais analisados ja pediu diagonal (+0,08;-0,053); os demais iniciaram com vx=0. O bag nao prova pulso frontal comandado em todos os inicios. Motor/deriva fisica nao foram alterados; odometria derivada de comandos nao prova deslocamento independente.

## Testes
- Build final dos dois pacotes em install_nav2_principal aprovado; instalacao operacional nao sobrescrita deliberadamente. Dependencia robo_recovery usada da instalacao existente.
- Sonda DWB real: 440 amostras, 168 com re e 4 laterais puras para cada lado. Expectativa de re configuravel na sonda; legado continua exigindo zero amostras de re.
- Nativos: zero colisoes em open, curve, final_yaw, obstacle, dynamic, corner. Os tres primeiros concluiram; os tres ultimos nao concluiram. Nao declarar contorno de caixas resolvido.
- Suite afetada inicial: 178 aprovados/1 falha por teste que pressupunha string de setores nao vazia. Corrigido para simular retorno corporal real e exigir DESCONHECIDA apos NaN, sem permitir giro cego. Selecao final: 70 aprovados, incluindo montagem customizada da TF, corredor/T, preempcao, retentativa proibida apos cancelamento, filtros e pulso lateral. Logs preservados; nao houve repeticao redundante da suite inteira.
- DDS restrito impede certificar fluxo ROS em processos separados e replay SLAM fechado. Sem restart/Nav2, sem parametro ativo alterado, sem comando de movimento ou goal enviado pela ferramenta.

## Rollback e proximo ensaio
- Backup desta rodada: backup.txt; `python3 entregas/ajustes_missao35_20261111/rollback.py --verificar` confere hashes e `--desfazer` nao e necessario (executar o mesmo script sem --verificar restaura as fontes).
- Rollback recusa sobrescrever edicoes posteriores e preserva estado removido. Para desfazer a arquitetura anterior, primeiro desfazer esta rodada; o rollback anterior recusara corretamente arquivos alterados depois dele. Overlay compilado deve ser recompilado apos rollback, antes de qualquer uso; nao restaura processo em memoria.
- Numa parada autorizada: recarregar os componentes afetados, partir do mapa/posegraph original preservado e medir orientacao/IMU novamente. Um ensaio por mudanca, sem misturar promocao dos patches M34. O robô ja em execucao ainda nao recebeu as novas correcoes.
