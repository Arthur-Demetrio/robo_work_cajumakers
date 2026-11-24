# Manual de operação — Robo-WORK

> **Rascunho operacional para validação no notebook e no robô físico.** Não substitui procedimento de segurança testado. Não movimentar o robô apenas por seguir este documento.

## 1. Preparação
1. Colocar o robô em área controlada, com rodas livres de pessoas e cabos; confirmar meio de cortar energia dos motores.
2. Inspecionar fixação de rodas mecanum, LiDAR, cabos, alimentação e Arduino.
3. Ligar computador e periféricos conforme procedimento elétrico validado pela equipe.
4. Confirmar ROS 2 Jazzy, workspace correto e `ROS_DOMAIN_ID=0`. Não introduzir variável `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` sem revisão: houve falhas de comunicação ao desligar Wi-Fi.
5. Conferir que o firmware ativo e os parâmetros de segurança foram verificados.

## 2. Inicialização
Os exemplos ficam em [terminais/](../terminais/README.md). A ordem conceitual é: hardware/IMU → LiDAR → odometria/localização → escolher mapeamento **ou** navegação conforme cenário → RViz. **Os nomes dos launch files estão pendentes de confirmação com o notebook**; não assumir que todos podem rodar juntos.

Em novos terminais, preparar o ambiente:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

Depois, verificar:
```bash
ros2 node list
ros2 topic list
ros2 topic hz /odom
ros2 topic hz /scan_front
ros2 lifecycle get /collision_monitor
```

Conferir o nome real do nó `/collision_monitor` e do tópico filtrado no sistema. Diagnóstico: `ros2 topic echo /diagnostics --once`.

## 3. Referência de odometria
Somente com o robô parado, após conferir o serviço:
```bash
ros2 service list -t | grep reset
```
O script [07-reset-odom.sh](../terminais/07-reset-odom.sh) chama `/reset_odometry` como hipótese operacional a **validar**. Não afirmar que houve reset sem confirmação da resposta e da leitura de `/odom`. Reset de odometria pode invalidar referência de localização, trajetórias e goals existentes.

## 4. Teste físico seguro
- Registrar commit/versão de software, mapa, parâmetros, piso, ambiente e objetivo.
- Confirmar Collision Monitor em estado ativo e sistema de parada independente disponível.
- Iniciar com movimento curto e baixa velocidade em ambiente livre.
- Observar comportamento real **e** RViz, verificando divergências.
- Parar imediatamente diante de atuação imprevisível, patinagem excessiva ou risco de colisão.
- Registrar evidências antes de mudar parâmetros.

## 5. Encerramento
1. Cancelar goals e parar comandos de velocidade; aguardar imobilização efetiva.
2. Desabilitar a alimentação dos motores pelo procedimento físico validado.
3. Encerrar os processos com Ctrl+C nos respectivos terminais.
4. Guardar logs, nomear teste e documentar resultados.
5. Nunca considerar Ctrl+C sozinho como parada de emergência.

## 6. Problemas já observados
- **Spin sem giro:** verificar velocidades antes/depois do Collision Monitor, limiares de PWM e `/odom`.
- **Quinas:** verificar footprint, costmaps, percurso, proximidade física e recuperação.
- **Wi-Fi desligado:** registrar DDS/configuração efetiva; não presumir que internet é requisito do ROS 2.
- **Folga discrepante:** medir dimensões físicas e comparar com footprint e TF do LiDAR.

## 7. Pendências de validação
- [ ] Confirmar launch files e ordem operacional exata.
- [ ] Confirmar serviço de reset.
- [ ] Confirmar portas seriais, dispositivos, calibração BNO055, tópicos e frames.
- [ ] Documentar sequência de alimentação e parada de emergência real.
- [ ] Executar checklist completo acompanhado de teste de bancada.
