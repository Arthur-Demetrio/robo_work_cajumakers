import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Log

rclpy.init()
node = Node('missao_ws')
pub = node.create_publisher(String, '/ir_para_area', 10)

estado = {'sucesso': False, 'falha': False, 'alvo': ''}

def log_cb(msg):
    if 'alinhador_passagem' not in msg.name:
        return
    texto = msg.msg
    alvo = estado['alvo']
    if f'ENCOSTAR {alvo}: SUCESSO' in texto:
        estado['sucesso'] = True
    if f'ENCOSTAR {alvo}:' in texto and (
        'FALH' in texto.upper() or 'ERRO' in texto.upper()
    ):
        estado['falha'] = True

node.create_subscription(Log, '/rosout', log_cb, 10)

def visitar(alvo):
    estado.update(alvo=alvo, sucesso=False, falha=False)
    print(f'\n>>> INDO PARA {alvo}', flush=True)


    msg = String()
    msg.data = alvo
    pub.publish(msg)

    inicio = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if estado['sucesso']:
            print(f'>>> {alvo}: SUCESSO', flush=True)
            return True
        if estado['falha']:
            print(f'>>> {alvo}: FALHA — missão interrompida')
            return False
        if time.monotonic() - inicio > 240:
            print('>>> Tempo limite — missão interrompida')
            return False
    return False

try:
    while node.count_subscribers('/ir_para_area') == 0:
        print('Aguardando alinhador...', flush=True)
        rclpy.spin_once(node, timeout_sec=1.0)

    if visitar('WS_1'):
        print('>>> AGUARDANDO 10 SEGUNDOS', flush=True)
        time.sleep(10)
        if visitar('WS_2'):
            if visitar('WS_1'):
                print('\n>>> MISSÃO CONCLUÍDA!')
finally:
    node.destroy_node()
    rclpy.try_shutdown()
