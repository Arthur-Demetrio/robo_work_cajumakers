#!/usr/bin/env python3
"""Roda o alinhador_passagem.py REAL (sem copiar nem editar o arquivo) e acrescenta um gatilho:
   /sim/encostar (std_msgs/String "WS_1") -> chama no._encostar("WS_1") do proprio no.
O codigo testado e exatamente o do src (le o arquivo na hora); o resultado sai no topico normal
/ir_para_area/resultado. Uso: python3 alinhador_bancada.py --ros-args -p arquivo_areas:=<yaml>
"""
import os
import sys
import threading

SCRIPTS = os.environ.get('BANCADA_SCRIPTS') or os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts')
sys.path.insert(0, SCRIPTS)

import rclpy.executors as executors  # noqa: E402
from std_msgs.msg import String  # noqa: E402

_add_node = executors.MultiThreadedExecutor.add_node


def _gatilho(self, node):
    ok = _add_node(self, node)
    if getattr(node, 'get_name', lambda: '')() != 'alinhador_passagem' or hasattr(node, '_bancada'):
        return ok
    node._bancada = True
    ocupado = threading.Event()

    def rodar(area):
        try:
            node.cancelamento_t.clear()
            node.res_area = None
            node.encostado = False
            node._encostar(area)
        except Exception as exc:   # Cancelado / SensorParado / erro: publica como o trabalhador faz
            node.get_logger().error('BANCADA: _encostar levantou %s: %s' % (type(exc).__name__, exc))
            if node.res_area is None:
                node._publicar_resultado(area, False, 'erro: %s: %s' % (type(exc).__name__, exc))
        finally:
            ocupado.clear()

    def ao_gatilho(msg):
        if ocupado.is_set():
            node.get_logger().warn('BANCADA: encosto ja em andamento, ignorei %s' % msg.data)
            return
        ocupado.set()
        node.get_logger().info('BANCADA: encostar %s' % msg.data)
        threading.Thread(target=rodar, args=(msg.data.strip(),), daemon=True).start()

    node.create_subscription(String, '/sim/encostar', ao_gatilho, 10)
    node.get_logger().info('BANCADA: gatilho /sim/encostar pronto (codigo de %s)' % SCRIPTS)
    return ok


executors.MultiThreadedExecutor.add_node = _gatilho

import alinhador_passagem  # noqa: E402

alinhador_passagem.main()
