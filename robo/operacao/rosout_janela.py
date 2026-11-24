#!/usr/bin/env python3
import sys
from datetime import datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log

NIVEL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
h, m, s = (float(x) for x in sys.argv[2].split(':'))
alvo = h * 3600 + m * 60 + s
leitor = rosbag2_py.SequentialReader()
leitor.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id=''),
            rosbag2_py.ConverterOptions('', ''))
vistas = set()
while leitor.has_next():
    topico, dados, _ = leitor.read_next()
    if topico != '/rosout':
        continue
    msg = deserialize_message(dados, Log)
    t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
    d = datetime.fromtimestamp(t)
    tod = d.hour * 3600 + d.minute * 60 + d.second + d.microsecond / 1e6
    if -25 <= tod - alvo <= 5:
        chave = (msg.name, msg.msg[:80])
        if chave in vistas:
            continue
        vistas.add(chave)
        print(f"{d.strftime('%H:%M:%S.%f')[:-3]} {NIVEL.get(msg.level, msg.level):5} [{msg.name}] {msg.msg[:150]}")
