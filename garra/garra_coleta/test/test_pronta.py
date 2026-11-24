"""A pose PRONTA precisa ser a mesma no cliente_garra, no initial_positions.yaml e no firmware.

O firmware vai sozinho para PRONTA no boot (PULSO_PRONTA, em pulsos) e o driver ROS assume o
initial_positions.yaml; se divergirem, o braco anda para outro lugar ao abrir a serial.
"""

import re
import unittest
from pathlib import Path

import yaml

from garra_coleta.cliente_garra import JUNTAS, PRONTA

RAIZ = Path(__file__).resolve().parents[2]
MOVEIT = RAIZ / 'garra_moveit_config' / 'config'
FIRMWARE = RAIZ / 'firmware' / 'garra_servos' / 'garra_servos.ino'


class TestPronta(unittest.TestCase):

    def test_initial_positions(self):
        ini = yaml.safe_load((MOVEIT / 'initial_positions.yaml').read_text())['initial_positions']
        for nome, q in zip(JUNTAS, PRONTA):
            self.assertAlmostEqual(ini[nome], q, places=3, msg=nome)

    def test_pulsos_do_firmware(self):
        cal = yaml.safe_load((MOVEIT / 'calibracao_servos.yaml').read_text())['juntas']
        m = re.search(r'PULSO_PRONTA\[N_SERVOS\]\s*=\s*\{([^}]*)\}', FIRMWARE.read_text())
        self.assertIsNotNone(m, 'PULSO_PRONTA nao encontrado no firmware')
        pulsos = [int(v) for v in m.group(1).split(',')[:3]]
        for nome, q, pulso in zip(JUNTAS, PRONTA, pulsos):
            esperado = round(cal[nome]['zero_us'] + q * cal[nome]['us_por_rad'])
            self.assertLessEqual(abs(pulso - esperado), 1, f'{nome}: {pulso} != {esperado}')


if __name__ == '__main__':
    unittest.main()
