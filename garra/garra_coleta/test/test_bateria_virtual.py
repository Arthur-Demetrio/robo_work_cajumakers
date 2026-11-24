"""Partes da bateria virtual que nao precisam do MoveIt: estruturas do YAML e resumo."""

import os
import unittest

import yaml

from garra_coleta.bateria_virtual import PREFIXO, caixas_da_estrutura, faixas

YAML = os.path.join(os.path.dirname(__file__), '..', 'config', 'bateria_virtual.yaml')


class TestBateria(unittest.TestCase):

    def test_yaml_valido(self):
        cfg = yaml.safe_load(open(YAML))
        for e in cfg['estruturas']:
            objetos, altura, corpo = caixas_da_estrutura(e, -0.029, cfg['largura_y'])
            self.assertGreaterEqual(altura, 0.0, e['nome'])
            self.assertTrue(corpo == 'robo_base' or corpo.startswith(PREFIXO), e['nome'])

    def test_estante_tem_teto_acima_do_vao(self):
        e = {'nome': 'e', 'tipo': 'estante', 'altura': 0.1, 'vao': 0.25, 'espessura': 0.02,
             'borda_x': 0.3, 'profundidade': 0.3}
        objetos, altura, corpo = caixas_da_estrutura(e, 0.0, 1.0)
        teto = next(o for o in objetos if o.id == PREFIXO + 'teto')
        p, d = teto.primitive_poses[0].position, teto.primitives[0].dimensions
        self.assertAlmostEqual(p.z - d[2] / 2, 0.35)
        self.assertAlmostEqual(p.x - d[0] / 2, 0.3)
        self.assertEqual(corpo, PREFIXO + 'prateleira')

    def test_chao_nao_cria_caixa(self):
        self.assertEqual(caixas_da_estrutura({'tipo': 'chao', 'borda_x': 0.275}, 0.0, 1.0)[0], [])

    def test_faixas(self):
        self.assertEqual(faixas([0.13, 0.14, 0.15, 0.18], 0.01), '13-15, 18 cm')
        self.assertEqual(faixas([], 0.01), '-')


if __name__ == '__main__':
    unittest.main()
