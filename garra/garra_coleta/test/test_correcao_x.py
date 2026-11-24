"""Correcao de x do pegar_cubo: tabela do chao e por altura da superficie."""

import unittest

from garra_coleta.pegar_cubo import x_corrigido


class TestCorrecaoX(unittest.TestCase):

    def test_chao_sem_z_e_com_z_do_chao(self):
        self.assertAlmostEqual(x_corrigido(0.405), 0.390)
        self.assertAlmostEqual(x_corrigido(0.405, 0.021), 0.390)

    def test_mesa_15cm(self):
        self.assertAlmostEqual(x_corrigido(0.497, 0.171), 0.457)
        self.assertAlmostEqual(x_corrigido(0.40, 0.171), 0.36)    # um ponto so: vale para toda a mesa

    def test_altura_sem_tabela_usa_o_chao(self):
        self.assertAlmostEqual(x_corrigido(0.485, 0.121), 0.485)  # mesa de 10 cm: ainda sem medida


if __name__ == '__main__':
    unittest.main()
