"""Altura automatica da superficie do cubo (camera_cubo.apoio_automatico), com camera sintetica."""

import unittest

import cv2
import numpy as np

from garra_coleta.camera_cubo import CUBO, FATOR_PNP, TAG, apoio_automatico

ALTURAS = [0.0, 0.05, 0.10, 0.15]
K = np.array([[1400.0, 0, 960], [0, 1400.0, 540], [0, 0, 1]])
DIST = np.zeros(5)


def camera(inclinacao_graus=40.0, altura=0.54):
    """Camera em (0, 0, altura) olhando para +x, inclinada para baixo. R: camera -> robo_base."""
    a = np.radians(inclinacao_graus)
    frente = np.array([np.cos(a), 0, -np.sin(a)])     # eixo z optico
    direita = np.array([0.0, -1.0, 0.0])                 # eixo x optico (y do robo e esquerda)
    baixo = np.cross(frente, direita)                    # eixo y optico
    return np.column_stack([direita, baixo, frente]), np.array([0.0, 0.0, altura])


def tag_visto(R, t, x, y, h, fator):
    zt, m = h + CUBO, TAG / 2
    pts = np.array([[x - m, y + m, zt], [x - m, y - m, zt], [x + m, y - m, zt], [x + m, y + m, zt]])
    pc = (R.T @ (pts - t).T).T
    img, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, DIST)
    return img.reshape(4, 2), (R.T @ (np.array([x, y, zt]) - t)) * fator


class TestApoioAutomatico(unittest.TestCase):
    def test_acha_a_altura_e_a_posicao(self):
        R, t = camera()
        for h in ALTURAS:
            for x in (0.40, 0.45, 0.50):
                for y in (-0.08, 0.0, 0.08):
                    for fator in (FATOR_PNP - 0.03, FATOR_PNP, FATOR_PNP + 0.03):
                        cantos, tvec = tag_visto(R, t, x, y, h, fator)
                        h_est, xyz, razao = apoio_automatico(cantos, tvec, K, DIST, R, t, ALTURAS)
                        self.assertEqual(h_est, h, (x, y, fator))
                        np.testing.assert_allclose(xyz, [x, y, h + CUBO / 2], atol=1e-3)
                        self.assertAlmostEqual(razao, fator, places=3)

    def test_sem_alturas(self):
        R, t = camera()
        cantos, tvec = tag_visto(R, t, 0.45, 0.0, 0.0, FATOR_PNP)
        self.assertEqual(apoio_automatico(cantos, tvec, K, DIST, R, t, []), (None, None, None))


if __name__ == "__main__":
    unittest.main()
