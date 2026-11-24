"""PARTIDA_LATERAL (12/11): k do feedforward por lado e partida lateral com o vx desde o inicio. Sem ROS.

Confere: (1) desligado = identico a baseline (missao24/34) comando a comando; (2) k por lado so muda o lado
configurado; (3) partida: rampa, roda nenhuma abaixo do PWM minimo, parada inalterada; (4) replay offline
pelos modelos do smoother e da ponte validados no bag da missao34; (5) patch: aplicar/recusar/desfazer."""
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / 'src/robo_navigation/scripts'
PATCH = ROOT / 'ferramentas/patches/aplicar_partida_lateral.py'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(ROOT / 'ferramentas/analise/partida_lateral'))
from alinhador_passagem import TravessiaLateral  # noqa: E402
import modelo  # noqa: E402

DT = 0.05


def _carregar(caminho, nome):
    spec = importlib.util.spec_from_file_location(nome, caminho)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


PATCH_MOD = _carregar(PATCH, 'aplicar_partida_lateral')
ATUAL = (SCRIPTS / 'alinhador_passagem.py').read_text()


def _texto_original():
    txt = ATUAL
    for velho, novo in reversed(PATCH_MOD.TROCAS):
        assert txt.count(novo) == 1
        txt = txt.replace(novo, velho)
    return txt


_TMP = tempfile.mkdtemp(prefix='partida_lateral_')
_ORIG_ARQ = os.path.join(_TMP, 'alinhador_original.py')
with open(_ORIG_ARQ, 'w') as _f:
    _f.write(_texto_original())
TravessiaOriginal = _carregar(_ORIG_ARQ, 'alinhador_original').TravessiaLateral


def deslize(cls, lado, folgas=(0.12, 0.12), dur=1.5, **kw):
    """Comandos do _deslizar_fundido desde a entrada no 'deslizar' (robo parado), a 20 Hz."""
    x = cls(lado, fundido=True, log=lambda _: None, **kw)
    x.estado, x.t_estado, x.ref, x.ref_ini = 'deslizar', 0.0, 0.0, 0.0
    x.paredes = (True, True)
    out = []
    for i in range(int(round(dur / DT))):
        t = i * DT
        x.folga = (t, folgas[0], folgas[1])
        out.append(x._deslizar_fundido(t, (0.0, 0.0, 0.0), 1.0))
    return out


FOLGAS = [(0.12, 0.12), (0.08, 0.16), (0.16, 0.08), (0.05, 0.20), (0.20, 0.05), (0.30, 0.30)]


class TestDesligadoIgualBaseline(unittest.TestCase):
    def test_marca_e_padroes_desligados(self):
        self.assertIn('PARTIDA_LATERAL (12/11)', ATUAL)
        for chave, padrao in (('fundido_k_ff_esq', '-1.0'), ('fundido_k_ff_dir', '-1.0'),
                              ('partida_lateral_s', '0.0')):
            self.assertIn("self.declare_parameter('%s', %s)" % (chave, padrao), ATUAL)
        x = TravessiaLateral(1, fundido=True)
        self.assertEqual((x.k_ff_lado, x.partida_s), (0.2, 0.0))

    def test_comandos_identicos_sem_parametros(self):
        for lado in (1, -1):
            for f in FOLGAS:
                self.assertEqual(deslize(TravessiaLateral, lado, f), deslize(TravessiaOriginal, lado, f), (lado, f))

    def test_comandos_identicos_com_parametros_desligados(self):
        kw = dict(k_ff_esq=-1.0, k_ff_dir=-1.0, partida_s=0.0)
        for lado in (1, -1):
            for f in FOLGAS:
                self.assertEqual(deslize(TravessiaLateral, lado, f, **kw), deslize(TravessiaOriginal, lado, f))

    def test_k_ff_global_continua_valendo(self):
        for lado in (1, -1):
            self.assertEqual(deslize(TravessiaLateral, lado, k_ff=0.5), deslize(TravessiaOriginal, lado, k_ff=0.5))


class TestKPorLado(unittest.TestCase):
    def test_so_o_lado_configurado_muda(self):
        esq = deslize(TravessiaLateral, 1, k_ff_esq=0.5)
        self.assertAlmostEqual(esq[-1][0], -0.04)   # -0,5 * 0,08
        self.assertEqual(deslize(TravessiaLateral, -1, k_ff_esq=0.5), deslize(TravessiaOriginal, -1))
        dir_ = deslize(TravessiaLateral, -1, k_ff_dir=0.6)
        self.assertAlmostEqual(dir_[-1][0], 0.048)
        self.assertEqual(deslize(TravessiaLateral, 1, k_ff_dir=0.6), deslize(TravessiaOriginal, 1))

    def test_k_zero_e_valido_e_mantem_piso(self):
        # k 0 por lado: so a correcao de centro (piso de 0,016 continua)
        self.assertEqual(deslize(TravessiaLateral, 1, k_ff_esq=0.0)[-1][0], 0.0)
        self.assertAlmostEqual(deslize(TravessiaLateral, 1, (0.16, 0.08), k_ff_esq=0.0)[-1][0], 0.016)


class TestPartida(unittest.TestCase):
    KW = dict(partida_s=0.5)

    def test_rampa_e_regime(self):
        for lado in (1, -1):
            c = deslize(TravessiaLateral, lado, **self.KW)
            o = deslize(TravessiaOriginal, lado)
            self.assertAlmostEqual(c[0][1], lado * 0.03)
            vys = [abs(v[1]) for v in c]
            self.assertEqual(vys, sorted(vys))
            self.assertAlmostEqual(max(vys), 0.08)
            n = int(0.5 / DT)
            self.assertEqual(c[n:], o[n:])   # depois da partida: identico

    def test_nenhuma_roda_abaixo_do_minimo_na_partida(self):
        for lado in (1, -1):
            for f in FOLGAS:
                for k in (0.2, 0.5, 1.0):
                    for vx, vy, _ in deslize(TravessiaLateral, lado, f, k_ff=k, **self.KW)[:10]:
                        self.assertTrue(vx == 0.0 or abs(vy) - abs(vx) >= 0.012 - 1e-9, (lado, f, k, vx, vy))

    def test_parada_de_seguranca_igual(self):
        for cls, kw in ((TravessiaLateral, self.KW), (TravessiaOriginal, {})):
            x = cls(1, fundido=True, log=lambda _: None, **kw)
            x.t0 = 0.0
            x.estado, x.t_estado, x.ref, x.ref_ini, x.yaw_ref = 'deslizar', 0.0, 0.0, 0.0, 0.0
            x.inicio_estado = x.p_inicio = (0.0, 0.0, 0.0)
            x.folga, x.paredes = (0.4, 0.03, 0.20), (True, True)
            self.assertEqual(x.passo(0.4, (0.0, 0.0, 0.0), 1.0), (0.0, 0.0, 0.0))
            self.assertEqual(x.estado, 'assentar')


def _1o_passo_ff(res):
    """Indice do 1o passo em que o vx (feedforward) chega as rodas."""
    return next(i for i, (o, _) in enumerate(res) if abs(o[0]) > 0)


class TestReplayOffline(unittest.TestCase):
    def _cmds(self, cls, lado, k, **kw):
        c = deslize(cls, lado, k_ff=k, dur=1.0, **kw)
        return c + [(0.0, 0.0, 0.0)] * 8   # parada: alinhador manda zero

    def test_feedforward_chega_antes(self):
        for lado in (1, -1):
            for k in (0.2, 0.44):
                base = modelo.replay(self._cmds(TravessiaOriginal, lado, k))
                novo = modelo.replay(self._cmds(TravessiaLateral, lado, k, partida_s=0.5))
                self.assertLessEqual(_1o_passo_ff(novo), 3, (lado, k))
                self.assertLess(_1o_passo_ff(novo), _1o_passo_ff(base), (lado, k))

    def test_baseline_reproduz_atraso_medido(self):
        # missao34: vx -0,019 so chegou ao /cmd_vel_smoothed com vy 0,07 (passo 7)
        res = modelo.replay([(-0.019, 0.08, 0.0)] * 12)
        self.assertEqual(_1o_passo_ff(res), 6)
        self.assertAlmostEqual(res[6][0][1], 0.07)

    def test_rodas_nunca_param_em_movimento_e_parada_igual(self):
        for lado in (1, -1):
            base = modelo.replay(self._cmds(TravessiaOriginal, lado, 0.2))
            novo = modelo.replay(self._cmds(TravessiaLateral, lado, 0.2, partida_s=0.5))
            for res in (base, novo):
                mov = [p for o, p in res if any(o)]
                self.assertTrue(all(all(abs(r) >= 70 for r in p) for p in mov))
                # 1o quadro: as 4 rodas partem juntas no minimo (como o bag: -79, 79, 79, -79)
                self.assertEqual(sorted(abs(r) for r in mov[0]), [79] * 4)
            n = 20   # comando zero a partir daqui
            pb = [p for _, p in base[n:]]
            pn = [p for _, p in novo[n:]]
            self.assertEqual(pb, pn)   # mesma desaceleracao/parada
            self.assertEqual(pn[-1], [0, 0, 0, 0])


class TestPatch(unittest.TestCase):
    def _rodar(self, arq, *args):
        return subprocess.run([sys.executable, str(PATCH), '--arquivo', arq, *args],
                              capture_output=True, text=True, check=True).stdout

    def test_aplicar_recusar_desfazer_em_copia(self):
        with tempfile.TemporaryDirectory() as d:
            arq = os.path.join(d, 'alinhador_passagem.py')
            orig = _texto_original()
            Path(arq).write_text(orig)
            self._rodar(arq)
            self.assertEqual(Path(arq).read_text(), ATUAL)
            self.assertIn('ja aplicado', self._rodar(arq))
            self.assertEqual(Path(arq).read_text(), ATUAL)
            self._rodar(arq, '--desfazer')
            self.assertEqual(Path(arq).read_text(), orig)
            self.assertTrue([n for n in os.listdir(d) if '.bak_partida_lateral_' in n])


if __name__ == '__main__':
    unittest.main()
