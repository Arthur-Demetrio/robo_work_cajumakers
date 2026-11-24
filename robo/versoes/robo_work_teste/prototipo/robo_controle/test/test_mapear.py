from robo_controle.mapear import (
    botao_apertado,
    eixo_movido,
    gerar_yaml,
    nome_de_arquivo,
)


def test_botao_apertado_pega_a_borda_de_subida():
    assert botao_apertado([0, 0, 0], [0, 1, 0]) == 1
    assert botao_apertado([0, 1, 0], [0, 1, 0]) is None  # ja estava apertado


def test_eixo_movido_indica_inversao():
    repouso = [0.0, 0.0, -0.06, 0.0]
    assert eixo_movido(repouso, [0.0, 0.95, -0.06, 0.0]) == (1, False)
    assert eixo_movido(repouso, [0.0, 0.0, -0.06, -1.0]) == (3, True)
    assert eixo_movido(repouso, [0.3, 0.0, -0.06, 0.0]) is None  # pouco


def test_yaml_gerado_sobrepoe_indices():
    texto = gerar_yaml("DragonRise", {"botao_homem_morto": 4}, {"giro": (3, True)})
    assert "controle_mapeamento:" in texto and "  ros__parameters:" in texto
    assert "    botao_homem_morto: 4" in texto
    assert "    eixo_giro: 3" in texto and "    inverter_giro: true" in texto


def test_nome_de_arquivo():
    assert nome_de_arquivo("DragonRise Inc. Generic USB Joystick") == \
        "dragonrise_inc_generic_usb_joystick"
