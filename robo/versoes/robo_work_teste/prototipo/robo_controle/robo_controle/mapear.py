"""PROTOTIPO: descobre quais botoes/eixos do controle fazem o que.

Controles genericos nao sao reconhecidos pelo SDL como gamepad, entao o
joy_node publica a numeracao BRUTA de cada controle. O controle.yaml vem
com os indices do DragonRise "PC TWIN SHOCK" (0079:0006) do notebook;
para outro controle, gere os indices com esta ferramenta. Esta
ferramenta pede para voce apertar cada botao/empurrar cada analogico,
le o /joy e grava um YAML so com os indices certos:

    ros2 run joy joy_node                      # terminal 1
    ros2 run robo_controle mapear_controle     # terminal 2

    ros2 launch robo_controle controle_mapeamento.launch.py \\
        botoes:=~/robo_testes_prototipos/controles/<controle>.yaml

Seguranca: so le o /joy, nao publica nada. Recusa rodar se o
controle_node estiver aberto (apertar botoes poderia finalizar o
mapeamento ou acionar a emergencia).
"""

from __future__ import annotations

import array
import fcntl
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

PASTA_SAIDA = "~/robo_testes_prototipos/controles"

# (parametro, o que pedir). Ordem = ordem das perguntas.
BOTOES = [
    ("botao_homem_morto", "o botao de HOMEM-MORTO (segurar para andar; sugestao: L1)"),
    ("botao_lento", "o botao de MODO LENTO (sugestao: R1)"),
    ("botao_emergencia", "a PARADA DE EMERGENCIA (sugestao: Circulo / B)"),
    ("botao_liberar", "LIBERAR a emergencia, sera segurado 2 s (sugestao: X / Cruz)"),
    ("botao_salvar_local", "SALVAR LOCAL (sugestao: Triangulo / Y)"),
    ("botao_salvar_mapa", "SALVAR MAPA (sugestao: Start)"),
    ("botao_finalizar", "FINALIZAR MAPEAMENTO, sera segurado 2 s (sugestao: Select)"),
]
# (eixo, o que pedir, convencao: este movimento deve dar valor POSITIVO)
EIXOS = [
    ("frente", "o analogico de ANDAR para FRENTE (sugestao: esquerdo, para cima)"),
    ("lateral", "o analogico de ANDAR DE LADO, para a ESQUERDA (sugestao: esquerdo)"),
    ("giro", "o analogico de GIRAR, para a ESQUERDA (sugestao: direito, horizontal)"),
]

LIMIAR_EIXO = 0.6


def botao_apertado(antes: Sequence[int], agora: Sequence[int]) -> Optional[int]:
    """Indice do botao que acabou de ser apertado (0 -> 1), ou None."""
    for i, (a, b) in enumerate(zip(antes, agora)):
        if a == 0 and b == 1:
            return i
    return None


def eixo_movido(
    repouso: Sequence[float], agora: Sequence[float], limiar: float = LIMIAR_EIXO
) -> Optional[Tuple[int, bool]]:
    """(indice, inverter) do eixo que mais saiu do repouso, se passou do limiar.

    inverter=True quando o movimento pedido deu valor NEGATIVO (o
    controle_node espera positivo para frente/esquerda).
    """
    melhor, maior = None, 0.0
    for i, (r, v) in enumerate(zip(repouso, agora)):
        if abs(v - r) > maior:
            melhor, maior = i, abs(v - r)
    if melhor is None or maior < limiar:
        return None
    return melhor, agora[melhor] - repouso[melhor] < 0


def nome_do_controle(device_id: int = 0) -> str:
    """Nome que o kernel da ao /dev/input/jsN (melhor esforco)."""
    try:
        fd = os.open(f"/dev/input/js{device_id}", os.O_RDONLY | os.O_NONBLOCK)
        try:
            buf = array.array("B", [0] * 128)
            fcntl.ioctl(fd, 0x80006A13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(128)
            return " ".join(buf.tobytes().split(b"\0")[0].decode(errors="replace").split())
        finally:
            os.close(fd)
    except OSError:
        return "controle"


def nome_de_arquivo(nome: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", nome.lower()).strip("_") or "controle"


def gerar_yaml(nome: str, botoes: dict, eixos: dict) -> str:
    linhas = [
        f"# Gerado por mapear_controle em {datetime.now():%Y-%m-%d %H:%M}",
        f"# Controle: {nome}",
        "# Use com: ros2 launch robo_controle controle_mapeamento.launch.py botoes:=<este arquivo>",
        "controle_mapeamento:",
        "  ros__parameters:",
    ]
    for nome_eixo, (indice, inverter) in eixos.items():
        linhas.append(f"    eixo_{nome_eixo}: {indice}")
        linhas.append(f"    inverter_{nome_eixo}: {'true' if inverter else 'false'}")
    for parametro, indice in botoes.items():
        linhas.append(f"    {parametro}: {indice}")
    return "\n".join(linhas) + "\n"


# ---------- Parte ROS (interativa) ----------


def main(args=None) -> None:
    import rclpy
    from sensor_msgs.msg import Joy

    rclpy.init(args=args)
    node = rclpy.create_node("mapear_controle")
    ultimo: List[Optional[Joy]] = [None]
    node.create_subscription(Joy, "/joy", lambda m: ultimo.__setitem__(0, m), 10)

    def esperar(condicao, timeout_s: float = 60.0):
        fim = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < fim:
            rclpy.spin_once(node, timeout_sec=0.05)
            if ultimo[0] is not None:
                resultado = condicao(ultimo[0])
                if resultado is not None:
                    return resultado
        return None

    try:
        time.sleep(1.0)
        rclpy.spin_once(node, timeout_sec=0.5)
        if any(n == "controle_mapeamento" for n in node.get_node_names()):
            print("O controle_node esta rodando: feche-o antes (apertar botoes aqui poderia "
                  "finalizar o mapeamento ou acionar a emergencia).")
            return
        print("Esperando /joy (rode 'ros2 run joy joy_node' em outro terminal)...")
        if esperar(lambda m: m, 15.0) is None:
            print("Nenhuma mensagem em /joy. O joy_node esta rodando e o controle conectado?")
            return
        nome = nome_do_controle()
        print(f"\nControle: {nome}  ({len(ultimo[0].axes)} eixos, {len(ultimo[0].buttons)} botoes)")
        print("Solte todos os botoes e analogicos. Se o controle tiver botao ANALOG, deixe o LED aceso.\n")
        time.sleep(1.5)

        botoes, usados = {}, set()
        for parametro, pedido in BOTOES:
            print(f"Aperte e solte {pedido}...")
            while True:
                esperar(lambda m: True if not any(m.buttons) else None)  # tudo solto
                antes = list(ultimo[0].buttons)
                indice = esperar(lambda m: botao_apertado(antes, m.buttons))
                if indice is None:
                    print("  tempo esgotado; mapeamento cancelado.")
                    return
                if indice in usados:
                    print(f"  botao {indice} ja foi usado para outra funcao; escolha outro.")
                    continue
                break
            botoes[parametro], usados = indice, usados | {indice}
            print(f"  -> botao {indice}")

        eixos = {}
        for nome_eixo, pedido in EIXOS:
            esperar(lambda m: True if not any(m.buttons) else None)
            time.sleep(0.5)
            rclpy.spin_once(node, timeout_sec=0.1)
            repouso = list(ultimo[0].axes)
            print(f"Empurre ate o fim e segure {pedido}...")
            achado = esperar(lambda m: eixo_movido(repouso, m.axes))
            if achado is None:
                print("  tempo esgotado; mapeamento cancelado.")
                return
            eixos[nome_eixo] = achado
            print(f"  -> eixo {achado[0]}{' (invertido)' if achado[1] else ''}. Pode soltar.")
            esperar(lambda m: True if eixo_movido(repouso, m.axes, 0.2) is None else None, 10.0)

        indices_eixos = [i for i, _ in eixos.values()]
        if len(set(indices_eixos)) < len(indices_eixos):
            print(f"\nATENCAO: dois movimentos cairam no mesmo eixo {indices_eixos}. Confira e "
                  "rode de novo (o controle esta no modo ANALOG?).")

        destino = Path(PASTA_SAIDA).expanduser() / f"{nome_de_arquivo(nome)}.yaml"
        destino.parent.mkdir(parents=True, exist_ok=True)
        destino.write_text(gerar_yaml(nome, botoes, eixos), encoding="utf-8")
        print(f"\nSalvo em {destino}\nUse com:\n  ros2 launch robo_controle "
              f"controle_mapeamento.launch.py botoes:={destino}")
    except KeyboardInterrupt:
        print("\ncancelado.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
