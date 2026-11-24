"""Conversao de entrada do controle em comando de velocidade, sem ROS.

Alvo: base mecanum do sistema Robo-WORK. O codigo de referencia esta em
~/Robo-WORK (git), mas os parametros EM USO no robo sao os do workspace
operacional ~/robo_jazzy_ws (mais novo). Diferente da base antiga
(diferencial, sem curva fechada), aqui frente (vx), lateral (vy) e giro (w)
sao independentes e podem ser combinados (ex.: diagonal).

O comando vai para /cmd_vel_nav e passa pelo pipeline do Nav2:

    /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
                 -> collision_monitor -> /cmd_vel -> robo_hardware

Por isso os limites padrao sao os do velocity_smoother em
~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml (conferido em
19/10/2026; a copia do ~/Robo-WORK tinha 0.05/0.035/0.08). Acima deles o
smoother reduz o vetor INTEIRO na mesma proporcao (scale_velocities: true)
e a direcao comandada mudaria sem aviso. Abaixo da deadband do smoother (0,015) o
comando vira zero; por isso cada eixo comeca um pouco acima dela.

Quem lida com os limites fisicos (PWM minimo, manter o rumo com a IMU) e o
proprio robo_hardware, nao este modulo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Limites:
    # Maximos = velocity_smoother.max_velocity [vx, vy, w].
    vx_max: float = 0.08
    vy_max: float = 0.08
    w_max: float = 0.14
    # Minimo de cada eixo: logo acima da deadband_velocity do smoother.
    v_min: float = 0.016
    w_min: float = 0.016
    # Modo lento (botao lento segurado): fracao dos maximos, para encostar em bancadas.
    fator_lento: float = 0.5
    zona_morta: float = 0.15


def _fora_da_zona_morta(valor: float, zona: float) -> float:
    """Reescala |valor| de [zona, 1] para [0, 1]; dentro da zona vira 0."""
    magnitude = abs(valor)
    if magnitude <= zona:
        return 0.0
    return math.copysign(min((magnitude - zona) / (1.0 - zona), 1.0), valor)


def _eixo(entrada: float, minimo: float, maximo: float) -> float:
    """[0, 1] do analogico -> [minimo, maximo], mantendo o sinal; 0 continua 0."""
    if entrada == 0.0:
        return 0.0
    maximo = max(maximo, minimo)
    return math.copysign(minimo + abs(entrada) * (maximo - minimo), entrada)


def joystick_para_velocidade(
    frente: float,
    lateral: float,
    giro: float,
    lento: bool = False,
    limites: Limites = Limites(),
) -> tuple[float, float, float]:
    """Converte os analogicos em (vx [m/s], vy [m/s], w [rad/s]).

    frente: +1 = frente. lateral: +1 = esquerda (+y no ROS).
    giro: +1 = anti-horario (esquerda).
    """
    zona = limites.zona_morta
    f = _fora_da_zona_morta(frente, zona)
    l = _fora_da_zona_morta(lateral, zona)
    g = _fora_da_zona_morta(giro, zona)
    escala = limites.fator_lento if lento else 1.0
    return (
        _eixo(f, limites.v_min, limites.vx_max * escala),
        _eixo(l, limites.v_min, limites.vy_max * escala),
        _eixo(g, limites.w_min, limites.w_max * escala),
    )

class BotaoSegurado:
    """Dispara uma unica vez quando o botao fica pressionado por `segundos`.

    Para acoes que nao podem acontecer por esbarrao (ex.: finalizar o
    mapeamento, que libera o robo para andar sozinho). Para disparar de
    novo, e preciso soltar e segurar outra vez.
    """

    def __init__(self, segundos: float) -> None:
        self.segundos = segundos
        self.desde: float | None = None
        self.disparou = False

    def atualizar(self, pressionado: bool, agora: float) -> bool:
        if not pressionado:
            self.desde, self.disparou = None, False
            return False
        if self.desde is None:
            self.desde = agora
        if not self.disparou and agora - self.desde >= self.segundos:
            self.disparou = True
            return True
        return False

    @property
    def segurando(self) -> bool:
        return self.desde is not None and not self.disparou
