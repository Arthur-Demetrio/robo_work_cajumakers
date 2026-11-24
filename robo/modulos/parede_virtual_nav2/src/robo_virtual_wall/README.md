# robo_virtual_wall 0.2.0

Pacote do módulo independente [parede_virtual_nav2](../../README.md).

Detecta fita vermelha/branca, projeta segmentos no chão em base_link e publica
uma máscara persistente para os filtros keepout do Nav2. Fita ausente pode ser
retirada após dois segundos, apenas parado e com o trecho visível. Não controla
motores nem publica goals. Esta edição não contém máscaras da garra.

HSV padrão em config/hsv_fita.json; capturas em ~/.ros/virtual_wall/captures,
alteráveis pelos parâmetros hsv_file e capture_directory do nó de visão.
Os exemplos de calibração são específicos da câmera do Robo-WORK em 13/11/2026.

Consulte [NAV2.md](NAV2.md) para tópicos, parâmetros e integração.
