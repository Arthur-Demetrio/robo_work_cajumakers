# Filtro do LiDAR

Topicos:

- entrada bruta: /scan_raw;
- saida completa para diagnostico: /scan;
- saida frontal de 180 graus para marcar obstaculos e alimentar o SLAM:
  /scan_front;
- saida frontal densa, usada somente para clearing do Nav2:
  /scan_front_clear.

O launch tambem e o unico publicador do TF estatico
base_link -> laser_raw, com x=0,2755 m, y=0,005 m e z=0,075 m.

O setor traseiro ignorado e publicado como NaN; portanto ele nao marca nem
limpa celulas. Um raio frontal sem retorno permanece como +inf. Como o LiDAR
tem poucos raios para uma grade de 5 cm, /scan_front_clear divide cada setor
angular em quatro sub-raios. Um sub-raio sintetico somente recebe +inf quando
os dois raios originais vizinhos tambem sao +inf. Se houver retorno finito ou
NaN em qualquer lado, o sub-raio vira NaN e nao pode apagar a borda de um
obstaculo verdadeiro.

A configuracao fisicamente validada, considerando que o firmware ja aplica
o alinhamento frontal de 270 graus, e:

    front_offset_deg: 0.0
    invert_angles: true
    front_half_angle_deg: 90.0

Executar:

    ros2 launch robo_lidar lidar_filter.launch.py

As mascaras angulares ficam vazias porque a metade traseira inteira ja e
descartada. Nao use /scan no SLAM ou nos costmaps. /scan_front_clear nunca
deve marcar obstaculos; ele e uma fonte exclusiva de clearing no Nav2.
