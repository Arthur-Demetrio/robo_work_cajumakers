import sys, math, types, importlib.util
import os
D = os.path.dirname(os.path.abspath(__file__))
sys.argv=['x']
spec=importlib.util.spec_from_file_location('al', os.path.join(D, '..', 'robo_navigation', 'scripts', 'alinhador_passagem.py')); al=importlib.util.module_from_spec(spec); spec.loader.exec_module(al)
RES=0.05; W=H=140  # 7 x 7 m
L=0.82  # largura dos corredores
def faz_mapa(braços):
    g=[0]*(W*H)
    c=3.5; m=L/2
    for j in range(H):
        for i in range(W):
            x=(i+0.5)*RES; y=(j+0.5)*RES
            dentro_h = abs(y-c)<m and ('O' in braços and x<=c or 'L' in braços and x>=c)
            dentro_v = abs(x-c)<m and ('S' in braços and y<=c or 'N' in braços and y>=c)
            dentro_cruz = abs(x-c)<m and abs(y-c)<m
            # corredores com 1.6 m de comprimento a partir do cruzamento, depois area aberta
            longe = max(abs(x-c),abs(y-c))>2.0
            livre = dentro_h or dentro_v or dentro_cruz or longe
            g[j*W+i]=0 if livre else 100
    return al.Mapa(RES,W,H,0,0,0,g)
def linha(pts, passo=0.025):
    out=[]
    for (x0,y0),(x1,y1) in zip(pts,pts[1:]):
        n=max(1,int(math.hypot(x1-x0,y1-y0)/passo))
        out+=[(x0+(x1-x0)*k/n, y0+(y1-y0)*k/n) for k in range(n)]
    return out+[pts[-1]]
casos={
 'T: sobe a haste e vira p/ leste (S->L)': ('OLS', [(3.5,0.8),(3.5,3.5),(6.2,3.5)]),
 'T: barra reto (O->L)':          ('OLS', [(0.8,3.5),(6.2,3.5)]),
 'T: barra e desce a haste (O->S)': ('OLS', [(0.8,3.5),(3.5,3.5),(3.5,0.8)]),
 'L para a direita (O->S)':       ('OS', [(0.8,3.5),(3.5,3.5),(3.5,0.8)]),
 'reto pelo cruzamento (O->L)': ('OLNS', [(0.8,3.5),(6.2,3.5)]),
 'virar no cruzamento (O->N)':  ('OLNS', [(0.8,3.5),(3.5,3.5),(3.5,6.2)]),
 'reto num corredor sem cruz':   ('OL',   [(0.8,3.5),(6.2,3.5)]),
 'curva em L (O->N)':            ('ON',   [(0.8,3.5),(3.5,3.5),(3.5,6.2)]),
}
for nome,(bracos,poli) in casos.items():
    m=faz_mapa(bracos); pts=linha(poli)
    larg=al.larguras_no_caminho(m,pts)
    tre=al.achar_passagens(pts,larg,0.90)
    print('==', nome, '| trechos estreitos:', len(tre))
    for i0,i1 in tre:
        g=al.geometria_passagem(pts,larg,i0,i1)
        print('   de (%.2f,%.2f) a (%.2f,%.2f) eixo %.0f graus largura %.2f' % (pts[i0]+pts[i1]+(math.degrees(math.atan2(g['eixo'][1],g['eixo'][0])),g['largura'])), '| entrada (%.2f,%.2f) saida (%.2f,%.2f)'%(g['entrada']+g['saida']))
    et,info=al.planejar_etapas(m,pts,pts[-1])
    if et:
        print('   1a etapa: preparo (%.2f,%.2f) yaw %.0f' % (et[0]['x'],et[0]['y'],math.degrees(et[0]['yaw'])), '| etapas:', len(et), '| saida A (%.2f,%.2f) cruzamentos retos %d' % (info['saida']+(info['cruzamentos'],)))
        c=info.get('curva')
        if c: print('   CURVA: juncao (%.2f,%.2f) eixo B %.0f graus, sair para a %s, saida B (%.2f,%.2f), largura B %.2f' % (c['juncao']+(math.degrees(math.atan2(c['eixo_b'][1],c['eixo_b'][0])),'ESQUERDA' if c['lado']>0 else 'DIREITA')+c['saida_b']+(c['largura_b'],)))
    else:
        print('   planejar_etapas:', info)
