#!/usr/bin/env python3
"""Pagina web para mover a plataforma Y e medir o curso dela (y do plano da garra pela camera).

Precisa rodando: garra_real.launch.py e camera_cubo --tag-plataforma 256 (publica
/garra/plataforma_y/posicao). NAO rode junto com o plataforma_y.py: os dois mandam em
/garra/plataforma_y.

    python3 scripts/plataforma_y_web.py          # abre http://localhost:8091
    python3 scripts/plataforma_y_web.py --host 0.0.0.0   # outro computador da rede

Botoes (ou teclas a/d, A/D devagar): segure para andar, soltou para. A pagina reenvia o comando a
cada 100 ms enquanto o botao estiver apertado; sem reenvio por HOMEM_MORTO_S o servidor para a
plataforma (aba fechada, rede caiu). Os fins de curso do firmware continuam valendo.
y do plano = y do tag + DESLOCAMENTO (o mesmo --deslocamento do plataforma_y.py).
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from geometry_msgs.msg import Point
import rclpy
from std_msgs.msg import Int8

DESLOCAMENTO = -0.0291
TOPICO_CMD = '/garra/plataforma_y'
TOPICO_POS = '/garra/plataforma_y/posicao'
PERIODO = 0.1
HOMEM_MORTO_S = 0.3
LIMITES_ATUAIS = (-0.1213, 0.1904)   # --y-min / --y-max do plataforma_y.py (2026-11-24)

PAGINA = r'''<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Plataforma Y</title>
<style>
 :root{color-scheme:light dark;font-family:system-ui,sans-serif}
 body{max-width:640px;margin:2rem auto;padding:0 1rem}
 h1{font-size:1.3rem;margin:0 0 1rem}
 .y{font-size:3rem;font-variant-numeric:tabular-nums;font-weight:600}
 .sub{opacity:.7;font-variant-numeric:tabular-nums}
 .barra{position:relative;height:28px;border:1px solid #8888;border-radius:6px;margin:1.2rem 0}
 .barra div{position:absolute;top:0;bottom:0}
 #faixa{background:#3b82f633}#velho{border-left:2px dashed #e11d48;border-right:2px dashed #e11d48}
 #agora{width:3px;background:currentColor}
 .botoes{display:grid;grid-template-columns:repeat(4,1fr);gap:.5rem;margin:1rem 0}
 button{font-size:1.1rem;padding:1.1rem .3rem;border-radius:10px;border:1px solid #8886;
        background:#8881;touch-action:none;user-select:none;cursor:pointer}
 button.ativo{background:#3b82f6;color:#fff}
 .linha{display:flex;gap:.5rem;flex-wrap:wrap}.linha button{padding:.6rem 1rem;font-size:1rem}
 #estado{margin-left:.5rem}.aviso{color:#e11d48}
 ol{font-variant-numeric:tabular-nums}
</style></head><body>
<h1>Plataforma Y: curso pela câmera</h1>
<div class="y" id="y">–</div>
<div class="sub">mín <b id="min">–</b> · máx <b id="max">–</b> <span id="estado"></span></div>
<div class="barra" title="esquerda = +y (à direita na barra fica -y)">
 <div id="velho"></div><div id="faixa"></div><div id="agora"></div></div>
<div class="sub">tracejado vermelho: limites do plataforma_y (−0,1213 a +0,1904). Esquerda do robô = +y (lado esquerdo da barra).</div>
<div class="botoes">
 <button data-v="-2">◀◀<br><small>esq. devagar</small></button>
 <button data-v="-1">◀<br><small>esquerda</small></button>
 <button data-v="1">▶<br><small>direita</small></button>
 <button data-v="2">▶▶<br><small>dir. devagar</small></button>
</div>
<div class="linha"><button id="marcar">Marcar y atual</button><button id="zerar">Zerar mín/máx</button></div>
<ol id="marcas"></ol>
<script>
// esquerda (+y) desenhada a esquerda: posicao na barra = (YMAX - y)/(YMAX - YMIN)
const YMIN=-0.20, YMAX=0.25, pct=y=>100*(YMAX-y)/(YMAX-YMIN), f=v=>(v>=0?'+':'')+v.toFixed(4)+' m';
let cmd=0, timer=null;
function enviar(){fetch('/cmd?v='+cmd,{method:'POST'}).catch(()=>{});}
function iniciar(v,b){cmd=v;document.querySelectorAll('.botoes button').forEach(x=>x.classList.toggle('ativo',x===b));
  enviar();clearInterval(timer);timer=setInterval(enviar,100);}
function parar(){cmd=0;clearInterval(timer);timer=null;enviar();
  document.querySelectorAll('.botoes button').forEach(x=>x.classList.remove('ativo'));}
document.querySelectorAll('.botoes button').forEach(b=>{
  b.addEventListener('pointerdown',e=>{b.setPointerCapture(e.pointerId);iniciar(+b.dataset.v,b);});
  ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,parar));});
const teclas={a:-1,d:1,A:-2,D:2};
addEventListener('keydown',e=>{if(e.repeat)return;const v=teclas[e.key];if(v){iniciar(v,document.querySelector(`[data-v="${v}"]`));}
  if(e.key===' '){parar();e.preventDefault();} if(e.key==='m')marcar();});
addEventListener('keyup',e=>{if(teclas[e.key]===cmd)parar();});
addEventListener('blur',parar);
function marcar(){fetch('/marcar',{method:'POST'});}
document.getElementById('marcar').onclick=marcar;
document.getElementById('zerar').onclick=()=>fetch('/zerar',{method:'POST'});
const v=document.getElementById('velho');v.style.left=pct(0.1904)+'%';v.style.width=(pct(-0.1213)-pct(0.1904))+'%';
async function atualizar(){
 try{const r=await (await fetch('/estado')).json();
  const ok=r.y!==null&&r.idade<0.5;
  document.getElementById('y').textContent=ok?f(r.y):'sem tag';
  document.getElementById('y').className='y'+(ok?'':' aviso');
  document.getElementById('min').textContent=r.min===null?'–':f(r.min);
  document.getElementById('max').textContent=r.max===null?'–':f(r.max);
  document.getElementById('estado').textContent=r.cmd_nome;
  if(ok){const a=document.getElementById('agora');a.style.left=pct(r.y)+'%';}
  if(r.min!==null){const x=document.getElementById('faixa');x.style.left=pct(r.max)+'%';x.style.width=(pct(r.min)-pct(r.max))+'%';}
  document.getElementById('marcas').innerHTML=r.marcas.map(m=>'<li>'+f(m)+'</li>').join('');
 }catch(e){document.getElementById('estado').textContent='servidor fora do ar';}
 setTimeout(atualizar,150);}
atualizar();
</script></body></html>'''

NOMES = {-2: 'esquerda devagar', -1: 'esquerda', 0: 'parada', 1: 'direita', 2: 'direita devagar'}


class Estado:

    def __init__(self):
        self.trava = threading.Lock()
        self.y = None
        self.t_y = 0.0
        self.min = self.max = None
        self.cmd = 0
        self.t_cmd = 0.0
        self.marcas = []


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--porta', type=int, default=8091)
    ap.add_argument('--host', default='127.0.0.1',
                    help='0.0.0.0 abre para outros computadores da rede (qualquer um la move a plataforma)')
    args = ap.parse_args()

    rclpy.init()
    no = rclpy.create_node('plataforma_y_web')
    pub = no.create_publisher(Int8, TOPICO_CMD, 10)
    e = Estado()

    def ao_ver(p):
        with e.trava:
            e.y, e.t_y = p.y + DESLOCAMENTO, time.monotonic()
            e.min = e.y if e.min is None else min(e.min, e.y)
            e.max = e.y if e.max is None else max(e.max, e.y)

    no.create_subscription(Point, TOPICO_POS, ao_ver, 10)

    def publicar(_=None):
        with e.trava:
            if time.monotonic() - e.t_cmd > HOMEM_MORTO_S:
                e.cmd = 0
            cmd = e.cmd
        pub.publish(Int8(data=cmd))

    no.create_timer(PERIODO, publicar)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _resp(self, corpo, tipo='application/json'):
            dados = corpo.encode()
            self.send_response(200)
            self.send_header('Content-Type', tipo + '; charset=utf-8')
            self.send_header('Content-Length', str(len(dados)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(dados)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == '/':
                return self._resp(PAGINA, 'text/html')
            if u.path == '/estado':
                with e.trava:
                    r = dict(y=e.y, idade=time.monotonic() - e.t_y, min=e.min, max=e.max,
                             cmd_nome=NOMES[e.cmd], marcas=e.marcas)
                return self._resp(json.dumps(r))
            self.send_error(404)

        def do_POST(self):
            u = urlparse(self.path)
            with e.trava:
                if u.path == '/cmd':
                    v = int(parse_qs(u.query).get('v', ['0'])[0])
                    e.cmd, e.t_cmd = max(-2, min(2, v)), time.monotonic()
                elif u.path == '/marcar':
                    if e.y is not None and time.monotonic() - e.t_y < 0.5:
                        e.marcas.append(round(e.y, 4))
                        print(f'marcado y = {e.y:+.4f} m', flush=True)
                elif u.path == '/zerar':
                    e.min = e.max = None
                else:
                    return self.send_error(404)
            if u.path == '/cmd' and e.cmd == 0:
                pub.publish(Int8(data=0))   # para na hora, sem esperar o timer
            self._resp('{}')

    srv = ThreadingHTTPServer((args.host, args.porta), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'Abra http://localhost:{args.porta}' + (f' (escutando em {args.host})' if args.host != '127.0.0.1' else ''), flush=True)
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            pub.publish(Int8(data=0))
            time.sleep(0.05)
        srv.shutdown()
        with e.trava:
            if e.min is not None:
                print(f'Faixa vista: y de {e.min:+.4f} a {e.max:+.4f} m; marcas: {e.marcas}', flush=True)
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
