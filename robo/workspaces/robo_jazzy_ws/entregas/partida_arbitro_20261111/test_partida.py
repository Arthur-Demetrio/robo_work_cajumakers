import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest
ROOT=Path('/home/marcos/robo_jazzy_ws')
SCRIPT=ROOT/'ferramentas/conferir_supervisor_ativo.py'
spec=importlib.util.spec_from_file_location('conferir',SCRIPT);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)


def evidencia():return dict(servico=True,principal=True,modo='observar',publicadores=['/arbitro_nav_principal'],heartbeat=['/alinhador_passagem'])


def test_ok():assert mod.avaliar(evidencia(),'observar')==[]


@pytest.mark.parametrize('chave,valor',[('servico',False),('principal',False),('modo','aplicar'),('publicadores',[]),('publicadores',['/arbitro_nav_principal']*2),('publicadores',['/outro']),('heartbeat',[])])
def test_falha_fechada(chave,valor):
    d=evidencia();d[chave]=valor;assert mod.avaliar(d,'observar')


def test_shell_real_delega_verificacao_e_preserva_falha(tmp_path):
    anterior=ROOT/'entregas/correcao_subida_supervisor_20261111/test_subida.py'
    s=importlib.util.spec_from_file_location('anterior',anterior);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
    startup,env,home=m.preparar(tmp_path)
    helper=home/'robo_jazzy_ws/ferramentas/conferir_supervisor_ativo.py'
    helper.write_text("import os,sys\nfrom pathlib import Path\nwith open(os.environ['CHAMADAS'],'a') as f:f.write('VERIFICACAO_PERSISTENTE\\n')\nsys.exit(int(os.environ.get('VERIFICACAO_FALHA','0')))\n")
    env['VERIFICACAO_FALHA']='1'
    r=subprocess.run(['bash',str(startup),'--mapeando','arena'],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=25)
    assert r.returncode==1,r.stdout
    assert 'VERIFICACAO_PERSISTENTE' in (home/'calls').read_text()
    assert 'ROBO PRONTO' not in r.stdout


def test_ros_real_isolado_sem_hardware(tmp_path):
    env=dict(os.environ,ROS_DOMAIN_ID='171',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    stub=tmp_path/'alinhador_passivo.py'
    stub.write_text('''import rclpy,sys,json
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from pathlib import Path
rclpy.init();n=Node('alinhador_passagem' if len(sys.argv)>2 else 'intruso_teste')
if len(sys.argv)>2:
 n.declare_parameter('nav2_principal',True);n.declare_parameter('supervisor_modo','observar')
 p=n.create_publisher(TwistStamped,'/navegacao/heartbeat',1)
 valores=[]
 def cmd(m):
  valores.append([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]);Path(sys.argv[1]).write_text(json.dumps(valores))
 n.create_subscription(TwistStamped,'/cmd_vel_nav',cmd,10)
else:p=n.create_publisher(TwistStamped,'/cmd_vel_nav',1)
try:rclpy.spin(n)
except (KeyboardInterrupt,ExternalShutdownException):pass
finally:n.destroy_node();rclpy.try_shutdown()
''')
    processos=[];logs=[]
    def iniciar(cmd,nome):
        f=(tmp_path/(nome+'.log')).open('w');logs.append(f)
        p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT);processos.append(p);return p
    def probe(timeout):
        r=subprocess.run([sys.executable,str(SCRIPT),'--timeout',str(timeout)],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout+8)
        return r
    try:
        samples=tmp_path/'cmd.json'
        fake=iniciar([sys.executable,str(stub),str(samples),'parametros'],'alinhador')
        arbitro=iniciar([sys.executable,str(ROOT/'src/robo_navigation/scripts/arbitro_nav_principal.py')],'arbitro')
        r=probe(12);assert r.returncode==0,r.stdout
        assert arbitro.poll() is None
        valores=json.loads(samples.read_text());assert len(valores)>=2 and all(c==[0,0,0] for c in valores)
        intruso=iniciar([sys.executable,str(stub),'sem_parametros'],'duplicado')
        time.sleep(2)
        r=probe(4);assert r.returncode==1 and 'exatamente um publisher' in r.stdout,r.stdout
        intruso.send_signal(signal.SIGINT);intruso.wait(timeout=5)
        arbitro.send_signal(signal.SIGINT);assert arbitro.wait(timeout=5)==0
        assert 'Traceback' not in (tmp_path/'arbitro.log').read_text()
        r=probe(3);assert r.returncode==1,r.stdout
    finally:
        for p in processos:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
