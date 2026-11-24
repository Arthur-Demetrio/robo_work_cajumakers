"""Detached Linux process control; equivalent to tmux for training lifetime."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
from .maps import ROOT


def main():
    p=argparse.ArgumentParser();p.add_argument('operation',choices=['start','resume','status','stop','evaluate'])
    p.add_argument('--run',default='runs/principal');p.add_argument('--cpus',type=int,default=2)
    a,extra=p.parse_known_args();run=Path(a.run).resolve()
    if a.operation in ('start','resume'):
        run.mkdir(parents=True,exist_ok=True)
        command=[sys.executable,'-u','-m','robo_rl_v2.train','--run',str(run),*extra]
        if a.operation=='resume':command.append('--resume')
        available=sorted(os.sched_getaffinity(0))
        if a.cpus<1:p.error('--cpus deve ser positivo')
        def limits():
            os.sched_setaffinity(0,available[:a.cpus]);os.nice(10);signal.signal(signal.SIGHUP,signal.SIG_IGN)
        environment=os.environ.copy();environment.update(OMP_NUM_THREADS=str(a.cpus),MKL_NUM_THREADS=str(a.cpus),OPENBLAS_NUM_THREADS=str(a.cpus),MPLBACKEND='Agg')
        with (run/'terminal.log').open('ab') as log:
            child=subprocess.Popen(command,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=log,
                                   start_new_session=True,preexec_fn=limits,env=environment)
        for _ in range(150):
            record=run/'process.json'
            if record.exists() and json.loads(record.read_text()).get('pid')==child.pid:break
            if child.poll() is not None:break
            time.sleep(.1)
        if child.poll() is not None:raise SystemExit(f'Processo terminou: veja {run}/terminal.log')
        print(f'Treino independente PID {child.pid}. Log: {run}/terminal.log')
    elif a.operation in ('status','stop'):
        record=run/'process.json'
        if not record.exists():
            print('Sem treino ativo registrado.');print((run/'status.json').read_text() if (run/'status.json').exists() else '');return
        state=json.loads(record.read_text());pid=int(state['pid'])
        try:cmd=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        except FileNotFoundError:print('Processo encerrado; registro antigo.');return
        if (b'robo_rl_v2.train' not in cmd or str(run).encode() not in cmd
                or Path(f'/proc/{pid}/stat').read_text().split()[21]!=state.get('start_ticks')):
            raise SystemExit('PID não corresponde ao treino; nenhum sinal enviado')
        if a.operation=='stop':
            os.kill(pid,signal.SIGTERM);print('Parada solicitada. Aguarde status.json e remoção de process.json.')
        else:
            print(f'Treino ativo PID {pid}');print((run/'latest.json').read_text() if (run/'latest.json').exists() else 'Primeiro checkpoint pendente')
            print(f'Log: tail -f {run}/terminal.log')
    else:
        model=json.loads((run/'latest.json').read_text())['model']
        if (run/'best.zip').exists():model=str(run/'best.zip')
        subprocess.run([sys.executable,'-m','robo_rl_v2.evaluate','--model',model,'--output',str(run/'evaluation_manual'),*extra],cwd=ROOT,check=True)

if __name__=='__main__':main()
