"""Exercise terminal launch/stop/resume inside one Linux PID namespace."""
import json,os,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parents[2];run=root/'runs/lifecycle_verified'
def call(script,*args):
    return subprocess.run([str(root/script),'--run',str(run),*args],cwd=root,check=True)
def wait_file(name,predicate=lambda data:True,seconds=120):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        path=run/name
        if path.exists():
            data=json.loads(path.read_text())
            if predicate(data):return data
        time.sleep(.1)
    raise AssertionError(f'Timeout: {name}')
call('iniciar_treino.sh','--steps','1024','--n-steps','128','--max-steps','64','--checkpoint-every','128','--eval-every','10000','--threads','1','--cpus','1')
state=wait_file('process.json');pid=state['pid']
assert os.getsid(pid)==pid, 'Processo deve possuir sessão Linux própria'
assert len(os.sched_getaffinity(pid))==1, 'Afinidade deve limitar a um CPU'
wait_file('latest.json',lambda d:d['steps']>=128)
call('acompanhar_treino.sh');call('parar_treino.sh')
stopped=wait_file('status.json',lambda d:d['stopped'])
checkpoint=wait_file('latest.json');assert Path(checkpoint['model']).exists()
print('Parada segura:',stopped,flush=True)
# status from old run is stale until new training exits: wait final steps explicitly.
call('continuar_treino.sh','--steps','1280','--threads','1','--cpus','1','--eval-every','10000','--checkpoint-every','128')
completed=wait_file('status.json',lambda d:d['completed'] and d['steps']>=1280)
assert completed['steps']>stopped['steps']
(run/'lifecycle_result.json').write_text(json.dumps(dict(stopped=stopped,resumed=completed,session_detached=True,cpu_affinity=1),indent=2))
print('Continuação OK:',completed,flush=True)
