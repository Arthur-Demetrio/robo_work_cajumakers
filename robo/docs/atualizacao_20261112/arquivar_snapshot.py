import os,tarfile,time,json,hashlib
from pathlib import Path
B=Path(__file__).resolve().parent;S=B/'snapshot';A=B/'artifacts';A.mkdir(exist_ok=True)
os.nice(15);cpus=sorted(os.sched_getaffinity(0));os.sched_setaffinity(0,[cpus[-4] if len(cpus)>3 else cpus[0]])
GROUPS={
'robo-work-ros-20261112.tar.gz':['workspaces'],
'robo-work-ia-20261112.tar.gz':['ia'],
'robo-work-mapas-simuladores-versoes-20261112.tar.gz':['mapas_operacionais','simuladores','versoes','operacao'],
'robo-work-referencia-historicos-git-20261112.tar.gz':['repositorio_local','historicos_git']}
index=[]
for name,dirs in GROUPS.items():
 start=time.time();tmp=A/(name+'.pending')
 with tarfile.open(tmp,'w:gz',compresslevel=1) as tar:
  for d in dirs:
   tar.add(S/d,arcname=d)
 target=A/name;tmp.rename(target)
 h=hashlib.sha256()
 with target.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 row={'name':name,'bytes':target.stat().st_size,'sha256':h.hexdigest(),'wall_s':time.time()-start};index.append(row);print(json.dumps(row),flush=True)
(A/'assets.json').write_text(json.dumps(index,indent=2))
(A/'SHA256SUMS').write_text(''.join(f"{r['sha256']}  {r['name']}\n" for r in index))
