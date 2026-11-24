import os,json,tarfile,hashlib,time
from pathlib import Path
B=Path(__file__).resolve().parent;A=B/'artifacts';M=B/'repo/docs/atualizacao_20261112/manifesto.json'
os.nice(15);cpus=sorted(os.sched_getaffinity(0));os.sched_setaffinity(0,[cpus[-3] if len(cpus)>2 else cpus[0]])
expected={r['path']:r for r in json.loads(M.read_text())['files']};seen=set();errors=[]
for p in sorted(A.glob('*.tar.gz')):
 n=0;start=time.time()
 with tarfile.open(p,'r|gz') as tar:
  for member in tar:
   if not member.isfile():continue
   row=expected.get(member.name)
   if row is None:errors.append({'file':member.name,'reason':'not in manifest'});continue
   h=hashlib.sha256();f=tar.extractfile(member)
   for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
   if h.hexdigest()!=row['sha256'] or member.size!=row['size']:errors.append({'file':member.name,'reason':'size/hash mismatch'})
   seen.add(member.name);n+=1
 print(json.dumps({'archive':p.name,'files_verified':n,'wall_s':time.time()-start}),flush=True)
errors.extend({'file':p,'reason':'missing from release archives'} for p in sorted(set(expected)-seen))
result={'files_verified':len(seen),'expected':len(expected),'errors':errors,'passed':not errors}
(B/'validacao/arquivos.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if errors:raise SystemExit(1)
