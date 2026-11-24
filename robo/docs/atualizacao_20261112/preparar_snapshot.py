import os,time,json,shutil,hashlib,subprocess,re,zipfile
from pathlib import Path
from datetime import datetime,timezone
BASE=Path(__file__).resolve().parent;SNAP=BASE/'snapshot';REPO=BASE/'repo'
os.nice(15)
a=sorted(os.sched_getaffinity(0));os.sched_setaffinity(0,[a[-3] if len(a)>2 else a[0]])
SNAP.mkdir(exist_ok=False)
SOURCES={
 'repositorio_local':Path('/home/marcos/Robo-WORK'),
 'workspaces/robo_jazzy_ws':Path('/home/marcos/robo_jazzy_ws'),
 'ia/robo_rl_v2':Path('/home/marcos/robo_rl_v2'),
 'simuladores/sim_encosto':Path('/home/marcos/sim_encosto'),
 'versoes/robo_work_teste':Path('/home/marcos/robo_work_teste'),
 'mapas_operacionais':Path('/home/marcos/mapas')}
SKIP={'.git','.venv','venv','build','install','install_nav2_principal','log','__pycache__','.pytest_cache','.ruff_cache','.mypy_cache','.agents','.claude','node_modules','.idea','.vscode'}
manifest=[];excluded=[];alerts=[];started=datetime.now(timezone.utc).isoformat()
secret=re.compile(rb'(?:gh[pousr]_[A-Za-z0-9_]{25,}|github_pat_[A-Za-z0-9_]{25,}|sk-(?:proj-)?[A-Za-z0-9_-]{35,}|AKIA[A-Z0-9]{16}|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----)')

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def copy(src,dst,label):
 dst.parent.mkdir(parents=True,exist_ok=True)
 if src.is_symlink():
  excluded.append({'source':str(src),'reason':'symlink; target not imported automatically'});return
 for attempt in range(5):
  before=src.stat();shutil.copy2(src,dst);after=src.stat()
  if (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns):break
  time.sleep(.1)
 else:raise RuntimeError(f'Arquivo muda continuamente: {src}')
 size=dst.stat().st_size
 if size<20*1024**2:
  data=dst.read_bytes()
  if secret.search(data):alerts.append({'path':label,'reason':'credential pattern; content not printed'})
 row={'path':label,'source':str(src),'size':size,'sha256':sha(dst),'source_mtime_ns':after.st_mtime_ns,'captured_utc':datetime.now(timezone.utc).isoformat()}
 if dst.suffix=='.zip':
  try:
   with zipfile.ZipFile(dst) as z:row['zip_crc_valid']=z.testzip() is None
  except zipfile.BadZipFile:row['zip_crc_valid']=False
 manifest.append(row)
for prefix,root in SOURCES.items():
 for folder,dirs,files in os.walk(root):
  for d in list(dirs):
   if d in SKIP:
    dirs.remove(d);excluded.append({'source':str(Path(folder)/d),'reason':'derived environment, build output, VCS metadata or machine configuration'})
  for name in sorted(files):
   p=Path(folder)/name
   if name=='.env' or name.startswith('.env.') and name!='.env.example' or p.suffix in ['.pem','.key','.pyc','.pyo']:
    excluded.append({'source':str(p),'reason':'credential file or Python cache'});continue
   label=str(Path(prefix)/p.relative_to(root));copy(p,SNAP/label,label)
 print('CAPTURED',prefix,flush=True)
for p in sorted(Path('/home/marcos').iterdir()):
 if p.is_file() and p.suffix in ['.sh','.py']:
  label='operacao/'+p.name;copy(p,SNAP/label,label)
print('Secret pattern alerts:',json.dumps(alerts),flush=True)
(BASE/'secret_scan.json').write_text(json.dumps({'alerts':alerts,'scanned_small_files':True},indent=2))
if alerts:raise RuntimeError('Review detected credential patterns before publishing')
# Capture Git history separately; existing worktrees are read only.
for prefix,root in SOURCES.items():
 repos=[root] if (root/'.git').exists() else []
 if prefix=='versoes/robo_work_teste':repos.extend(p.parent for p in root.glob('*/.git'))
 for repo in repos:
  dest=SNAP/'historicos_git'/(repo.name+'.bundle');dest.parent.mkdir(exist_ok=True)
  result=subprocess.run(['git','-C',str(repo),'bundle','create',str(dest),'--all'],capture_output=True,text=True)
  if result.returncode:
   if 'Refusing to create empty bundle' in result.stderr:
    excluded.append({'source':str(repo),'reason':'Repository has no refs; source captured, empty bundle omitted'});continue
   raise RuntimeError(result.stderr)
  manifest.append({'path':str(dest.relative_to(SNAP)),'source':str(repo),'size':dest.stat().st_size,'sha256':sha(dest),'type':'git_bundle_all_refs'})
# GitHub tree contains navigable source/reports; complete snapshots in release.
archived=[]
for row in manifest:
 p=SNAP/row['path'];parts=Path(row['path']).parts
 if row['size']>4*1024**2 or any(x in parts for x in ['runs','interrupcoes','bags','logs','historicos_git']) or p.suffix in ['.zip','.gz','.mcap','.db3','.data','.posegraph','.so','.bundle'] or p.name.startswith('events.out.tfevents') or any(x.startswith('build_') or x=='CMakeFiles' for x in parts) or p.suffix in ['.o','.a'] or p.name=='CMakeCache.txt':
  archived.append(row['path']);continue
 rel=Path(row['path'])
 target=REPO/(Path(*rel.parts[1:]) if rel.parts[0]=='repositorio_local' else rel)
 target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
metadata=REPO/'docs/atualizacao_20261112';metadata.mkdir(parents=True,exist_ok=True)
info={'started_utc':started,'finished_utc':datetime.now(timezone.utc).isoformat(),'account':'OttavioSilva','repository':'OttavioSilva/Robo-WORK','snapshot_semantics':'Per-file coherent capture; active training not paused. Logs/evaluations are a time-bounded snapshot, not final results.','sources':{k:str(v) for k,v in SOURCES.items()},'file_count':len(manifest),'total_bytes':sum(r['size'] for r in manifest),'files':manifest,'release_only_files':archived,'excluded':excluded}
(metadata/'manifesto.json').write_text(json.dumps(info,indent=2,ensure_ascii=False))
(BASE/'snapshot_info.json').write_text(json.dumps({k:v for k,v in info.items() if k not in ['files','excluded','release_only_files']},indent=2,ensure_ascii=False))
print('SNAPSHOT COMPLETE',json.dumps({'files':len(manifest),'bytes':info['total_bytes'],'release_only':len(archived)}),flush=True)
