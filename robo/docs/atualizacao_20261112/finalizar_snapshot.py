from pathlib import Path
code=Path(__file__).with_name('preparar.py').read_text()
exec(code.split('for prefix,root in SOURCES.items():')[0].replace('SNAP.mkdir(exist_ok=False)',''))
for prefix,root in SOURCES.items():
 repos=[root] if (root/'.git').exists() else []
 if prefix=='versoes/robo_work_teste':repos.extend(p.parent for p in root.glob('*/.git'))
 for repo in repos:
  refs=subprocess.run(['git','-C',str(repo),'show-ref'],capture_output=True,text=True)
  if not refs.stdout.strip():excluded.append({'source':str(repo/'.git'),'reason':'Repository has no refs/commits; source files captured, no bundle possible'});continue
  dest=SNAP/'historicos_git'/(repo.name+'.bundle')
  if not dest.exists():
   subprocess.run(['git','-C',str(repo),'bundle','create',str(dest),'--all'],check=True)
  subprocess.run(['git','-C',str(repo),'bundle','verify',str(dest)],check=True,stdout=subprocess.DEVNULL)
for prefix,root in SOURCES.items():
 for folder,dirs,files in os.walk(root):
  for d in list(dirs):
   if d in SKIP:
    dirs.remove(d);excluded.append({'source':str(Path(folder)/d),'reason':'derived environment, build output, VCS metadata or machine configuration'})
  for name in files:
   p=Path(folder)/name
   if name=='.env' or name.startswith('.env.') and name!='.env.example' or p.suffix in ['.pem','.key','.pyc','.pyo'] or p.is_symlink():excluded.append({'source':str(p),'reason':'credential file, cache or symlink'})
for p in sorted(SNAP.rglob('*')):
 if not p.is_file():continue
 rel=p.relative_to(SNAP);label=str(rel);source=None
 if rel.parts[0]=='historicos_git':source='Git bundle; all refs'
 elif rel.parts[0]=='operacao':source=str(Path('/home/marcos')/rel.name)
 else:
  for prefix,root in SOURCES.items():
   try:sub=rel.relative_to(prefix)
   except ValueError:continue
   source=str(root/sub);break
 row={'path':label,'source':source,'size':p.stat().st_size,'sha256':sha(p),'source_mtime_ns':p.stat().st_mtime_ns,'captured_utc':datetime.fromtimestamp(p.stat().st_ctime,timezone.utc).isoformat()}
 if p.suffix=='.zip':
  try:
   with zipfile.ZipFile(p) as z:row['zip_crc_valid']=z.testzip() is None
  except zipfile.BadZipFile:row['zip_crc_valid']=False
 manifest.append(row)
started=min(r['captured_utc'] for r in manifest)
exec(code[code.index('# GitHub tree contains'):])
