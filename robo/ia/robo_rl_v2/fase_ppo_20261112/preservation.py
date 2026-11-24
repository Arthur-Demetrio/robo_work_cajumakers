import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
ORIGINAL=ROOT.parent
PATH=ROOT/'preservation_before.json'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def capture():
 files=[]
 for name in ['robo_rl_v2','assets','experimentos_v2_20261110','diagnostico_20261111','docs','backups']:
  files.extend(p for p in (ORIGINAL/name).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
 files.extend(p for p in (ORIGINAL/'runs').rglob('*.zip') if p.is_file())
 PATH.write_text(json.dumps({str(p.relative_to(ORIGINAL)):sha(p) for p in files},indent=2))
def verify():
 before=json.loads(PATH.read_text());changed=[p for p,h in before.items() if not (ORIGINAL/p).exists() or sha(ORIGINAL/p)!=h]
 result=dict(files_verified=len(before),changed=changed,unchanged=not changed)
 (ROOT/'preservation_after.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
 if not PATH.exists():capture()
 print(json.dumps(verify()))
