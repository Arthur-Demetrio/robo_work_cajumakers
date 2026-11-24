"""Extract original embedded image from PDF page 9 without increasing its resolution."""
import argparse,subprocess,hashlib,json,shutil
from pathlib import Path
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('--pdf',type=Path,required=True);a=p.parse_args()
if not a.pdf.is_file():p.error('PDF local não encontrado. Baixe o documento ou anexe-o à conversa.')
root=Path(__file__).resolve().parent/'source';folder=root/'pdf_images';folder.mkdir(exist_ok=True)
subprocess.run(['pdfimages','-f','9','-l','9','-png',str(a.pdf),str(folder/'page9')],check=True)
images=list(folder.glob('page9-*.png'))
if not images:raise SystemExit('Página sem imagem embutida detectável; requer inspeção manual, sem alegar extração concluída.')
source=max(images,key=lambda f:Image.open(f).width*Image.open(f).height)
shutil.copy2(source,root/'figure_1_embedded.png')
subprocess.run(['pdftoppm','-f','9','-l','9','-singlefile','-r','200','-png',str(a.pdf),str(root/'page9_render')],check=True)
metadata=dict(pdf_sha256=hashlib.sha256(a.pdf.read_bytes()).hexdigest(),page_index=8,embedded_image=str(source.name),native_resolution=list(Image.open(source).size),method='pdfimages native image extraction; page rendered separately at 200 dpi',annotation_registration='PENDING: source annotations still refer to the provided screenshot')
(root/'pdf_extraction.json').write_text(json.dumps(metadata,indent=2));print(json.dumps(metadata,indent=2))
