"""Manual source-image annotations. No symmetric templates or automatic gap widening."""
from pathlib import Path
import json,hashlib,shutil,argparse
import numpy as np,cv2,yaml
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
reference=PROJECT/'assets/arenas/2016/referencia_usuario.png'
shutil.copy2(reference,ROOT/'source/provided_reference.png')
# Floor corners in original 1047x565 screenshot: left, front, right, back.
corners=[[117,265],[426,443],[890,177],[567,4]]
H=cv2.getPerspectiveTransform(np.float32(corners),np.float32([[0,0],[1,0],[1,1],[0,1]]))
# Circular surface is assumed parallel to floor. Its ellipse helps estimate axis ratio,
# not an absolute metric scale. Surface-to-floor offset is itself an estimate.
t=np.linspace(0,2*np.pi,256,endpoint=False)
circle=np.c_[362+36*np.cos(t),345+20*np.sin(t)]
unit=cv2.perspectiveTransform(circle[None].astype(np.float32),H)[0];cov=np.cov(unit.T)
parser=argparse.ArgumentParser();parser.add_argument('--area',type=float,default=100.);args=parser.parse_args()
if not 10<=args.area<=120:parser.error('Área deve estar entre 10 e 120 m²')
ratio=float(np.sqrt(cov[1,1]/cov[0,0]));area=args.area;width=float(np.sqrt(area*ratio));height=float(np.sqrt(area/ratio))
metric=np.diag([width,height,1.])@H

def convert(points):return cv2.perspectiveTransform(np.float32([points]),metric)[0].astype(float).tolist()
objects=[]
def wall(name,points):
 objects.append(dict(id=name,kind='wall',source_ground_polyline_px=points,geometry=dict(polyline=convert(points),width_m=.075),z_range_m=[0,.28],color='#596873',visual_screen_height_px=17,estimated=['width_m','height_m','ground_footline_occlusions']))
def polygon(name,kind,points,offset=3,z=(0,.10),color='#edf0f2'):
 # Visible top surface is elevated: approximate vertical screen displacement to floor.
 ground=(np.array(points)+[0,offset]).tolist()
 objects.append(dict(id=name,kind=kind,source_visible_polygon_px=points,source_ground_polygon_px=ground,geometry=dict(polygon=convert(ground)),z_range_m=list(z),color=color,visual_screen_height_px=offset,estimated=['height_m','screen_height_offset_px','absolute_dimensions']))
wall('W01_nicho_lateral_mesa',[[232,200],[257,215],[300,191]])
wall('W02_divisoria_mesa',[[274,254],[344,214]])
wall('W03_espinha_central',[[377,258],[589,137],[563,121]])
wall('W04_ramo_inferior',[[414,287],[537,216],[579,240]])
wall('W05_nicho_cobertura_direita',[[604,211],[576,195],[634,162],[745,226]])
wall('W06_cotovelo_fundo',[[446,99],[488,75],[528,99]])
wall('W07_nicho_fundo',[[568,25],[610,49],[570,72]])
wall('W08_nicho_direita',[[809,164],[850,188],[811,211]])
wall('W09_nicho_frontal',[[442,376],[481,398],[522,375]])
platforms=[
 ('P01_ao_lado_mesa',[[272,254],[305,235],[332,250],[299,269]]),
 ('P02_base_cobertura_esquerda',[[319,153],[348,136],[372,150],[342,167]]),
 ('P03_fundo_esquerdo',[[364,132],[395,114],[421,130],[392,147]]),
 ('P04_centro_esquerdo',[[413,182],[441,166],[469,183],[441,199]]),
 ('P05_espinha_fundo',[[508,137],[547,115],[568,127],[529,150]]),
 ('P06_fundo_direito_1',[[633,83],[661,67],[698,88],[669,105]]),
 ('P07_fundo_direito_2',[[694,120],[722,103],[758,124],[730,141]]),
 ('P08_base_cobertura_direita',[[643,192],[669,177],[694,193],[668,210]]),
 ('P09_centro_frontal',[[463,252],[492,235],[526,254],[498,272]]),
 ('P10_nicho_lateral_direito',[[781,187],[819,165],[838,176],[800,198]]),
 ('P11_frente_direita',[[613,296],[653,273],[681,290],[643,313]]),
 ('P12_mesa_precisao',[[657,268],[695,246],[725,263],[687,285]]),
 ('P13_nicho_frontal',[[468,370],[495,354],[510,363],[482,380]]),
 ('P14_viga_baixa_frontal',[[538,344],[586,317],[601,326],[552,353]]),
 ('P15_base_circular',[[319,348],[353,329],[386,347],[352,367]])]
for name,points in platforms:
 polygon(name,'platform',points)
 objects[-1]['service_candidate']=name not in ('P14_viga_baixa_frontal','P15_base_circular')
 if name=='P15_base_circular':objects[-1]['associated_with']='C01_plataforma_circular'
 if name=='P14_viga_baixa_frontal':objects[-1]['role_status']='UNKNOWN_LOW_STRUCTURE_NOT_ASSUMED_WORKSTATION'
polygon('T01_mesa_vermelha_tampo','table_top',[[167,223],[205,201],[298,255],[259,278]],offset=36,z=(.65,.70),color='#a2472c')
# Ground supports: three visible feet and one rear foot inferred from table parallelogram.
for i,point in enumerate([[169,259],[206,237],[297,291],[260,310]]):
 objects.append(dict(id=f'T01_pe_{i+1}',kind='support',geometry=dict(center=convert([point])[0],radius_m=.035),source_ground_point_px=point,z_range_m=[0,.65],color='#333333',visual_screen_height_px=0,estimated=['radius_m','rear_foot_position' if i==1 else 'foot_ground_position']))
polygon('E01_cobertura_esquerda','elevated_shelf',[[309,133],[348,109],[368,132],[331,155]],offset=30,z=(.31,.36),color='#9b9da0')
polygon('E02_cobertura_direita','elevated_shelf',[[647,167],[667,146],[705,169],[685,191]],offset=28,z=(.31,.36),color='#9b9da0')
# Pitched/sloped upper surfaces: ground projection uses separately estimated supports.
# One constant pixel shift would turn their footprints into implausible trapezoids.
for object_id,ground,offsets in [
 ('E01_cobertura_esquerda',[[309,163],[348,140],[368,152],[331,174]],[30,31,20,19]),
 ('E02_cobertura_direita',[[647,199],[667,187],[705,209],[685,221]],[32,41,40,30])]:
 obj=next(o for o in objects if o['id']==object_id)
 obj['geometry']['polygon']=convert(ground);obj['source_ground_polygon_px']=ground;obj['visual_screen_height_px']=offsets
 obj['estimated'].append('sloped_roof_ground_projection_from_supports')
for label,points in [('E01',[[309,163],[348,140],[368,152],[331,174]]),('E02',[[650,195],[668,184],[704,205],[686,216]])]:
 for i,point in enumerate(points):
  objects.append(dict(id=f'{label}_suporte_{i+1}',kind='support',geometry=dict(center=convert([point])[0],radius_m=.025),source_ground_point_px=point,z_range_m=[0,.31],color='#4b5054',visual_screen_height_px=0,estimated=['support_visibility','support_ground_position','radius_m']))
# Preserve the traced contour; do not force a perfectly round footprint after rectification.
polygon('C01_plataforma_circular','circular_object',np.c_[362+36*np.cos(t[::8]),340+20*np.sin(t[::8])].tolist(),offset=5,z=(.07,.15),color='#a9aaad')
config=dict(name='arena_2016_v2',status='visual_review_candidate_not_approved',description='reconstrução aproximada da arena RoboCup@Work 2016 baseada em referência visual e restrições geométricas conhecidas',source=dict(url='https://cbr.robocup.org.br/wp-content/uploads/2026/07/CBR_RoboCup_Work_Rulebook_2026.pdf',figure=1,pdf_page_index=8,source_file='source/provided_reference.png',source_type='user_provided_screenshot',pdf_original_extraction='PENDING_DOWNLOAD_BLOCKED',source_size_px=[1047,565],sha256=hashlib.sha256(reference.read_bytes()).hexdigest()),floor=dict(corners_px=corners,corner_order=['left','front','right','back'],corner_uncertainty_px=3,area_m2=area,area_is_official=False,allowed_area_m2=[10,120],size_m=[width,height],axis_ratio_method='assumed circular platform + ground homography; provisional',axis_ratio=ratio,boundary='outside_floor_blocked_no_perimeter_wall',minimum_passage_m=.8,homography_image_to_unit=H.tolist()),sensor=dict(lidar_height_m=.20,height_status='ESTIMATED_NOT_MEASURED',resolution_m=.025),robot=dict(footprint=[[.365,.315],[.365,-.315],[-.205,-.315],[-.205,.315]],body_z_range_m=[0,.55],body_height_status='ESTIMATED_NOT_MEASURED',clearance_m=.01),objects=objects,service_regions=[],random_obstacles=dict(enabled=False,reason='geometry fidelity review before randomization'))
(ROOT/'layout_draft.yaml').write_text(yaml.safe_dump(config,sort_keys=False,allow_unicode=True))
(ROOT/'annotations_px.json').write_text(json.dumps(dict(floor_corners=corners,objects=objects),indent=2))
print(width,height,ratio,len(objects))
