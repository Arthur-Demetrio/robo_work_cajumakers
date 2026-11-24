"""Gera o mundo SDF da bancada de encosto a partir do service_areas.yaml (frame map = mundo Gazebo).

Geometria (CLAUDE.md, medida):
  - footprint x -0,215..0,375, y +-0,325 (0,59 x 0,65 m); para-choque usado pelo codigo em x=0,365;
  - LiDAR em base_link (0,33; 0,005); plano do feixe abaixo do tampo da mesa (mesa com 0,10 m);
  - face da WS: centro (x, y), yaw = rumo do robo de frente para ela; mesa se estende alem da face.
O robo e um corpo cinematico (VelocityControl, sem gravidade, 1 cm acima do chao): o movimento real
e decidido pela ponte_sim (modelo do motor), nao pela fisica de roletes mecanum.
"""
import math

FOOT = (-0.215, 0.375, -0.325, 0.325)
LIDAR = (0.33, 0.005)
Z_LIDAR = 0.12   # acima do corpo: o gpu_lidar tem uma pequena abertura vertical
ALTURA_CORPO = 0.04       # corpo abaixo do feixe: o LiDAR do Gazebo nao enxerga o proprio robo
ESTACIONAMENTO = (-20.0, -20.0)   # onde ficam os obstaculos fora de uso


def _box(nome, x, y, yaw, sx, sy, sz, cor='0.6 0.45 0.3 1', estatico=True):
    return f'''
    <model name="{nome}"><static>{'true' if estatico else 'false'}</static>
      <pose>{x:.4f} {y:.4f} {sz / 2:.4f} 0 0 {yaw:.5f}</pose>
      <link name="l">
        <collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{cor}</ambient><diffuse>{cor}</diffuse></material></visual>
      </link>
    </model>'''


def mesas_sdf(areas, erro=(0.0, 0.0, 0.0), degrau=0.017, largura=None):
    """Cada WS = 2 metades (degrau: a metade esquerda fica 'degrau' m mais funda, como os ~1,7 cm
    medidos na WS_1). erro = (dx, dy, dyaw) da mesa real em relacao ao cadastro (frame map)."""
    out = []
    for nome, a in areas.items():
        f = a.get('face')
        if not f:
            continue
        mesa = a.get('table', {})
        prof = float(mesa.get('depth', 0.5))
        larg = float(largura or f.get('largura_medida', mesa.get('width', 0.8)))
        alt = max(float(mesa.get('height', 0.10)), Z_LIDAR + 0.13)   # so precisa cortar o feixe
        psi = f['yaw'] + erro[2]
        c, s = math.cos(psi), math.sin(psi)
        fx, fy = f['x'] + erro[0], f['y'] + erro[1]
        for lado, fundo in ((-1, 0.0), (1, degrau)):     # lado +1 = esquerda de quem olha a mesa
            # centro da metade: meia largura para o lado, metade da profundidade para dentro
            ox = (prof / 2 + fundo)
            oy = lado * larg / 4
            out.append(_box('%s_%s' % (nome, 'esq' if lado > 0 else 'dir'),
                            fx + c * ox - s * oy, fy + s * ox + c * oy, psi, prof, larg / 2, alt))
    return ''.join(out)


def robo_sdf(x=0.0, y=0.0, yaw=0.0):
    cx, sx = (FOOT[0] + FOOT[1]) / 2, FOOT[1] - FOOT[0]
    sy = FOOT[3] - FOOT[2]
    return f'''
    <model name="robo">
      <pose>{x:.4f} {y:.4f} 0.01 0 0 {yaw:.5f}</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial><mass>20</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="corpo"><pose>{cx:.4f} 0 {ALTURA_CORPO / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {ALTURA_CORPO:.3f}</size></box></geometry></collision>
        <visual name="corpo"><pose>{cx:.4f} 0 {ALTURA_CORPO / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {ALTURA_CORPO:.3f}</size></box></geometry>
          <material><ambient>0.2 0.3 0.8 1</ambient><diffuse>0.2 0.3 0.8 1</diffuse></material></visual>
        <visual name="frente"><pose>0.365 0 0.045 0 0 0</pose>
          <geometry><box><size>0.01 {sy:.3f} 0.01</size></box></geometry>
          <material><ambient>1 0.2 0.2 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material></visual>
        <sensor name="lidar" type="gpu_lidar">
          <pose>{LIDAR[0]} {LIDAR[1]} {Z_LIDAR - 0.01:.3f} 0 0 0</pose>
          <update_rate>5.9</update_rate>
          <topic>/sim/scan</topic>
          <lidar><scan><horizontal><samples>180</samples><resolution>1</resolution>
            <min_angle>-3.141592653589793</min_angle><max_angle>3.106686068549907</max_angle></horizontal></scan>
            <range><min>0.02</min><max>8.0</max><resolution>0.001</resolution></range></lidar>
        </sensor>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <topic>/sim/vel_real</topic>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <dimensions>2</dimensions>
        <odom_frame>map</odom_frame><robot_base_frame>base_link</robot_base_frame>
        <odom_topic>/sim/verdade</odom_topic><odom_publish_frequency>50</odom_publish_frequency>
      </plugin>
    </model>'''


def mundo_sdf(areas, erro=(0.0, 0.0, 0.0), degrau=0.017, robo=(0.0, 0.0, 0.0), n_obst=2):
    obst = ''.join(_box('obst_%d' % i, ESTACIONAMENTO[0] - i, ESTACIONAMENTO[1], 0.0, 0.10, 0.10, 0.20,
                        cor='0.9 0.1 0.1 1') for i in range(n_obst))
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="bancada">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <light type="directional" name="sol"><pose>0 0 10 0 0 0</pose><direction>-0.3 0.2 -1</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse></light>
    <model name="chao"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
    </link></model>
    {mesas_sdf(areas, erro, degrau)}
    {obst}
    {robo_sdf(*robo)}
  </world>
</sdf>
'''
