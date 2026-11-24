"""Tracker ROS2 Jazzy v0.4.1 em modo SOMENTE OBSERVACAO.

Tópicos novos para executar lado a lado com a v0.3, sem integracao
com station manager, comandos de velocidade ou objetivos Nav2.
"""
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .detection import extract_segments
from .geometry import side_evidence,continuation
from .tracking import Tracker,Observation
from .rectangle import rectangle_hypothesis


class StationTracker(Node):
    def __init__(self):
        super().__init__('robo_station_tracker_v041')
        for k,v in [
            ('scan_topic','/scan_front'),('target_frame','map'),
            ('target_width',.90),('width_tolerance',.15),
            ('partial_width_min',.45),('max_distance',3.0),
            ('max_bearing_deg',85.0),('max_gap',.13),('adaptive_gap',True),
            ('min_points',6),('line_residual',.035),
            ('max_scan_wait',1.0),('depth_min',.25),('depth_max',.50),
            ('corner_tolerance',.10),('angle_gate_deg',15.0),
            ('offset_gate',.10),('gap_gate',.12),
            ('short_age',12.0),('memory_age',180.0),
            ('max_tracks',40),('view_spacing',.23),
            ('jump_translation',.70),('jump_rotation_deg',45.0)
        ]:self.declare_parameter(k,v)
        self.p=lambda k:self.get_parameter(k).value
        self.frame=self.p('target_frame')
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.queue=deque(maxlen=12)
        self.tracker=Tracker(self.p('angle_gate_deg'),self.p('offset_gate'),self.p('gap_gate'),
                             self.p('short_age'),self.p('memory_age'),self.p('max_tracks'),self.p('view_spacing'))
        self.sub=self.create_subscription(LaserScan,self.p('scan_topic'),self.receive,qos_profile_sensor_data)
        self.markers_pub=self.create_publisher(MarkerArray,'/station_tracker_v041/markers',10)
        self.tracks_pub=self.create_publisher(String,'/station_tracker_v041/tracks',10)
        self.reset_service=self.create_service(Trigger,'/station_tracker_v041/reset',self.reset)
        self.create_timer(.04,self.process_queue)
        self.create_timer(.5,self.publish)
        self.last_tf_pose=None;self.last_stamp=None
        self.get_logger().info('TRACKER v0.4.1: observacao independente, sem movimentos, sem cadastro automatico.')

    def reset(self,request,response):
        self.tracker.reset();self.queue.clear()
        self.last_tf_pose=None;self.last_stamp=None
        response.success=True;response.message='Memoria de hipoteses limpa; nenhum dado do SLAM foi alterado.'
        return response

    def receive(self,msg):
        self.queue.append((time.monotonic(),msg))

    def process_queue(self):
        if not self.queue:return
        arrived,msg=self.queue[0]
        stamp=Time.from_msg(msg.header.stamp)
        if not self.tf_buffer.can_transform(self.frame,msg.header.frame_id,stamp,
                                            timeout=Duration(seconds=0)):
            if time.monotonic()-arrived>self.p('max_scan_wait'):
                self.queue.popleft()
                self.get_logger().warning('TF no instante do scan indisponivel; descartado',throttle_duration_sec=5.)
            return
        self.queue.popleft()
        try:
            transform=self.tf_buffer.lookup_transform(self.frame,msg.header.frame_id,stamp,
                                                       timeout=Duration(seconds=0))
        except TransformException:return
        rot=transform.transform.rotation
        yaw=math.atan2(2*(rot.w*rot.z+rot.x*rot.y),1-2*(rot.y**2+rot.z**2))
        cx,cy=transform.transform.translation.x,transform.transform.translation.y
        now=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.last_tf_pose and self.last_stamp is not None:
            dt=now-self.last_stamp
            dist=math.hypot(cx-self.last_tf_pose[0],cy-self.last_tf_pose[1])
            diff=abs((yaw-self.last_tf_pose[2]+math.pi)%(2*math.pi)-math.pi)
            if dt<-.5 or (0<dt<1.0 and (dist>self.p('jump_translation') or diff>math.radians(self.p('jump_rotation_deg')))):
                self.tracker.reset()
                self.get_logger().warning('Salto temporal ou de TF detectado: memoria zerada; revise SLAM.')
        self.last_tf_pose=(cx,cy,yaw);self.last_stamp=now
        cos,sin=math.cos(yaw),math.sin(yaw)
        def world(point):
            x,y=point
            return (cx+cos*x-sin*y,cy+sin*x+cos*y)
        points=[]
        for i,r in enumerate(msg.ranges):
            if not math.isfinite(r) or r<msg.range_min or r>msg.range_max:points.append(None)
            else:
                angle=msg.angle_min+i*msg.angle_increment
                points.append((r*math.cos(angle),r*math.sin(angle)))
        segments=extract_segments(points,msg.angle_increment,self.p('max_gap'),
                                  self.p('min_points'),self.p('line_residual'),self.p('adaptive_gap'))
        obs=[]
        for seg in segments:
            a,b,n,residual=seg
            width=math.dist(a,b)
            if not self.p('partial_width_min')<=width<=self.p('target_width')+self.p('width_tolerance'):
                continue
            mx,my=(a[0]+b[0])/2,(a[1]+b[1])/2
            if mx<=0 or math.hypot(mx,my)>self.p('max_distance') or abs(math.atan2(my,mx))>math.radians(self.p('max_bearing_deg')):
                continue
            lateral=side_evidence(seg,segments,self.p('depth_min'),self.p('depth_max'),self.p('corner_tolerance'))
            is_wall=continuation(seg,segments)
            obs.append(Observation(world(a),world(b),lateral,is_wall))
        self.tracker.update(obs,(cx,cy),now)

    def publish(self):
        now=self.get_clock().now().nanoseconds*1e-9
        self.tracker.expire(now)
        out=[];markers=MarkerArray()
        er=Marker();er.action=Marker.DELETEALL;markers.markers.append(er)
        for t in self.tracker.tracks:
            state=self.tracker.status(t,now)
            rect=rectangle_hypothesis(t,self.tracker.tracks,now)
            age=max(0.,now-t.last_seen)
            out.append(dict(id=f'TRACK_{t.id:02d}',estado=state,
                            centro_map=[round(x,3) for x in t.center],
                            largura_observada_m=round(t.width,3),
                            eixo_rad=round(t.axis,3),observacoes=t.n,
                            pontos_vista=len(t.views),
                            pontos_vista_com_lateral=len(t.depth_views),
                            idade_s=round(age,2),profundidade_observada_m=(round(t.depth,3) if t.depth else None),
                            leituras_com_lateral=t.side_hits,continuidades_parede=t.wall_hits,
                            suspeita_parede=t.suspected_wall,bancada_confirmada=False,
                            hipotese_retangulo=rect))
            line=Marker()
            line.header.frame_id=self.frame
            line.header.stamp=self.get_clock().now().to_msg()
            line.ns='tracker_v041';line.id=t.id*10;line.type=Marker.LINE_STRIP
            line.action=Marker.ADD;line.pose.orientation.w=1.0;line.scale.x=.028
            if t.suspected_wall:line.color.r=1.;line.color.g=.25;line.color.b=.25
            elif t.side_hits:line.color.r=.20;line.color.g=.75;line.color.b=.78
            else:line.color.r=1.;line.color.g=.72;line.color.b=.17
            line.color.a=.9 if age<=1 else .35
            for p in (t.p0,t.p1):
                pt=Point();pt.x=float(p[0]);pt.y=float(p[1]);line.points.append(pt)
            markers.markers.append(line)
            label=Marker();label.header=line.header;label.ns='tracker_v041';label.id=t.id*10+1
            label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD
            label.pose.orientation.w=1.0;label.pose.position.x=t.center[0]
            label.pose.position.y=t.center[1];label.pose.position.z=.24
            label.scale.z=.13;label.color.r=1.;label.color.g=1.;label.color.b=1.;label.color.a=1.
            label.text=f'TRACK_{t.id:02d} {t.n}obs {len(t.views)}v {t.side_hits}lat {state}'
            markers.markers.append(label)
        msg=String();msg.data=json.dumps({'modo':'OBSERVACAO','versao':'v0.4.1','confirmadas':0,'tracks':out},ensure_ascii=False)
        self.tracks_pub.publish(msg);self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node=StationTracker()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
