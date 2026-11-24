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
from tf2_ros import Buffer, TransformListener, TransformException

from .detection import extract_segments
from .tracking import Tracker
from .geometry import side_evidence


class StationTrackerNode(Node):
    def __init__(self):
        super().__init__('robo_station_tracker')
        for key, value in [
            ('scan_topic','/scan_front'), ('target_frame','map'),
            ('target_width',0.90), ('width_tolerance',0.15),
            ('max_distance',3.0), ('max_bearing_deg',85.0),
            ('max_gap',0.13), ('min_points',6), ('line_residual',0.035),
            ('center_gate',0.22), ('angle_gate_deg',22.0),
            ('track_max_age',12.0), ('viewpoint_spacing',0.25),
            ('max_tracks',30), ('max_scan_wait',1.0),
            ('depth_min',0.25),('depth_max',0.50),('corner_tolerance',0.10),
            ('memory_age',180.0)
        ]:
            self.declare_parameter(key, value)
        self.p = lambda name: self.get_parameter(name).value
        self.frame = self.p('target_frame')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.queue = deque(maxlen=12)
        self.tracker = Tracker(self.p('center_gate'),self.p('angle_gate_deg'),
                               self.p('track_max_age'),self.p('viewpoint_spacing'),
                               self.p('max_tracks'),self.p('memory_age'))
        self.sub = self.create_subscription(LaserScan,self.p('scan_topic'),self.receive,qos_profile_sensor_data)
        self.markers_pub = self.create_publisher(MarkerArray,'/station_tracker/markers',10)
        self.tracks_pub = self.create_publisher(String,'/station_tracker/tracks',10)
        self.create_timer(0.04,self.process_queue)
        self.create_timer(0.5,self.publish)
        self.get_logger().info('TRACKER v0.3: hipotese geometrica, observacao apenas, SEM MOVIMENTO.')

    def receive(self, scan):
        self.queue.append((time.monotonic(),scan))

    def process_queue(self):
        if not self.queue:
            return
        arrived,msg = self.queue[0]
        stamp = Time.from_msg(msg.header.stamp)
        if not self.tf_buffer.can_transform(self.frame,msg.header.frame_id,stamp,timeout=Duration(seconds=0)):
            if time.monotonic() - arrived > self.p('max_scan_wait'):
                self.queue.popleft()
                self.get_logger().warning('TF de uma leitura indisponivel; leitura descartada',throttle_duration_sec=5.0)
            return
        self.queue.popleft()
        try:
            tf = self.tf_buffer.lookup_transform(self.frame,msg.header.frame_id,stamp,timeout=Duration(seconds=0))
        except TransformException:
            return
        q=tf.transform.rotation
        heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        ct,st=math.cos(heading),math.sin(heading)
        ox,oy=tf.transform.translation.x,tf.transform.translation.y
        points=[]
        for i,r in enumerate(msg.ranges):
            if not math.isfinite(r) or r<msg.range_min or r>msg.range_max:
                points.append(None)
            else:
                angle=msg.angle_min+i*msg.angle_increment
                points.append((r*math.cos(angle),r*math.sin(angle)))
        segments=extract_segments(points,self.p('max_gap'),self.p('min_points'),self.p('line_residual'))
        candidates=[]
        for a,b,n,residual in segments:
            width=math.dist(a,b)
            cx,cy=(a[0]+b[0])/2,(a[1]+b[1])/2
            if abs(width-self.p('target_width')) > self.p('width_tolerance'):
                continue
            if math.hypot(cx,cy)>self.p('max_distance') or cx<=0 or abs(math.atan2(cy,cx))>math.radians(self.p('max_bearing_deg')):
                continue
            gx=ox+ct*cx-st*cy;gy=oy+st*cx+ct*cy
            yaw=(math.atan2(b[1]-a[1],b[0]-a[0])+heading)%math.pi
            depths=[depth for _,depth in side_evidence((a,b,n,residual),segments,
                         self.p('depth_min'),self.p('depth_max'),self.p('corner_tolerance'))]
            candidates.append((gx,gy,yaw,width,depths))
        # Use scan timestamp for track age; simulated/ROS timestamp consistency.
        now=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.tracker.update(candidates,(ox,oy),now)
        self.last_time=now

    def publish(self):
        tracks=self.tracker.tracks
        now=self.get_clock().now().nanoseconds/1e9
        if now > 0:
            self.tracker.tracks = [t for t in self.tracker.tracks if now-t.last_seen <= (self.p('memory_age') if t.side_hits>=3 else self.p('track_max_age'))]
            tracks=self.tracker.tracks
        valid=[]
        array=MarkerArray()
        erase=Marker();erase.action=Marker.DELETEALL;array.markers.append(erase)
        for track in tracks:
            age=max(0,now-track.last_seen)
            # Never report GEOMETRIA_CONFIRMADA without side/depth evidence.
            state='CANDIDATA_OBSERVADA' if age<=1.0 else 'NAO_VISIVEL'
            if track.n>=20 and len(track.views)>=2:
                state='RASTREADA_GEOMETRIA_INCOMPLETA' if age<=1.0 else 'NAO_VISIVEL'
            # Canto perpendicular repetido é apenas indício: paredes e quinas
            # podem produzir o mesmo padrão. Não autorizar docking/navegação.
            if track.side_hits>=3 and len(track.views)>=2:
                state='GEOMETRIA_COMPATIVEL_NAO_VERIFICADA' if age<=1.0 else 'OCULTA_REVALIDAR'
            valid.append({'id':f'TRACK_{track.id:02d}','estado':state,
                          'centro_map':[round(track.x,3),round(track.y,3)],
                          'largura_m':round(track.width,3),
                          'eixo_rad':round(track.yaw,3),'observacoes':track.n,
                          'pontos_vista':len(track.views),'idade_s':round(age,2),
                          'profundidade': round(sorted(track.side_depths)[len(track.side_depths)//2],3) if track.side_depths else 'NAO_MEDIDA',
                          'leituras_com_lateral':track.side_hits,'bancada_confirmada':False})
            base=track.id*10
            line=Marker();line.header.frame_id=self.frame;line.header.stamp=self.get_clock().now().to_msg()
            line.ns='station_tracker';line.id=base;line.type=Marker.LINE_STRIP
            line.action=Marker.ADD;line.pose.orientation.w=1.0
            line.scale.x=0.028
            line.color.r=1.0;line.color.g=0.72;line.color.b=0.16;line.color.a=0.85 if age<=1 else 0.35
            from geometry_msgs.msg import Point
            ct,st=math.cos(track.yaw),math.sin(track.yaw)
            for s in [-0.5,0.5]:
                point=Point();point.x=track.x+s*track.width*ct;point.y=track.y+s*track.width*st
                line.points.append(point)
            array.markers.append(line)
            label=Marker();label.header=line.header;label.ns='station_tracker';label.id=base+1
            label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.0
            label.pose.position.x=track.x;label.pose.position.y=track.y;label.pose.position.z=0.25
            label.scale.z=0.15;label.color.r=1.0;label.color.g=1.0;label.color.b=1.0;label.color.a=1.0
            label.text=f'TRACK_{track.id:02d} {track.n}obs {len(track.views)}v {track.side_hits}lat {state}'
            array.markers.append(label)
        msg=String();msg.data=json.dumps({'modo':'OBSERVACAO','confirmadas':0,'tracks':valid},ensure_ascii=False)
        self.tracks_pub.publish(msg)
        self.markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node=StationTrackerNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
