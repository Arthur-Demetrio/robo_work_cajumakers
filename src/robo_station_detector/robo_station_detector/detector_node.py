import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_euler

from .detection import extract_segments, select_candidate, approach_pose


class Detector(Node):
    def __init__(self):
        super().__init__('robo_station_detector')
        self.declare_parameter('scan_topic', '/scan_front')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('target_width', 0.90)
        self.declare_parameter('width_tolerance', 0.15)
        self.declare_parameter('max_distance', 3.0)
        self.declare_parameter('max_bearing_deg', 65.0)
        self.declare_parameter('max_gap', 0.13)
        self.declare_parameter('min_points', 6)
        self.declare_parameter('line_residual', 0.035)
        self.declare_parameter('front_offset', 0.365)
        self.declare_parameter('clearance', 0.20)
        self.declare_parameter('require_stable_scans', 5)
        self.declare_parameter('stable_center_tolerance', 0.13)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(MarkerArray, '/station_detector/markers', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/station_detector/approach_pose', 10)
        self.center_pub = self.create_publisher(PointStamped, '/station_detector/center', 10)
        self.status_pub = self.create_publisher(String, '/station_detector/status', 10)
        self.last_center = None
        self.streak = 0
        self.frame = self.get_parameter('target_frame').value
        self.pending_scans = deque(maxlen=12)
        self.sub = self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.on_scan,qos_profile_sensor_data)
        self.tf_timer = self.create_timer(0.04, self.process_pending)
        self.get_logger().info('Detector de caixa: OBSERVACAO apenas; nenhum comando de movimento. Marcadores em /station_detector/markers')

    def on_scan(self, msg):
        # Recebe leituras sem bloquear o executor ROS.
        self.pending_scans.append((time.monotonic(), msg))

    def process_pending(self):
        if not self.pending_scans:
            return

        received_at, msg = self.pending_scans[0]

        ready = self.tf_buffer.can_transform(
            self.frame,
            msg.header.frame_id,
            rclpy.time.Time.from_msg(msg.header.stamp),
            timeout=rclpy.duration.Duration(seconds=0.0)
        )

        if not ready:
            if time.monotonic() - received_at > 1.0:
                self.pending_scans.popleft()
                self.streak = 0
                self.last_center = None
                self.marker_pub.publish(self.clear_markers())
                self.get_logger().warning(
                    'TF_ATRASADO: leitura descartada apos 1 segundo',
                    throttle_duration_sec=1.0
                )
            return

        self.pending_scans.popleft()
        self.process_scan(msg)

    def process_scan(self, msg):
        pts=[]
        for i,r in enumerate(msg.ranges):
            if not math.isfinite(r) or r<msg.range_min or r>msg.range_max:
                pts.append(None)
            else:
                angle=msg.angle_min+i*msg.angle_increment
                pts.append((r*math.cos(angle),r*math.sin(angle)))
        segments=extract_segments(pts, self.get_parameter('max_gap').value, self.get_parameter('min_points').value,self.get_parameter('line_residual').value)
        candidate=select_candidate(segments, self.get_parameter('target_width').value,self.get_parameter('width_tolerance').value,self.get_parameter('max_distance').value,math.radians(self.get_parameter('max_bearing_deg').value))
        self.get_logger().info(
            f'DIAGNOSTICO: segmentos={len(segments)} '
            f'candidato={candidate is not None}',
            throttle_duration_sec=1.0
        )
        if candidate is None:
            self.get_logger().warning(
                f'SEM_CANDIDATO streak_anterior={self.streak}',
                throttle_duration_sec=1.0
            )
            self.streak=0
            self.last_center=None
            self.marker_pub.publish(self.clear_markers())
            return
        _,p0,p1,width,n = candidate
        ax,ay,yaw,cx,cy=approach_pose(p0,p1,self.get_parameter('clearance').value,self.get_parameter('front_offset').value)
        try:
            # Use TF at scan stamp; rejected when TF unavailable rather than misplacing a candidate.
            tf=self.tf_buffer.lookup_transform(self.frame,msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp),timeout=rclpy.duration.Duration(seconds=0.0))
        except TransformException as exc:
            self.get_logger().warning(
                f'TF_INDISPONIVEL: {exc}',
                throttle_duration_sec=1.0
            )
            self.streak=0
            self.marker_pub.publish(self.clear_markers())
            return
        def transform(p):
            q=tf.transform.rotation
            # planar yaw; yaw components are adequate for a 2-D LIDAR
            yaw_tf=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            x,y=p
            return (tf.transform.translation.x+math.cos(yaw_tf)*x-math.sin(yaw_tf)*y,
                    tf.transform.translation.y+math.sin(yaw_tf)*x+math.cos(yaw_tf)*y,yaw_tf)
        a=transform(p0); b=transform(p1); c=transform((cx,cy)); target=transform((ax,ay))
        yaw_world=yaw+target[2]
        desvio = (
            math.dist((c[0], c[1]), self.last_center)
            if self.last_center is not None else None
        )
        limite = self.get_parameter('stable_center_tolerance').value

        if desvio is not None and desvio <= limite:
            self.streak += 1
        else:
            self.get_logger().warning(
                f'CENTRO_MUDOU desvio={desvio} '
                f'limite={limite:.3f}m '
                f'centro_atual=({c[0]:.3f},{c[1]:.3f})',
                throttle_duration_sec=1.0
            )
            self.streak = 1
        self.last_center=(c[0],c[1])
        stable=self.streak>=self.get_parameter('require_stable_scans').value
        array=MarkerArray()
        def marker(mid,typ,scale,rgba):
            m=Marker()
            m.header.frame_id=self.frame
            m.header.stamp=msg.header.stamp
            m.ns='station_detector'
            m.id=mid
            m.type=typ
            m.action=Marker.ADD
            m.pose.orientation.w=1.0
            m.scale.x,m.scale.y,m.scale.z=scale
            m.color.r,m.color.g,m.color.b,m.color.a=rgba
            m.lifetime=rclpy.duration.Duration(seconds=0.7).to_msg()
            return m
        line=marker(1,Marker.LINE_STRIP,(0.035,0.0,0.0),(0.0,0.9,0.3,1.0))
        for x,y,_ in (a,b):
            from geometry_msgs.msg import Point
            pt=Point();pt.x=x;pt.y=y;line.points.append(pt)
        array.markers.append(line)
        center=marker(2,Marker.SPHERE,(0.11,0.11,0.11),(1.0,0.62,0.0,1.0))
        center.pose.position.x=c[0];center.pose.position.y=c[1]
        array.markers.append(center)
        arrow=marker(3,Marker.ARROW,(0.33,0.06,0.08),(0.15,0.45,1.0,1.0) if stable else (0.6,0.6,0.6,0.75))
        arrow.pose.position.x=target[0];arrow.pose.position.y=target[1]
        q=quaternion_from_euler(0,0,yaw_world)
        arrow.pose.orientation.x,arrow.pose.orientation.y,arrow.pose.orientation.z,arrow.pose.orientation.w=[float(v) for v in q]
        array.markers.append(arrow)
        self.marker_pub.publish(array)
        s=String();s.data=f'CANDIDATA largura={width:.2f}m centro=({c[0]:.2f},{c[1]:.2f}) scans_estaveis={self.streak} ESTAVEL={stable} (nao e identificacao definitiva)'
        self.status_pub.publish(s)
        if stable:
            p=PoseStamped();p.header.frame_id=self.frame;p.header.stamp=msg.header.stamp
            p.pose.position.x=target[0];p.pose.position.y=target[1]
            p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=[float(v) for v in q]
            self.pose_pub.publish(p)
            pt=PointStamped();pt.header=p.header;pt.point.x=c[0];pt.point.y=c[1]
            self.center_pub.publish(pt)

    def clear_markers(self):
        out=MarkerArray();m=Marker();m.action=Marker.DELETEALL;out.markers.append(m);return out


def main():
    rclpy.init()
    node=Detector()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__': main()
