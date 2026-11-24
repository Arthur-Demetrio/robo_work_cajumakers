"""Observation-only ROS adapter for the existing striped-tape detector."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import time
from datetime import datetime
import shutil
import numpy as np

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from . import detector
from .projection import load_calibration, project_observed_segment, preview_observation, resize_preview
from .point_probe import inspect_point, measured_check
from .stabilization import SegmentSmoother, PreviewDistanceFilter
from .visible_retirement import observed_ground


class VirtualWall(Node):
    def __init__(self):
        super().__init__('virtual_wall')
        defaults = dict(device='/dev/video2', width=640, height=480, fps=30.0,
                        processing_hz=15.0, fourcc='MJPG', show_windows=False,
                        calibrate=False,
                        image_topic='', camera_frame='camera_optical_frame',
                        hsv_file=str(Path(get_package_share_directory('robo_virtual_wall')) / 'config/hsv_fita.json'),
                        capture_directory='',
                        homography_file='', preview_homography_file='', preview_allow_resize=False, max_range_m=3.0,
                        roi_top=0.10, roi_bottom=0.85)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        if self.p('calibrate') and not self.p('show_windows'):
            raise ValueError('calibrate requires show_windows=true')
        if not 0 <= self.p('roi_top') < self.p('roi_bottom') <= 1:
            raise ValueError('Require 0 <= roi_top < roi_bottom <= 1')
        if min(self.p('fps'), self.p('processing_hz'), self.p('max_range_m')) <= 0:
            raise ValueError('Rates and max_range_m must be positive')
        with open(Path(self.p('hsv_file')).expanduser(), encoding='utf-8') as stream:
            self.hsv = json.load(stream)
        for key, limit in [('h_min', 179), ('h_max', 179), ('s_min', 255),
                           ('s_max', 255), ('v_min', 255), ('v_max', 255)]:
            value = self.hsv[key]
            if type(value) is not int or not 0 <= value <= limit:
                raise ValueError('Invalid HSV value: ' + key)
        if self.hsv['s_min'] > self.hsv['s_max'] or self.hsv['v_min'] > self.hsv['v_max']:
            raise ValueError('Inverted saturation/value limits')
        detector.ROI_TOPO = self.p('roi_top')
        if self.p('calibrate'):
            cv2.namedWindow('ajuste HSV')
            for key in self.hsv:
                cv2.createTrackbar(key, 'ajuste HSV', self.hsv[key],
                                  179 if key.startswith('h_') else 255, lambda _: None)
            self.get_logger().info('Ajuste HSV ao vivo: s salva com backup; q encerra.')
        self.calibration = None
        self.preview = None
        self.preview_size = None
        self.probe_calibration = None
        self.probe_selection = None
        self.probe_message = ''
        self.probe_input = None
        self.probe_checks = []
        self.probe_checks_path = None
        self.fitting_pixels = []
        if self.p('preview_homography_file'):
            preview_path = Path(self.p('preview_homography_file')).expanduser()
            self.preview = load_calibration(preview_path, preview=True)
            self.fitting_pixels = json.loads(preview_path.read_text()).get('pixels', [])
        if self.p('homography_file'):
            self.calibration = load_calibration(Path(self.p('homography_file')).expanduser())
        if self.p('show_windows'):
            cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('camera', self.on_camera_click)
            self.get_logger().info('Clique: medir ponto. M: informar trena. Botao direito: limpar.')
        self.bridge = CvBridge()
        self.annotated = self.create_publisher(Image, '~/image', qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(Image, '~/mask', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '~/detections', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '~/segments', 10)
        self.cap = None
        self.last_processed = 0.0
        self.segment_smoother = SegmentSmoother()
        self.preview_filter = PreviewDistanceFilter()
        self.create_timer(0.5, self.expire_markers)
        if self.p('image_topic'):
            self.create_subscription(Image, self.p('image_topic'), self.on_image,
                                     qos_profile_sensor_data)
        else:
            self.cap = cv2.VideoCapture(self.p('device'), cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap.release()
                raise RuntimeError('Cannot open V4L2 camera: ' + self.p('device'))
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.p('fourcc')))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.p('width'))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.p('height'))
            self.cap.set(cv2.CAP_PROP_FPS, self.p('fps'))
            self.get_logger().info('Negotiated camera: %.0fx%.0f %.1f FPS' % (
                self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                self.cap.get(cv2.CAP_PROP_FPS)))
            self.create_timer(1.0 / self.p('processing_hz'), self.capture)
        self.get_logger().info('Observation only; metric projection: %s' % bool(self.calibration))

    def on_camera_click(self, event, x, y, flags, userdata):
        if event == cv2.EVENT_RBUTTONDOWN:
            self.probe_selection = None
            self.probe_input = None
            self.probe_message = ''
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.probe_input = None
            self.probe_selection = None
            if self.probe_calibration is None:
                self.probe_message = 'Clique sem calibracao de previa disponivel'
                return
            sx = self.probe_calibration[1][0]/self.preview[1][0]
            sy = self.probe_calibration[1][1]/self.preview[1][1]
            fitting = np.asarray(self.fitting_pixels)*[sx,sy] if self.fitting_pixels else []
            self.probe_selection = inspect_point((x,y), self.probe_calibration, fitting)
            self.probe_message = ('Ponto fora da regiao calibrada' if self.probe_selection is None else '')
            if self.probe_selection:
                self.get_logger().info('Ponto clicado: '+json.dumps(self.probe_selection))

    def save_probe_check(self):
        try:
            check = measured_check(self.probe_selection, self.probe_input)
            checks = self.probe_checks + [check]
            directory = (Path(self.p('capture_directory')).expanduser() if self.p('capture_directory')
                         else Path.home() / '.ros' / 'virtual_wall' / 'captures')
            directory.mkdir(parents=True, exist_ok=True)
            if self.probe_checks_path is None:
                self.probe_checks_path = directory / ('checks-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
            data = dict(pixels=[c['pixel'] for c in checks],metres=[c['metres'] for c in checks],
                        checks=checks, image_size=list(self.probe_calibration[1]),
                        frame_id='virtual_wall_front', source_preview=self.p('preview_homography_file'),
                        validated=False)
            temporary = self.probe_checks_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(data,indent=2)+'\n', encoding='utf-8')
            temporary.replace(self.probe_checks_path)
            self.probe_checks = checks
            self.probe_message = 'Ponto salvo: erro %.1f cm; total %d' % (100*check['error_m'],len(checks))
            self.get_logger().info(self.probe_message+'; arquivo: '+str(self.probe_checks_path))
        except (ValueError, OSError) as error:
            self.probe_message = str(error)
            self.get_logger().warning(self.probe_message)
        self.probe_input = None

    def expire_markers(self):
        # Explicit removal also works when RViz does not advance ROS time.
        if time.monotonic() - self.last_processed > 1.0:
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.markers_pub.publish(MarkerArray(markers=[clear]))

    def capture(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning('Camera read failed', throttle_duration_sec=5.0)
            return
        header = Image().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.p('camera_frame')
        self.process(frame, header)

    def on_image(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not -0.1 <= now - stamp <= 0.5:
            return
        if time.monotonic() - self.last_processed < 1.0 / self.p('processing_hz'):
            return
        self.process(self.bridge.imgmsg_to_cv2(message, 'bgr8'), message.header)

    def process(self, frame, header):
        if time.monotonic() - self.last_processed > 0.5:
            self.segment_smoother.update([])
            self.preview_filter.update(None)
        self.last_processed = time.monotonic()
        height, width = frame.shape[:2]
        if self.p('calibrate'):
            for key in self.hsv:
                self.hsv[key] = cv2.getTrackbarPos(key, 'ajuste HSV')
        perception_valid = True
        mask, hsv = detector.mascara_fita(frame, self.hsv)
        mask[int(height * self.p('roi_bottom')):, :] = 0
        united = detector.unir_tracejado(mask)
        # Closing must not reintroduce excluded rows.
        united[:int(height * self.p('roi_top')), :] = 0
        united[int(height * self.p('roi_bottom')):, :] = 0
        annotated = frame.copy()
        walls = detector.detectar_paredes(united, mask, hsv, annotated)
        # Smooth only the display. Published measurements retain current pixels.
        display_walls = self.segment_smoother.update(walls)
        for p1, p2, _ in display_walls:
            cv2.line(annotated, p1, p2, (0, 255, 0), 3)
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = MarkerArray(markers=[clear])
        output = []
        for index, (p1, p2, angle) in enumerate(walls):
            entry = dict(pixels=[[int(v) for v in p1], [int(v) for v in p2]],
                         angle_deg=float(angle))
            if self.calibration:
                try:
                    points = project_observed_segment((p1, p2), self.calibration,
                                             (width, height), self.p('max_range_m'))
                except ValueError as error:
                    points = None
                    self.get_logger().warning(str(error), throttle_duration_sec=5.0)
                if points is not None:
                    entry['base_link_m'] = points.tolist()
                    marker = Marker()
                    marker.header.stamp = header.stamp
                    marker.header.frame_id = 'base_link'
                    marker.ns, marker.id = 'virtual_wall_observation', index
                    marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.03
                    marker.color.r, marker.color.a = 1.0, 1.0
                    marker.lifetime.sec = 1
                    marker.points = [Point(x=float(x), y=float(y), z=0.01) for x, y in points]
                    markers.markers.append(marker)
            output.append(entry)
        self.markers_pub.publish(markers)
        ground_view = None
        if self.calibration and perception_valid:
            support = observed_ground(frame, self.calibration, self.p('roi_top'),
                                      self.p('roi_bottom'), self.p('max_range_m'))
            if support is not None:
                ground_view = support.tolist()
        status = String()
        status.data = json.dumps(dict(observation_only=True,
            stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec),
            image_size=[width, height], metric_frame='base_link' if self.calibration and perception_valid else '',
            perception_valid=perception_valid,
            observed_ground_base_link_m=ground_view,
            self_mask=dict(mode='none', valid=True),
            detections=output))
        self.status_pub.publish(status)
        active_preview = self.preview
        scaled_preview = False
        preview_error = None
        if self.preview_size != (width, height):
            self.preview_filter.update(None)
            self.probe_selection = None
            self.probe_input = None
            self.probe_checks = []
            self.probe_checks_path = None
            self.preview_size = (width, height)
        if self.preview is not None and (width, height) != self.preview[1]:
            active_preview = None
            if self.p('preview_allow_resize'):
                try:
                    active_preview = resize_preview(self.preview, (width, height))
                    scaled_preview = True
                except ValueError as error:
                    preview_error = str(error)
            else:
                preview_error = 'Resolucao mudou: recalibrar ou habilitar previa reescalada'
        self.probe_calibration = active_preview
        if preview_error:
            self.preview_filter.update(None)
            self.get_logger().warning(preview_error, throttle_duration_sec=5.)
            cv2.putText(annotated, 'Distancia indisponivel: resolucao sem calibracao',
                        (5, 50), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,255), 2)
        if active_preview is not None:
            matrix, _, polygon = active_preview
            cv2.polylines(annotated, [polygon.astype(np.int32)], True, (255, 200, 0), 1)
            cv2.putText(annotated, ('PREVIA REESCALADA: conferir enquadramento e trena' if scaled_preview else
                         'PREVIA: origem na borda frontal; conferir com trena'),
                        (5, height-12), cv2.FONT_HERSHEY_SIMPLEX, .4, (0,255,255), 1)
            observations = [preview_observation((p1, p2), active_preview,
                            (width, height), self.p('max_range_m'))
                            for p1, p2, _ in walls]
            crossings = [o for o in observations if o['kind'] == 'crossing']
            value = min((o['forward_m'] for o in crossings), default=None)
            filtered = self.preview_filter.update(value)
            for observation in crossings:
                cv2.circle(annotated, observation['pixel'], 6, (0,255,255), -1)
            longitudinal = [o for o in observations if o['kind'] == 'longitudinal']
            if filtered is not None:
                label = 'Eixo na linha central (filtrado): %.1f cm' % (100*filtered)
            elif longitudinal:
                offset = min(longitudinal, key=lambda o: abs(o['lateral_m']))['lateral_m']
                label = 'Fita longitudinal: lateral %.1f cm (%s)' % (
                    100*abs(offset), 'esquerda' if offset >= 0 else 'direita')
            elif observations and all(o['kind'] == 'outside' for o in observations):
                label = 'Fita detectada FORA da regiao calibrada'
            elif observations:
                label = 'Fita detectada sem cruzamento transversal valido'
            else:
                label = 'Nenhuma fita detectada neste quadro'
            cv2.putText(annotated, label, (5, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0,255,255), 2)
        if self.probe_selection is not None:
            x, y = self.probe_selection['pixel']
            forward, lateral = self.probe_selection['predicted_m']
            cv2.drawMarker(annotated, (int(x),int(y)), (255,0,255), cv2.MARKER_CROSS, 18, 2)
            side = 'esquerda' if lateral >= 0 else 'direita'
            text = 'CLIQUE: frente %.1f cm | %s %.1f cm' % (100*forward,side,100*abs(lateral))
            cv2.putText(annotated,text,(5,75),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,0,255),1)
            if self.probe_selection['fitting_reference']:
                cv2.putText(annotated,'Cruz usada no ajuste: nao valida precisao',
                            (5,96),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
        if self.probe_input is not None:
            text = 'Trena cm: FRENTE LATERAL (+esq/-dir): '+self.probe_input+'_'
        else:
            text = self.probe_message or 'Clique: medir | M: trena | R: foto | botao direito: limpar'
        cv2.putText(annotated,text,(5,height-32),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
        cv2.putText(annotated, 'Fitas aceitas: %d' % len(walls), (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        for image, encoding, publisher in [(annotated, 'bgr8', self.annotated),
                                          (united, 'mono8', self.mask_pub)]:
            msg = self.bridge.cv2_to_imgmsg(image, encoding)
            msg.header = header
            publisher.publish(msg)
        if self.p('show_windows'):
            cv2.imshow('camera', annotated)
            cv2.imshow('mascara vermelha', mask)
            cv2.imshow('mascara unida', united)
            key = cv2.waitKey(1) & 0xff
            if ord('A') <= key <= ord('Z'):
                key += ord('a')-ord('A')
            if self.probe_input is not None:
                if key in (10,13):
                    self.save_probe_check()
                elif key == 27:
                    self.probe_input = None
                elif key in (8,127):
                    self.probe_input = self.probe_input[:-1]
                elif key in b'0123456789.,-+ ' and len(self.probe_input) < 32:
                    self.probe_input += chr(key)
                return
            if key == ord('m'):
                if self.probe_selection is not None:
                    self.probe_input = ''
                else:
                    self.probe_message = 'Clique primeiro em um ponto dentro da regiao azul'
            if key == ord('r'):
                directory = (Path(self.p('capture_directory')).expanduser() if self.p('capture_directory')
                         else Path.home() / '.ros' / 'virtual_wall' / 'captures')
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / ('raw-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.png')
                if cv2.imwrite(str(path), frame):
                    path.with_suffix('.json').write_text(json.dumps(dict(
                        hsv=self.hsv, clicked_point=self.probe_selection, image_size=[width, height],
                        fourcc=self.p('fourcc'), roi_top=self.p('roi_top'),
                        roi_bottom=self.p('roi_bottom')), indent=2), encoding='utf-8')
                    self.get_logger().info('Imagem original salva: %s' % path)
                else:
                    self.get_logger().error('Falha ao salvar imagem original: %s' % path)
            if key == ord('s') and self.p('calibrate'):
                if self.hsv['s_min'] > self.hsv['s_max'] or self.hsv['v_min'] > self.hsv['v_max']:
                    self.get_logger().warning('Limites S/V invertidos; ajuste antes de salvar.')
                else:
                    try:
                        path = Path(self.p('hsv_file')).expanduser()
                        backup = path.with_name(path.name + '.backup-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
                        with backup.open('xb') as output, path.open('rb') as source:
                            shutil.copyfileobj(source, output)
                        temporary = path.with_name(path.name + '.tmp-' + backup.name)
                        with temporary.open('x', encoding='utf-8') as stream:
                            json.dump(self.hsv, stream, indent=2)
                        temporary.replace(path)
                        self.get_logger().info('HSV salvo; backup: %s' % backup)
                    except OSError as error:
                        self.get_logger().error('Falha ao salvar HSV: %s' % error)
            if key == ord('q'):
                rclpy.try_shutdown()

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.p('show_windows'):
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VirtualWall()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
