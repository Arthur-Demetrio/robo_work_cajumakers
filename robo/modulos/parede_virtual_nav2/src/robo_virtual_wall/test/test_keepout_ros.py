"""ROS adapter exercise without navigation, camera, or hardware commands."""
import json
import pytest
rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from robo_virtual_wall.keepout_node import TapeKeepout


def test_adapter_confirmation_stamp_tf_mask_and_clear():
    rclpy.init(domain_id=137)
    node = TapeKeepout()
    try:
        published = []
        infos = []
        node.mask_pub.publish = published.append
        node.info_pub.publish = infos.append
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 100
        grid.info.resolution = .05
        grid.info.origin.orientation.w = 1.
        node.on_map(grid)
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        transform.transform.translation.x = 1.
        times = []
        def lookup(target,source,stamp):
            assert (target,source) == ('map','base_link')
            times.append(stamp.nanoseconds)
            return transform
        node.tf_buffer.lookup_transform = lookup
        def message(metric_frame='base_link',age=0.):
            now = node.get_clock().now().nanoseconds-int(age*1e9)
            return String(data=json.dumps(dict(metric_frame=metric_frame,
                stamp=dict(sec=now//1000000000,nanosec=now%1000000000),
                detections=[dict(base_link_m=[[.5,.1],[.5,.4]])])))
        node.on_detections(message(metric_frame=''))
        assert not node.tracker.pending
        first = message()
        node.on_detections(first)
        node.on_detections(first)  # duplicate timestamp cannot confirm a wall
        assert not node.tracker.walls
        node.on_detections(message(age=2.))
        assert not node.tracker.pending
        for _ in range(3):
            node.on_detections(message())
        assert len(node.tracker.walls) == 1
        assert node.tracker.walls[0][0,0] == pytest.approx(1.5)
        node.publish()
        assert 100 in published[-1].data
        assert published[-1].header.frame_id == 'map'
        assert infos[-1].type == 0 and infos[-1].multiplier == 1.
        assert times
        node.clear(Empty.Request(),Empty.Response())
        assert not any(published[-1].data)
        # A future-TF miss must be retried without consuming its timestamp.
        from tf2_ros import ExtrapolationException
        def not_ready(*args):
            raise ExtrapolationException('future')
        node.tf_buffer.lookup_transform = not_ready
        pending = message()
        node.on_detections(pending)
        assert len(node.waiting_tf) == 1
        assert not node.tracker.pending
        node.tf_buffer.lookup_transform = lookup
        node.retry_waiting_tf()
        assert not node.waiting_tf
        assert len(node.tracker.pending) == 1
        # Once a queued image expires, discard it instead of using latest TF.
        node.waiting_tf.append(message(age=2.))
        node.retry_waiting_tf()
        assert not node.waiting_tf
        assert not node.tracker.pending
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_high_resolution_preview_distance_remains_observation_only(tmp_path, monkeypatch):
    from pathlib import Path
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
    from robo_virtual_wall.node import VirtualWall
    preview = Path(__file__).parents[1]/'calibration/front_preview_20261106_repositioned.json'
    rclpy.init(domain_id=137,args=['--ros-args',
        '-p','image_topic:=/test/image',
        '-p','preview_homography_file:='+str(preview),
        '-p','preview_allow_resize:=true',
        '-p','roi_top:=0.0'])
    node = VirtualWall()
    try:
        statuses,images = [],[]
        node.status_pub.publish = statuses.append
        node.annotated.publish = images.append
        image = np.full((480,640,3),255,np.uint8)
        for x in range(100,580,25):
            cv2.rectangle(image,(x,300),(x+12,312),(0,0,255),-1)
        frame = cv2.resize(image,(1280,960),interpolation=cv2.INTER_NEAREST)
        header = Image().header
        header.stamp = node.get_clock().now().to_msg()
        node.process(frame,header)
        assert node.preview_filter.values
        assert images[-1].width == 1280 and images[-1].height == 960
        status = json.loads(statuses[-1].data)
        assert status['metric_frame'] == ''
        assert not any('base_link_m' in d for d in status['detections'])
        # Exercise the same callback used by the GUI without opening a window.
        x,y = np.rint(node.probe_calibration[2].mean(axis=0)).astype(int)
        node.on_camera_click(cv2.EVENT_LBUTTONDOWN,int(x),int(y),0,None)
        assert node.probe_selection and not node.probe_selection['fitting_reference']
        forward,lateral = node.probe_selection['predicted_m']
        node.probe_input = '%.3f %.3f' % (100*forward,100*lateral)
        monkeypatch.setattr(Path,'home',classmethod(lambda cls: tmp_path))
        node.save_probe_check()
        saved = json.loads(node.probe_checks_path.read_text())
        assert saved['image_size'] == [1280,960]
        assert len(saved['pixels']) == 1
        assert saved['validated'] is False
        assert saved['checks'][0]['error_m'] < .001
        node.on_camera_click(cv2.EVENT_RBUTTONDOWN,0,0,0,None)
        assert node.probe_selection is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_real_calibration_detector_to_keepout_mask():
    from pathlib import Path
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
    from robo_virtual_wall.node import VirtualWall
    calibration = Path(__file__).parents[1]/'calibration/front_base_link_validated.json'
    rclpy.init(domain_id=137,args=['--ros-args','-p','image_topic:=/test/image',
        '-p','homography_file:='+str(calibration),'-p','roi_top:=0.0'])
    vision = VirtualWall()
    mask = TapeKeepout()
    try:
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 100
        grid.info.resolution = .05
        grid.info.origin.position.x = grid.info.origin.position.y = -2.
        grid.info.origin.orientation.w = 1.
        mask.on_map(grid)
        tf = TransformStamped()
        tf.transform.rotation.w = 1.
        tf.transform.translation.x = 1.
        mask.tf_buffer.lookup_transform = lambda *args: tf
        outputs = []
        mask.mask_pub.publish = outputs.append
        statuses = []
        def on_status(message):
            statuses.append(json.loads(message.data))
            mask.on_detections(message)
        vision.status_pub.publish = on_status
        frame = np.full((480,640,3),255,np.uint8)
        for x in range(100,580,25):
            cv2.rectangle(frame,(x,300),(x+12,312),(0,0,255),-1)
        for _ in range(3):
            header = Image().header
            header.stamp = vision.get_clock().now().to_msg()
            vision.process(frame,header)
        assert statuses[-1]['metric_frame'] == 'base_link'
        assert statuses[-1]['detections'][0]['base_link_m']
        assert mask.tracker.walls
        assert np.mean(mask.tracker.walls[0][:,0]) == pytest.approx(1.71,abs=.03)
        mask.publish()
        assert 100 in outputs[-1].data
        # Observed tape endpoints outside the blue polygon were clipped, not dropped.
        assert np.ptp(mask.tracker.walls[0][:,1]) > .2
    finally:
        vision.destroy_node()
        mask.destroy_node()
        rclpy.shutdown()
