import math
from typing import List, Sequence, Tuple

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from robo_lidar.scan_utils import conservative_clearing_upsample


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanFilterNode(Node):
    """Rotate, mirror, crop and mask a LaserScan without touching firmware."""

    def __init__(self) -> None:
        super().__init__("scan_filter")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("front_topic", "/scan_front")
        self.declare_parameter("clearing_topic", "/scan_front_clear")
        self.declare_parameter("clearing_upsample_factor", 4)
        self.declare_parameter("front_offset_deg", 270.0)
        self.declare_parameter("invert_angles", False)
        self.declare_parameter("front_half_angle_deg", 90.0)
        self.declare_parameter("publish_full_filtered", True)
        self.declare_parameter("self_filter_enabled", True)
        self.declare_parameter("lidar_x_m", 0.2765)
        self.declare_parameter("lidar_y_m", 0.0)
        self.declare_parameter("robot_min_x_m", -0.29)
        self.declare_parameter("robot_max_x_m", 0.29)
        self.declare_parameter("robot_min_y_m", -0.30)
        self.declare_parameter("robot_max_y_m", 0.30)
        # No ROS 2, uma lista vazia precisa de tipo explicito.
        self.declare_parameter(
            "mask_intervals_deg", Parameter.Type.DOUBLE_ARRAY
        )

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        front_topic = str(self.get_parameter("front_topic").value)
        clearing_topic = str(self.get_parameter("clearing_topic").value)

        published_topics = (output_topic, front_topic, clearing_topic)
        if input_topic in published_topics:
            raise ValueError(
                "input_topic precisa ser diferente dos topicos publicados"
            )
        if len(set(published_topics)) != len(published_topics):
            raise ValueError("os topicos publicados precisam ser diferentes")

        self.full_pub = self.create_publisher(LaserScan, output_topic, 10)
        self.front_pub = self.create_publisher(LaserScan, front_topic, 10)
        self.clearing_pub = self.create_publisher(
            LaserScan, clearing_topic, 10
        )
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self._on_scan, 10
        )

        self.get_logger().info(
            "LiDAR: "
            f"{input_topic} -> {output_topic}, {front_topic} e "
            f"{clearing_topic}"
        )

    def _mask_intervals(self) -> List[Tuple[float, float]]:
        try:
            raw = list(self.get_parameter("mask_intervals_deg").value)
        except ParameterUninitializedException:
            return []
        if len(raw) % 2 != 0:
            self.get_logger().warning(
                "mask_intervals_deg deve conter pares; ultimo valor ignorado"
            )
            raw = raw[:-1]

        return [
            (math.radians(float(raw[i])), math.radians(float(raw[i + 1])))
            for i in range(0, len(raw), 2)
        ]

    @staticmethod
    def _inside_interval(
        angle: float, interval: Tuple[float, float]
    ) -> bool:
        start = normalize_angle(interval[0])
        end = normalize_angle(interval[1])
        angle = normalize_angle(angle)
        if start <= end:
            return start <= angle <= end
        return angle >= start or angle <= end

    @staticmethod
    def _empty_like(msg: LaserScan) -> LaserScan:
        out = LaserScan()
        # O bridge marca o instante de recepcao do pacote. Nao carimbamos de
        # novo aqui, pois isso esconderia atrasos reais entre os dois nos.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        # NaN significa "raio ignorado". +inf fica reservado aos raios
        # realmente observados sem retorno, permitindo clearing no Nav2.
        out.ranges = [math.nan] * len(msg.ranges)
        out.intensities = [0.0] * len(msg.ranges)
        return out

    @staticmethod
    def _usable_range(value: float, msg: LaserScan) -> bool:
        if math.isinf(value):
            return value > 0.0
        return math.isfinite(value) and msg.range_min <= value <= msg.range_max

    @staticmethod
    def _store_range(scan: LaserScan, index: int, value: float) -> None:
        previous = scan.ranges[index]
        if math.isnan(previous) or value < previous:
            scan.ranges[index] = value

    @staticmethod
    def _dense_clearing_scan(
        scan: LaserScan, upsample_factor: int
    ) -> LaserScan:
        """Expand only angular sectors proven free by adjacent beams."""
        if upsample_factor < 1:
            raise ValueError("clearing_upsample_factor precisa ser >= 1")

        dense = LaserScan()
        dense.header.stamp = scan.header.stamp
        dense.header.frame_id = scan.header.frame_id
        dense.angle_min = scan.angle_min
        dense.scan_time = scan.scan_time
        dense.range_min = scan.range_min
        dense.range_max = scan.range_max

        count = len(scan.ranges)
        if count <= 1 or upsample_factor == 1:
            dense.angle_max = scan.angle_max
            dense.angle_increment = scan.angle_increment
            dense.time_increment = scan.time_increment
            dense.ranges = list(scan.ranges)
            dense.intensities = list(scan.intensities)
            return dense

        dense_count = (count - 1) * upsample_factor + 1
        dense.angle_increment = scan.angle_increment / upsample_factor
        dense.angle_max = (
            dense.angle_min
            + (dense_count - 1) * dense.angle_increment
        )
        dense.time_increment = scan.time_increment / upsample_factor

        dense.ranges, dense.intensities = conservative_clearing_upsample(
            scan.ranges,
            scan.intensities,
            upsample_factor,
        )

        return dense

    def _on_scan(self, msg: LaserScan) -> None:
        if not msg.ranges or msg.angle_increment == 0.0:
            return

        full = self._empty_like(msg)
        front = self._empty_like(msg)
        offset = math.radians(
            float(self.get_parameter("front_offset_deg").value)
        )
        invert = bool(self.get_parameter("invert_angles").value)
        half_angle = math.radians(
            float(self.get_parameter("front_half_angle_deg").value)
        )
        publish_full = bool(
            self.get_parameter("publish_full_filtered").value
        )
        clearing_upsample_factor = int(
            self.get_parameter("clearing_upsample_factor").value
        )
        masks = self._mask_intervals()
        self_filter_enabled = bool(
            self.get_parameter("self_filter_enabled").value
        )

        lidar_x = float(self.get_parameter("lidar_x_m").value)
        lidar_y = float(self.get_parameter("lidar_y_m").value)

        robot_min_x = float(self.get_parameter("robot_min_x_m").value)
        robot_max_x = float(self.get_parameter("robot_max_x_m").value)
        robot_min_y = float(self.get_parameter("robot_min_y_m").value)
        robot_max_y = float(self.get_parameter("robot_max_y_m").value)

        count = len(msg.ranges)
        for source_index, distance in enumerate(msg.ranges):
            sensor_angle = msg.angle_min + source_index * msg.angle_increment
            robot_angle = normalize_angle(sensor_angle - offset)
            if invert:
                robot_angle = normalize_angle(-robot_angle)

            if any(
                self._inside_interval(robot_angle, interval)
                for interval in masks
            ):
                continue

            if not self._usable_range(distance, msg):
                continue
            if self_filter_enabled and math.isfinite(distance):
                point_x = lidar_x + distance * math.cos(robot_angle)
                point_y = lidar_y + distance * math.sin(robot_angle)

                inside_robot = (
                    robot_min_x <= point_x <= robot_max_x
                    and robot_min_y <= point_y <= robot_max_y
                )

                if inside_robot:
                    continue  
            target_float = (robot_angle - msg.angle_min) / msg.angle_increment
            target_index = int(round(target_float)) % count
            in_front = abs(robot_angle) <= half_angle + 1e-9

            if publish_full:
                self._store_range(full, target_index, distance)

            if in_front:
                self._store_range(front, target_index, distance)

            if msg.intensities and source_index < len(msg.intensities):
                intensity = msg.intensities[source_index]
                if publish_full:
                    full.intensities[target_index] = intensity
                if in_front:
                    front.intensities[target_index] = intensity

        if publish_full:
            self.full_pub.publish(full)
        self.front_pub.publish(front)
        self.clearing_pub.publish(
            self._dense_clearing_scan(front, clearing_upsample_factor)
        )


def main(args: Sequence[str] = None) -> None:
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
