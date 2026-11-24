#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    """Normaliza um angulo para o intervalo [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def parse_intervals(text: str) -> List[Tuple[float, float]]:
    """Converte 'inicio:fim,inicio:fim' de graus para radianos."""
    intervals: List[Tuple[float, float]] = []
    if not text.strip():
        return intervals

    for item in text.split(","):
        parts = item.strip().split(":")
        if len(parts) != 2:
            raise ValueError(
                "blocked_intervals_deg deve usar o formato inicio:fim,inicio:fim"
            )
        start_deg = float(parts[0])
        end_deg = float(parts[1])
        intervals.append(
            (
                normalize_angle(math.radians(start_deg)),
                normalize_angle(math.radians(end_deg)),
            )
        )
    return intervals


def angle_is_blocked(angle: float, intervals: List[Tuple[float, float]]) -> bool:
    angle = normalize_angle(angle)
    for start, end in intervals:
        if start <= end:
            if start <= angle <= end:
                return True
        elif angle >= start or angle <= end:
            return True
    return False


class ScanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_filter")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan_front")
        self.declare_parameter("invert_angles", True)
        self.declare_parameter("blocked_intervals_deg", "")
        self.declare_parameter("force_out_of_range_to_inf", True)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.invert_angles = bool(self.get_parameter("invert_angles").value)
        blocked_text = str(self.get_parameter("blocked_intervals_deg").value)
        self.force_out_of_range = bool(
            self.get_parameter("force_out_of_range_to_inf").value
        )

        try:
            self.blocked_intervals = parse_intervals(blocked_text)
        except ValueError as error:
            self.get_logger().fatal(str(error))
            raise

        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Filtro LiDAR pronto: {input_topic} -> {output_topic}; "
            f"inversao angular={'ativa' if self.invert_angles else 'inativa'}; "
            f"intervalos bloqueados={len(self.blocked_intervals)}"
        )

    def scan_callback(self, source: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = source.header
        filtered.time_increment = source.time_increment
        filtered.scan_time = source.scan_time
        filtered.range_min = source.range_min
        filtered.range_max = source.range_max

        if self.invert_angles:
            # Correcao lateral validada em 21/09, conforme o codigo
            # utilizado antes dos testes de quina de 30/09.
            filtered.angle_min = -source.angle_max
            filtered.angle_max = -source.angle_min
            filtered.angle_increment = abs(source.angle_increment)
            filtered.ranges = list(reversed(source.ranges))
            filtered.intensities = list(reversed(source.intensities))
        else:
            filtered.angle_min = source.angle_min
            filtered.angle_max = source.angle_max
            filtered.angle_increment = source.angle_increment
            filtered.ranges = list(source.ranges)
            filtered.intensities = list(source.intensities)

        for index, distance in enumerate(filtered.ranges):
            angle = filtered.angle_min + index * filtered.angle_increment
            invalid_range = (
                self.force_out_of_range
                and math.isfinite(distance)
                and (distance < filtered.range_min or distance > filtered.range_max)
            )
            if invalid_range or angle_is_blocked(angle, self.blocked_intervals):
                filtered.ranges[index] = math.inf
                if index < len(filtered.intensities):
                    filtered.intensities[index] = 0.0

        self.publisher.publish(filtered)


def main(args=None) -> None:
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
