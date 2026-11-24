"""Odometria local medida por scan matching 2D, sem encoders."""

import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .scan_matching import Pose2D, compose, match_scans, relative, transform_points, wrap_angle


class LidarOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("odometry_node")

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan_front")
        self.declare_parameter("valid_topic", "/lidar_odometry/valid")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("scan_timeout_s", 0.70)
        self.declare_parameter("match_timeout_s", 0.85)
        self.declare_parameter("reference_reset_timeout_s", 1.20)
        self.declare_parameter("minimum_scan_points", 24)
        self.declare_parameter("maximum_correspondence_distance_m", 0.30)
        self.declare_parameter("trim_fraction", 0.75)
        self.declare_parameter("maximum_icp_iterations", 12)
        self.declare_parameter("search_linear_window_m", 0.06)
        self.declare_parameter("search_linear_step_m", 0.02)
        self.declare_parameter("search_angular_window_rad", 0.06)
        self.declare_parameter("search_angular_step_rad", 0.02)
        self.declare_parameter("maximum_match_rmse_m", 0.085)
        self.declare_parameter("maximum_scan_translation_m", 0.20)
        self.declare_parameter("maximum_scan_rotation_rad", 0.25)
        self.declare_parameter("stationary_translation_deadband_m", 0.001)
        self.declare_parameter("stationary_rotation_deadband_rad", 0.003)
        self.declare_parameter("keyframe_translation_m", 0.030)
        self.declare_parameter("keyframe_rotation_rad", 0.030)
        self.declare_parameter("velocity_filter_alpha", 0.55)
        # IMU (BNO055 via robo_bridge): o giro medido pelo giroscopio entre
        # dois scans vira o ponto de partida da busca de rotacao. Sem /imu
        # recente, volta para a previsao pela velocidade dos scans.
        self.declare_parameter("use_imu", True)
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("imu_timeout_s", 0.30)

        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.valid_topic = str(self.get_parameter("valid_topic").value)
        self.publish_rate = self._positive("publish_rate_hz")
        self.scan_timeout = self._positive("scan_timeout_s")
        self.match_timeout = self._positive("match_timeout_s")
        self.reference_reset_timeout = self._positive("reference_reset_timeout_s")
        self.minimum_scan_points = int(self.get_parameter("minimum_scan_points").value)
        self.maximum_correspondence_distance = self._positive(
            "maximum_correspondence_distance_m"
        )
        self.trim_fraction = float(self.get_parameter("trim_fraction").value)
        self.maximum_icp_iterations = int(
            self.get_parameter("maximum_icp_iterations").value
        )
        self.search_linear_window = self._nonnegative("search_linear_window_m")
        self.search_linear_step = self._positive("search_linear_step_m")
        self.search_angular_window = self._nonnegative(
            "search_angular_window_rad"
        )
        self.search_angular_step = self._positive("search_angular_step_rad")
        self.maximum_match_rmse = self._positive("maximum_match_rmse_m")
        self.maximum_scan_translation = self._positive(
            "maximum_scan_translation_m"
        )
        self.maximum_scan_rotation = self._positive("maximum_scan_rotation_rad")
        self.stationary_translation_deadband = self._nonnegative(
            "stationary_translation_deadband_m"
        )
        self.stationary_rotation_deadband = self._nonnegative(
            "stationary_rotation_deadband_rad"
        )
        self.keyframe_translation = self._positive("keyframe_translation_m")
        self.keyframe_rotation = self._positive("keyframe_rotation_rad")
        self.velocity_alpha = float(self.get_parameter("velocity_filter_alpha").value)
        self.use_imu = bool(self.get_parameter("use_imu").value)
        self.imu_timeout = self._positive("imu_timeout_s")
        if self.minimum_scan_points < 8:
            raise ValueError("minimum_scan_points deve ser pelo menos 8")
        if not 0.40 <= self.trim_fraction <= 1.0:
            raise ValueError("trim_fraction deve ficar entre 0.40 e 1.0")
        if self.maximum_icp_iterations < 1:
            raise ValueError("maximum_icp_iterations deve ser positivo")
        if not 0.0 < self.velocity_alpha <= 1.0:
            raise ValueError("velocity_filter_alpha deve ficar em (0, 1]")

        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.velocity = Pose2D(0.0, 0.0, 0.0)
        self.last_scan_wall: Optional[float] = None
        self.last_match_wall: Optional[float] = None
        self.last_update_time = self.get_clock().now()

        self.reference_points: Optional[np.ndarray] = None
        self.reference_pose: Optional[Pose2D] = None
        self.reference_wall: Optional[float] = None
        self.locked = False
        self.mode = "AGUARDANDO_SCANS"
        self.scan_messages = 0
        self.accepted_matches = 0
        self.rejected_matches = 0
        self.reference_resets = 0
        self.last_rmse = math.inf
        self.last_correspondences = 0

        # Giro integrado do giroscopio (rad, referencia arbitraria) e o valor
        # dele no ultimo match aceito.
        self.imu_yaw_integrated = 0.0
        self.imu_last_wall: Optional[float] = None
        self.imu_yaw_at_match: Optional[float] = None
        self.imu_active = False
        self.imu_predictions = 0
        self.imu_rate = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self.valid_publisher = self.create_publisher(Bool, self.valid_topic, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data
        )
        if self.use_imu:
            self.create_subscription(
                Imu,
                str(self.get_parameter("imu_topic").value),
                self.on_imu,
                qos_profile_sensor_data,
            )
        self.create_service(Trigger, "~/guard_status", self.on_status)
        self.create_service(Trigger, "~/localization_status", self.on_status)
        self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            "Odometria LIDAR-ONLY iniciada: scans consecutivos medem "
            "odom -> base_link; encoders, EKF e integracao de /cmd_vel NAO sao usados."
        )

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} deve ser positivo e finito")
        return value

    def _nonnegative(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} deve ser nao negativo e finito")
        return value

    def _scan_points_in_base(self, message: LaserScan) -> Optional[np.ndarray]:
        angles = float(message.angle_min) + np.arange(len(message.ranges)) * float(
            message.angle_increment
        )
        ranges = np.asarray(message.ranges, dtype=float)
        valid = np.isfinite(ranges)
        valid &= ranges >= max(float(message.range_min), 0.08)
        valid &= ranges <= min(float(message.range_max), 6.0)

        valid_count = int(np.count_nonzero(valid))

        if valid_count < self.minimum_scan_points:
            self.get_logger().warning(
                f"SCAN INVALIDO: poucos pontos validos "
                f"({valid_count}/{len(ranges)}), minimo={self.minimum_scan_points}"
            )
            return None
        points = np.column_stack(
            (ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]))
        )
        frame = message.header.frame_id or self.base_frame
        if frame == self.base_frame:
            return points
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"SCAN INVALIDO: falha TF "
                f"{frame} -> {self.base_frame}: {exc}"
            )
            return None
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        extrinsic = Pose2D(
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw,
        )
        return transform_points(points, extrinsic)

    def on_imu(self, message: Imu) -> None:
        now = time.monotonic()
        if self.imu_last_wall is not None:
            dt = now - self.imu_last_wall
            # Buraco grande no /imu: nao integra o intervalo (o giro desse
            # trecho seria desconhecido) e deixa a previsao cair para a
            # velocidade ate o proximo match.
            if dt <= self.imu_timeout:
                self.imu_yaw_integrated += float(message.angular_velocity.z) * dt
            else:
                self.imu_yaw_at_match = None
        self.imu_rate = float(message.angular_velocity.z)
        self.imu_last_wall = now
        if not self.imu_active:
            self.imu_active = True
            self.get_logger().info("IMU detectada: previsao de giro pelo giroscopio")

    def _imu_fresh(self, now: float) -> bool:
        return (
            self.use_imu
            and self.imu_last_wall is not None
            and now - self.imu_last_wall <= self.imu_timeout
        )

    def on_scan(self, message: LaserScan) -> None:
        now = time.monotonic()
        self.last_scan_wall = now
        self.scan_messages += 1
        points = self._scan_points_in_base(message)
        if points is None:
            self.rejected_matches += 1
            self.mode = "SCAN_INVALIDO"
            return

        if self.reference_points is None or self.reference_pose is None:
            self._reset_reference(points, now, count_reset=False)
            self.mode = "REFERENCIA_INICIAL"
            return

        # Previsao por velocidade constante. Girando no lugar a ~0,56 rad/s o
        # robo gira ~0,10 rad entre scans, no limite da janela angular da
        # busca; partindo da pose anterior, giros maiores eram medidos a menos
        # (ex.: 0,15 real -> 0,105 medido), o Nav2 mandava girar mais e o
        # robo ficava rodando no lugar enquanto o SLAM borrava o mapa.
        predicted = self.pose
        if self.locked and self.last_match_wall is not None:
            dt_pred = min(now - self.last_match_wall, 0.5)
            yaw_pred = self.velocity.yaw * dt_pred
            # Com IMU: giro medido desde o ultimo match (inclui inicio e fim
            # de giro, onde a velocidade filtrada atrasa).
            if self._imu_fresh(now) and self.imu_yaw_at_match is not None:
                yaw_pred = self.imu_yaw_integrated - self.imu_yaw_at_match
                self.imu_predictions += 1
            elif self.imu_active and not self._imu_fresh(now):
                self.imu_active = False
                self.get_logger().warn("IMU sem dados: previsao de giro pela velocidade")
            predicted = compose(
                self.pose,
                Pose2D(
                    self.velocity.x * dt_pred,
                    self.velocity.y * dt_pred,
                    yaw_pred,
                ),
            )
        initial = relative(self.reference_pose, predicted)
        match_start = time.monotonic()
        result = match_scans(
            self.reference_points,
            points,
            initial,
            maximum_distance=self.maximum_correspondence_distance,
            trim_fraction=self.trim_fraction,
            minimum_count=self.minimum_scan_points,
            maximum_iterations=self.maximum_icp_iterations,
            linear_window=self.search_linear_window,
            linear_step=self.search_linear_step,
            angular_window=self.search_angular_window,
            angular_step=self.search_angular_step,
        )
        match_elapsed = time.monotonic() - match_start

        self.get_logger().info(
            f"TEMPO MATCH_SCANS: {match_elapsed * 1000.0:.1f} ms"
        )
        acceptable = result is not None
        if result is not None:
            motion = math.hypot(result.pose.x, result.pose.y)
            acceptable = (
                result.rmse <= self.maximum_match_rmse
                and result.correspondences >= self.minimum_scan_points
                and motion <= self.maximum_scan_translation
                and abs(result.pose.yaw) <= self.maximum_scan_rotation
            )

        if not acceptable or result is None:
            self.rejected_matches += 1
            self.mode = "MATCH_REJEITADO"
            if (
                self.reference_wall is not None
                and now - self.reference_wall >= self.reference_reset_timeout
            ):
                self._reset_reference(points, now, count_reset=True)
                self.locked = False
            return

        delta = result.pose
        translation = math.hypot(delta.x, delta.y)
        if (
            translation <= self.stationary_translation_deadband
            and abs(delta.yaw) <= self.stationary_rotation_deadband
        ):
            delta = Pose2D(0.0, 0.0, 0.0)

        corrected_pose = compose(self.reference_pose, delta)
        print(
            "DEBUG ACUM | "
            f"ref=({self.reference_pose.x:+.4f}, "
            f"{self.reference_pose.y:+.4f}, "
            f"{self.reference_pose.yaw:+.4f}) | "
            f"delta=({delta.x:+.4f}, "
            f"{delta.y:+.4f}, "
            f"{delta.yaw:+.4f}) | "
            f"corrigida=({corrected_pose.x:+.4f}, "
            f"{corrected_pose.y:+.4f}, "
            f"{corrected_pose.yaw:+.4f})",
            flush=True,
        )
        dt = max(0.02, now - (self.reference_wall or now))
        measured_velocity = Pose2D(
            delta.x / dt,
            delta.y / dt,
            wrap_angle(delta.yaw) / dt,
        )
        alpha = self.velocity_alpha
        self.velocity = Pose2D(
            alpha * measured_velocity.x + (1.0 - alpha) * self.velocity.x,
            alpha * measured_velocity.y + (1.0 - alpha) * self.velocity.y,
            alpha * measured_velocity.yaw + (1.0 - alpha) * self.velocity.yaw,
        )
        self.pose = corrected_pose
        # V2: nao troca a referencia a cada scan. Movimentos de poucos mm
        # acumulam contra o mesmo keyframe ate ficarem claramente observaveis.
        # Isso evita perder deslocamento real na zona morta/ruido do LiDAR.
        if (
            translation >= self.keyframe_translation
            or abs(delta.yaw) >= self.keyframe_rotation
        ):
            print(
                "DEBUG KEYFRAME | "
                f"ref_antiga=({self.reference_pose.x:+.4f}, "
                f"{self.reference_pose.y:+.4f}, "
                f"{self.reference_pose.yaw:+.4f}) | "
                f"delta=({delta.x:+.4f}, "
                f"{delta.y:+.4f}, "
                f"{delta.yaw:+.4f}) | "
                f"nova_ref=({corrected_pose.x:+.4f}, "
                f"{corrected_pose.y:+.4f}, "
                f"{corrected_pose.yaw:+.4f}) | "
                f"delta_norm={translation:.4f}",
                flush=True,
            )
            self.reference_points = points
            self.reference_pose = corrected_pose
            self.reference_wall = now
            self.mode = "LIDAR_LOCK_KEYFRAME"
        else:
            self.mode = "LIDAR_LOCK_ACUMULANDO"
        self.last_match_wall = now
        self.imu_yaw_at_match = self.imu_yaw_integrated if self._imu_fresh(now) else None
        self.locked = True
        self.accepted_matches += 1
        self.last_rmse = result.rmse
        self.last_correspondences = result.correspondences
        
    def _reset_reference(
        self, points: np.ndarray, now: float, *, count_reset: bool
    ) -> None:
        self.reference_points = points
        self.reference_pose = self.pose
        self.reference_wall = now
        if count_reset:
            self.reference_resets += 1

    def on_timer(self) -> None:
        ros_now = self.get_clock().now()
        self.last_update_time = ros_now
        wall_now = time.monotonic()
        valid = self._is_valid(wall_now)
        if not valid:
            self.velocity = Pose2D(0.0, 0.0, 0.0)
        self.valid_publisher.publish(Bool(data=valid))
        self._publish_odometry(ros_now, valid)

    def _is_valid(self, now: float) -> bool:
        return (
            self.locked
            and self.last_scan_wall is not None
            and now - self.last_scan_wall <= self.scan_timeout
            and self.last_match_wall is not None
            and now - self.last_match_wall <= self.match_timeout
        )

    def _publish_odometry(self, now, valid: bool) -> None:
        # Entre dois scans (~185 ms) a pose do LiDAR fica parada. Com IMU, o
        # yaw publicado soma o giro medido desde o ultimo match: o controlador
        # ve o giro com ~50 ms de atraso em vez de ~200 ms e para o giro no
        # lugar mais cedo (menos excesso de giro). No proximo match a pose
        # do LiDAR assume de novo (a diferenca e de ~1 grau).
        yaw = self.pose.yaw
        twist = self.velocity if valid else Pose2D(0.0, 0.0, 0.0)
        wall_now = time.monotonic()
        if self._imu_fresh(wall_now) and self.imu_yaw_at_match is not None:
            yaw = wrap_angle(yaw + self.imu_yaw_integrated - self.imu_yaw_at_match)
            if valid:
                twist = Pose2D(twist.x, twist.y, self.imu_rate)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.pose.x
        message.pose.pose.position.y = self.pose.y
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = twist.x
        message.twist.twist.linear.y = twist.y
        message.twist.twist.angular.z = twist.yaw
        xy_variance = 0.025 if valid else 1.0
        yaw_variance = 0.04 if valid else 1.5
        message.pose.covariance[0] = xy_variance
        message.pose.covariance[7] = xy_variance
        message.pose.covariance[35] = yaw_variance
        message.twist.covariance[0] = xy_variance
        message.twist.covariance[7] = xy_variance
        message.twist.covariance[35] = yaw_variance
        self.odom_publisher.publish(message)
        if self.publish_tf:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.pose.x
            transform.transform.translation.y = self.pose.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def on_status(self, _request, response):
        now = time.monotonic()
        scan_age = math.inf if self.last_scan_wall is None else now - self.last_scan_wall
        match_age = math.inf if self.last_match_wall is None else now - self.last_match_wall
        rmse = "-" if not math.isfinite(self.last_rmse) else f"{self.last_rmse:.4f}"
        valid = self._is_valid(now)
        response.success = valid
        response.message = (
            f"modo={self.mode}; fonte_local=LIDAR_ONLY_SCAN_MATCHING; "
            f"encoders_usados=NAO; cmd_vel_integrado=NAO; ekf_usado=NAO; valido={valid}; scans={self.scan_messages}; "
            f"matches_aceitos={self.accepted_matches}; "
            f"matches_rejeitados={self.rejected_matches}; resets={self.reference_resets}; "
            f"rmse={rmse}m; correspondencias={self.last_correspondences}; "
            f"idade_scan={scan_age:.3f}s; idade_match={match_age:.3f}s; "
            f"pose=[{self.pose.x:.3f},{self.pose.y:.3f},{self.pose.yaw:.3f}]; "
            f"keyframe_trans={self.keyframe_translation:.3f}m; deadband_trans={self.stationary_translation_deadband:.3f}m; "
            f"imu_previsao={'SIM' if self._imu_fresh(now) else 'NAO'}; previsoes_com_imu={self.imu_predictions}"
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarOdometryNode()
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
