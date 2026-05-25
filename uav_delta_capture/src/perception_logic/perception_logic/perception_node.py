from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class PerceptionLogicNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.publish_topic = self.declare_parameter('publish_topic', 'vision/target_offset').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_optical_frame').value

        self.offset_pub = self.create_publisher(PointStamped, self.publish_topic, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self._offset_msg = PointStamped()

        self.get_logger().info(
            f'Perception node started: image_topic={self.image_topic}, publish_topic={self.publish_topic}'
        )

    def infer_npu(self, image: Image) -> Optional[Tuple[float, float, float]]:
        """预留 NPU 推理接口（OpenVINO/ONNX Runtime）。

        Returns:
            (u_px, v_px, score) 或 None
        """
        # TODO: STM32MP257F NPU 推理逻辑接入点
        if image.width == 0 or image.height == 0:
            return None
        return float(image.width) * 0.5, float(image.height) * 0.5, 1.0

    def image_callback(self, msg: Image) -> None:
        result = self.infer_npu(msg)
        if result is None:
            self.get_logger().debug('infer_npu returned None.')
            return

        u_px, v_px, score = result
        if score < 0.1:
            self.get_logger().debug('Low confidence target skipped.')
            return

        cx = float(msg.width) * 0.5
        cy = float(msg.height) * 0.5

        self._offset_msg.header.stamp = msg.header.stamp
        self._offset_msg.header.frame_id = msg.header.frame_id or self.camera_frame
        self._offset_msg.point.x = u_px - cx
        self._offset_msg.point.y = v_px - cy
        self._offset_msg.point.z = 0.0

        self.offset_pub.publish(self._offset_msg)
        self.get_logger().debug(
            f'Published target offset px: ({self._offset_msg.point.x:.2f}, {self._offset_msg.point.y:.2f})'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
