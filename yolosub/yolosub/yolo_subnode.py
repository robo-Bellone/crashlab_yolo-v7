'''
### 모든거 다 받는거

import rclpy
from rclpy.node import Node
from darknet_ros_msgs.msg import BoundingBoxes

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription = self.create_subscription(
            BoundingBoxes,
            '/darknet_ros/bounding_boxes',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received bounding boxes:')
        for box in msg.bounding_boxes:
            self.get_logger().info(
                f'Class: {box.class_id}, ID: {box.id}, Probability: {box.probability}, '
                f'xmin: {box.xmin}, ymin: {box.ymin}, xmax: {box.xmax}, ymax: {box.ymax}'
            )

def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber = YoloSubscriber()
    rclpy.spin(yolo_subscriber)
    yolo_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    '''

import rclpy
from rclpy.node import Node
from darknet_ros_msgs.msg import BoundingBoxes

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription = self.create_subscription(
            BoundingBoxes,
            '/darknet_ros/bounding_boxes',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        found_id_0 = False  # id 0을 찾았는지 여부를 추적하는 변수

        for box in msg.bounding_boxes:
            if box.id == 0:
                self.get_logger().info(f'Class: {box.class_id}')
                self.get_logger().info(f'ID: {box.id}')
                self.get_logger().info(f'Probability: {box.probability:.4f}')
                self.get_logger().info(f'xmin: {box.xmin}')
                self.get_logger().info(f'xmax: {box.xmax}')
                self.get_logger().info(f'ymin: {box.ymin}')
                self.get_logger().info(f'ymax: {box.ymax}')
                self.get_logger().info('---------------------------')
                found_id_0 = True

        if not found_id_0:
            self.get_logger().info('NO person')

def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber = YoloSubscriber()
    rclpy.spin(yolo_subscriber)
    yolo_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


