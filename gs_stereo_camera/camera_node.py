import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters for namespace and device
        self.declare_parameter('device', 0)  # Default to device 0
        self.declare_parameter('namespace', '')  # Empty string for no namespace

        # Get the parameters
        self.device = self.get_parameter('device').get_parameter_value().integer_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # Log the parameters for debugging
        self.get_logger().info(f'Using device {self.device}')
        self.get_logger().info(f'Namespace: {self.namespace}')

        # Open the video capture device
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.device}')
            return

        # Initialize CvBridge to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

        # Set up publishers for image and camera info
        self.image_pub = self.create_publisher(Image, f'{self.namespace}/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, f'{self.namespace}/camera_info', 10)

        # Set a timer to capture frames periodically
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize CameraInfo (can be loaded from a calibration file in the future)
        self.camera_info = self.create_camera_info()

    def create_camera_info(self):
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_frame"
        camera_info.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        camera_info.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # Set intrinsic camera parameters (these should ideally come from actual calibration)
        camera_info.k = [640.0, 0.0, 320.0,
                        0.0, 480.0, 240.0,
                        0.0, 0.0, 1.0]
        # Set the projection matrix P (12 elements)
        camera_info.p = [640.0, 0.0, 320.0, 0.0,
                        0.0, 480.0, 240.0, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        return camera_info


    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            # Update header and publish Image
            ros_image.header.frame_id = "camera_frame"
            self.image_pub.publish(ros_image)

            # Publish CameraInfo with updated timestamp
            self.camera_info.header.stamp = self.get_clock().now().to_msg()
            self.camera_info_pub.publish(self.camera_info)
        else:
            self.get_logger().error('Failed to capture image')

    def stop_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_camera()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()