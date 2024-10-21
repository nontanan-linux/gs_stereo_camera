import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Declare parameters
        self.declare_parameter('left_ns', '/stereo/left')
        self.declare_parameter('right_ns', '/stereo/right')
        self.declare_parameter('minDisparity', 12)
        self.declare_parameter('numDisparities', 5)
        self.declare_parameter('blockSize', 15)
        self.declare_parameter('show_image', False)
        # Get parameter values
        self.left_ns = self.get_parameter('left_ns').get_parameter_value().string_value
        self.right_ns = self.get_parameter('right_ns').get_parameter_value().string_value
        self.minDisparity = self.get_parameter('minDisparity').get_parameter_value().integer_value
        self.numDisparities = self.get_parameter('numDisparities').get_parameter_value().integer_value
        self.blockSize = self.get_parameter('blockSize').get_parameter_value().integer_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        # StereoSGBM setup
        self.stereo = cv2.StereoSGBM_create(minDisparity=self.minDisparity,
                                            numDisparities=16*self.numDisparities, 
                                            blockSize=self.blockSize)
        # stereo = cv2.StereoBM_create(numDisparities=16*self.numDisparities, blockSize=self.blockSize)
        self.bridge = CvBridge()
        # Subscriptions to left and right images
        self.left_image_sub = self.create_subscription(
            Image, f'{self.left_ns}/image_raw', self.left_image_callback, 10)
        self.right_image_sub = self.create_subscription(
            Image, f'{self.right_ns}/image_raw', self.right_image_callback, 10)
        self.left_image = None
        self.right_image = None
        # Timeout handling
        self.image_timeout = 1.0  # in seconds
        self.left_last_received = self.get_clock().now()
        self.right_last_received = self.get_clock().now()
        # Timer to periodically check stereo images and timeout
        self.stereo_timer = self.create_timer(0.05, self.stereo_callback)

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.left_last_received = self.get_clock().now()

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.right_last_received = self.get_clock().now()

    def stereo_callback(self):
        # Calculate time differences
        current_time = self.get_clock().now()
        left_time_diff = (current_time - self.left_last_received).nanoseconds / 1e9
        right_time_diff = (current_time - self.right_last_received).nanoseconds / 1e9
        # Check for timeout
        if left_time_diff > self.image_timeout:
            self.get_logger().warning('Left image timeout')
        if right_time_diff > self.image_timeout:
            self.get_logger().warning('Right image timeout')
        if self.left_image is not None and self.right_image is not None:
            if left_time_diff < self.image_timeout and right_time_diff < self.image_timeout:
                # Convert to grayscale
                gray_left = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
                gray_right = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)
                # Compute disparity
                disparity = self.stereo.compute(gray_left, gray_right)
                # Normalize and apply color map
                disparity_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                depth_colormap = cv2.applyColorMap(disparity_norm, cv2.COLORMAP_JET)
                # Normalize the disparity map for visualization
                # disp_visual = cv2.normalize(disparity, None, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(disparity, alpha=0.03), cv2.COLORMAP_JET)
                if self.show_image:
                    # Show the depth map (optional)
                    cv2.imshow('Disparity Map', depth_colormap)
                    cv2.waitKey(1)  # Display the image for a short period
                    
    def depth_map(imgL, imgR):
        window_size = 2  # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
        left_matcher = cv2.StereoSGBM_create(
            minDisparity=-1,
            numDisparities=16*16,  # max_disp has to be dividable by 16 f. E. HH 192, 256
            blockSize=window_size,
            P1=9 * 3 * window_size,
            # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
            P2=128 * 3 * window_size,
            disp12MaxDiff=12,
            uniquenessRatio=40,
            speckleWindowSize=50,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
        # FILTER Parameters
        lmbda = 70000
        sigma = 1.7
        visual_multiplier = 6
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)
        displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
        dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
        displ = np.int16(displ)
        dispr = np.int16(dispr)
        filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!
        filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
        filteredImg = np.uint8(filteredImg)
        return filteredImg

def main(args=None):
    rclpy.init(args=args)
    stereo_node = StereoCameraNode()
    try:
        rclpy.spin(stereo_node)
    except KeyboardInterrupt:
        pass
    finally:
        stereo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
