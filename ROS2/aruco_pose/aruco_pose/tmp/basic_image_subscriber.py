# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
   
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from pose_estimation import aruco_pose_esitmation
  
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self, camera_mat=None, trans=None):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      2)
    self.subscription # prevent unused variable warning
    self.camera_mat = camera_mat   
    self.trans = trans
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
  
    current_frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    #im_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    # Convert ROS Image message to OpenCV image
    #current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
    if self.camera_mat is not None and self.trans is not None:
      current_frame, all_poses = aruco_pose_esitmation(frame, self.camera_mat, 0.2, self.trans)
      # markers = pose_marker_in_world(Twc, all_poses)
      #   for i in range(len(markers)):
      #       print("Marker ",i," pose wrt world frame:\n", markers[i])

    # Display image
    cv2.imshow("camera", current_frame)
     
    cv2.waitKey(1)
   
def main(args=None):
   
  # Specify paths for the calibration matrices
  calibration_matrix_path = "./calibration_matrices/calibration_matrix.npy"
  distortion_coefficients_path = "./calibration_matrices/distortion_coefficients.npy"

  # Load the matrices in a readable format
  print("Accessing ", calibration_matrix_path)
  k = np.load(calibration_matrix_path)
  print("Accessing ", distortion_coefficients_path)
  d = np.load(distortion_coefficients_path)

  camera_intrincics = [k, d]

  Twc = np.array([[1, 0, 0, 10],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]],
                dtype=float)

  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  image_subscriber = ImageSubscriber(camera_mat = camera_intrincics, trans = Twc)
   
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
