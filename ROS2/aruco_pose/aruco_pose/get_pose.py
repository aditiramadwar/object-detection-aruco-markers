
'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import numpy as np
import cv2
import sys
import argparse
import time

class pose_estimation():

  def __init__(self, camera_intrincics, Twc, marker_length):

    self.Twc = Twc
    self.marker_length = marker_length
    self.camera_intrincics = camera_intrincics
    self.homogeneous_rotation_matrix = np.array([[0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 1]],
                                                dtype=float)

    cv2.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    self.parameters = cv2.aruco.DetectorParameters_create()

  def get_homogeneous_matrix(self, pose):

      rot, tvec = pose
      # homogeneous matrix 4x4 [r|t]
      self.homogeneous_rotation_matrix[:3, :3] = rot
      self.homogeneous_rotation_matrix[:3, 3] = tvec

      # mat = np.around(self.homogeneous_rotation_matrix, decimals=2)
      # print("homogeneous_rotation_matrix: \n", mat)

      return self.homogeneous_rotation_matrix


  def pose_marker_in_world(self, poses):

      all_Twa = []
      for i in range(len(poses)):
          rvec, tvec = poses[i]
          # Convert Rotation vector to matrix
          rot, _ = cv2.Rodrigues(rvec) 	
          # print("Rotation 3x3: \n", rot)

          homogeneous_rotation_matrix = self.get_homogeneous_matrix([rot, tvec])

          # covert reference frame
          # need to refine calculation. output not consistent
          Twa = self.Twc @ homogeneous_rotation_matrix
          # print("Twa: \n", Twa)

          all_Twa.append(Twa)

      return all_Twa

  def aruco_pose_esitmation(self, frame):

      '''
      frame - Frame from the video stream
      matrix_coefficients - Intrinsic matrix of the calibrated camera
      distortion_coefficients - Distortion coefficients associated with your camera

      return:-
      frame - The frame with the axis drawn on it
      '''

      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      corners, ids, _ = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=self.parameters)
      marker_poses = [None]*len(corners)

      # If markers are detected
      if len(corners) > 0:
          for i in range(0, len(ids)):
              # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
              rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_intrincics[0],
                                                                         self.camera_intrincics[1])
              marker_poses[i] = [rvec, tvec]
              # print("For Marker", i, "the depth is:",tvec[0][0][2])
              # print("Marker: ", i)
              # rvec = [theta_x, theta_y, theta_z]
              # print("Rotation 3x1:", rvec)
              # tvec = [tx, ty, tz]
              # print("Translation:", tvec)

              # Draw a square around the markers
              cv2.aruco.drawDetectedMarkers(frame, corners) 

              # Draw Axis
              cv2.drawFrameAxes(frame, self.camera_intrincics[0], self.camera_intrincics[1], rvec, tvec, 0.1)  
      else:
          print("No marker found!")

      return frame, marker_poses

class ImagePublisher(Node):
  def __init__(self, camera_intrincics, marker_length, Twc):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('pose_publisher')

    self.camera_intrincics = camera_intrincics
    self.marker_length = marker_length
    self.Twc = Twc

    # TO DO: implement inheritance to avoid creating another object
    self.marker_pose = pose_estimation(self.camera_intrincics, self.Twc, self.marker_length)

    self.timer = self.create_timer(0.01, self.get_poses)

    self.video = cv2.VideoCapture(0)

  def get_poses(self):

    while True:
        ret, frame = self.video.read()

        if not ret:
            break
        
        output, all_poses = self.marker_pose.aruco_pose_esitmation(frame)

        # this calculation needs to be refined
        markers = self.marker_pose.pose_marker_in_world(all_poses)
        for i in range(len(markers)):

            mat = np.around(markers[i], decimals=2)
            print("Marker ", i," pose wrt world frame:\n", mat)

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    self.video.release()
    cv2.destroyAllWindows()
    print("Exit node!")

    self.destroy_node()



def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)

  marker_length = 0.2

  calibration_matrix_path = "/home/johnny/ros2_galactic/src/aruco_pose/aruco_pose/calibration_matrices/calibration_matrix.npy"
  distortion_coefficients_path = "/home/johnny/ros2_galactic/src/aruco_pose/aruco_pose/calibration_matrices/distortion_coefficients.npy"

  print("Accessing ", calibration_matrix_path)
  k = np.load(calibration_matrix_path)
  print("Accessing ", distortion_coefficients_path)
  d = np.load(distortion_coefficients_path)
  camera_intrincics = [k, d]

  # this matrix will be a paramter to this function (we will subscribe to the planner for this data)
  # Twa = Twc*Tca

  # Currently hardcoded a matrix where the world frame is translated 10 units in the positive x direction
  # of the camera frame.

  Twc = np.array([[1, 0, 0, 10],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]],
                  dtype=float)

  # Create the node
  image_publisher = ImagePublisher(camera_intrincics, marker_length, Twc)

  rclpy.spin(image_publisher)

  image_publisher.destroy_node()

  rclpy.shutdown()  


if __name__ == '__main__':
 
   main()
    