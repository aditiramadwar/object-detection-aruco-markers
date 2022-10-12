'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import cv2
import sys
#from utils import ARUCO_DICT
import argparse
import time
homogeneous_rotation_matrix = np.array([[0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0, 0, 0, 1]],
                                        dtype=float)
# data to get from planner
# def pose_camera_in_world()

def get_homogeneous_matrix(pose):
    rot, tvec = pose
    # homogeneous matrix 4x4 [r|t]
    homogeneous_rotation_matrix[:3, :3] = rot
    homogeneous_rotation_matrix[:3, 3] = tvec
    print("homogeneous_rotation_matrix: \n", homogeneous_rotation_matrix)
    return homogeneous_rotation_matrix


def pose_marker_in_world(Twc, poses):
    all_Twa = []
    for i in range(len(poses)):
        rvec, tvec = poses[i]
        # Convert Rotation vector to matrix
        rot, _ = cv2.Rodrigues(rvec) 	
        # print("Rotation 3x3: \n", rot)

        get_homogeneous_matrix([rot, tvec])

        # covert reference frame
        # need to refine calculation. output not consistent
        Twa = Twc @ homogeneous_rotation_matrix
        # print("Twa: \n", Twa)

        all_Twa.append(Twa)

    return all_Twa


def aruco_pose_esitmation(frame, camera, marker_length, Twc):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, _ = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)

    marker_poses = [None]*len(corners)
        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera[0],
                                                                       camera[1])
            vec = [rvec, tvec]
            marker_poses[i]=vec
            # print("For Marker", i, "the depth is:",tvec[0][0][2])
            # print("Marker: ", i)

            # rvec = [theta_x, theta_y, theta_z]
            # print("Rotation 3x1:", rvec)


            # tvec = [tx, ty, tz]
            # print("Translation:", tvec)


            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.drawFrameAxes(frame, camera[0], camera[1], rvec, tvec, 0.1)  
    else:
        print("No marker found!")

    return frame, marker_poses

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    #ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    #ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-m", "--marker_length", type=float, default=0.2, help="Length of ArUCo tag to detect(in meters)")

    args = vars(ap.parse_args())

    marker_length = args["marker_length"]
    #calibration_matrix_path = args["K_Matrix"]
    #distortion_coefficients_path = args["D_Coeff"]
    calibration_matrix_path = "../calibration/calibration_matrices/calibration_matrix.npy"
    distortion_coefficients_path = "../calibration/calibration_matrices/distortion_coefficients.npy"

    print("Accessing ", calibration_matrix_path)
    k = np.load(calibration_matrix_path)
    print("Accessing ", distortion_coefficients_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(0)
    time.sleep(2.0)
    # this matrix will be a paramter to this function (we will subscribe to the planner for this data)
    # Twa = Twc*Tca
    Twc = np.array([[1, 0, 0, 10],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]],
                    dtype=float)
    camera_intrincics = [k, d]
    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        output, all_poses = aruco_pose_esitmation(frame, camera_intrincics, marker_length, Twc)

        # this calculation needs to be refined
        markers = pose_marker_in_world(Twc, all_poses)
        for i in range(len(markers)):
            print("Marker ",i," pose wrt world frame:\n", markers[i])

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()
