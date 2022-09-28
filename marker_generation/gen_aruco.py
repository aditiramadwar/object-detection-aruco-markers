# import the necessary packages
import numpy as np
import argparse
import cv2
import sys
#DICT_4X4_50

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output_path", required=False,default="../markers/", help="path to output image containing ArUCo tag")
ap.add_argument("-i", "--id", type=int, required=True, help="ID of ArUCo tag to generate")
ap.add_argument("-t", "--type", type=str,default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to generate")
args = vars(ap.parse_args())

marker_id = args["id"]
output_file_name = args["output_path"]+"/" + str(marker_id) + ".png"

# load the ArUCo dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# allocate memory for the output ArUCo tag and then draw the ArUCo
# tag on the output image
print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(args["type"], args["id"]))
tag = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, args["id"], 300, tag, 1)

# write the generated ArUCo tag to disk and then display it to our screen
cv2.imwrite(output_file_name, tag)
cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(0)
