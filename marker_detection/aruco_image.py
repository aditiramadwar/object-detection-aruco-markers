# python3 aruco_image.py  -i img.png
# import the necessary packages
import argparse
import imutils
import cv2
import sys

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to input image containing ArUCo tag")

args = vars(ap.parse_args())
print("[INFO] loading image...")
image =  cv2.imread(args["image"])
image = imutils.resize(image, width=600)

# load the ArUCo dictionary, grab the ArUCo parameters, and detect
# the markers
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()
(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
# verify *at least* one ArUco marker was detected
x_coord=[0,0]
y_coord=[0,0]
idx=0
if len(corners) > 0:
	# flatten the ArUco IDs list
	ids = ids.flatten()
	# loop over the detected ArUCo corners
	for (markerCorner, markerID) in zip(corners, ids):
		# extract the marker corners (which are always returned in
		# top-left, top-right, bottom-right, and bottom-left order)
		corners = markerCorner.reshape((4, 2))
		(topLeft, topRight, bottomRight, bottomLeft) = corners
		# convert each of the (x, y)-coordinate pairs to integers
		topRight = (int(topRight[0]), int(topRight[1]))
		bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
		bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
		topLeft = (int(topLeft[0]), int(topLeft[1]))
		# compute and draw the center (x, y)-coordinates of the ArUco
		# marker
		cX = int((topLeft[0] + bottomRight[0]) / 2.0)
		cY = int((topLeft[1] + bottomRight[1]) / 2.0)
		cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
		# draw the ArUco marker ID on the image
		# print("[INFO] ArUco marker ID: {}".format(markerID))
		if (str(markerID)=='0'):
			idx = 0;
		elif (str(markerID)=='1'):
			idx = 1;
		x_coord[idx] = cX
		y_coord[idx] = cY
	# draw a rectangle using the aruco marker coordinates
	cv2.rectangle(image, (x_coord[0],y_coord[0]), (x_coord[1],y_coord[1]), (0, 255, 0), 2)
	# get the center of the window detected
	cX = int((x_coord[0] + x_coord[1]) / 2.0)
	cY = int((y_coord[0] + y_coord[1]) / 2.0)
	cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
	# show the output image
	cv2.imshow("Image", image)
	# press 'q' key to close image
	cv2.waitKey(0)
	# Save the output image
	cv2.imwrite("data/output.png", image)
	print("Image saved!")








