# import the necessary packages
import argparse
import imutils
import cv2
import sys
# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
#ap.add_argument("-i", "--image", required=True,
#	help="path to input image containing ArUCo tag")
ap.add_argument("-t", "--type", type=str,
	default="DICT_ARUCO_ORIGINAL",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

winName = 'Ribbon detection'
cv2.namedWindow(winName, cv2.WINDOW_NORMAL)

cap = cv2.VideoCapture(0)
print ("Capturing video...")
while(True):
	hasFrame, image = cap.read()
	if not hasFrame:
		print("Done processing !!!")
		print("Output file is stored as ", outputFile)
		cv2.waitKey(3000)
		cap.release()
		break
	#image = cv2.imread(args["image"])
	image = imutils.resize(image, width=600)
	# verify that the supplied ArUCo tag exists and is supported by
	# OpenCV
	if ARUCO_DICT.get(args["type"], None) is None:
		# print("[INFO] ArUCo tag of '{}' is not supported".format(
		# 	args["type"]))
		sys.exit(0)
	# load the ArUCo dictionary, grab the ArUCo parameters, and detect
	# the markers
	# print("[INFO] detecting '{}' tags...".format(args["type"]))
	arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
	arucoParams = cv2.aruco.DetectorParameters_create()
	(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
		parameters=arucoParams)
	# verify *at least* one ArUco marker was detected
	x_coord=[0,0]
	y_coord=[0,0]
	idx=0
	if len(corners) > 1:
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
			# print("[INFO] ArUco marker ID: {}".format(markerID))
			if (str(markerID)=='0'):
				idx = 0;
			elif (str(markerID)=='1'):
				idx = 1;
			x_coord[idx] = cX
			y_coord[idx] = cY
	cv2.rectangle(image, (x_coord[0],y_coord[0]), (x_coord[1],y_coord[1]), (0, 255, 0), 2)
	cX = int((x_coord[0] + x_coord[1]) / 2.0)
	cY = int((y_coord[0] + y_coord[1]) / 2.0)
	cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
	cv2.imshow(winName, image) 
	# press 'q' key to stop processing
	if cv2.waitKey(1) & 0xFF == ord('q'):
        	break








