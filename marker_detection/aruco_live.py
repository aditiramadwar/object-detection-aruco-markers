# import the necessary packages
import argparse
import imutils
import cv2
import sys

winName = 'Marker detection: Live Feed'
cv2.namedWindow(winName, cv2.WINDOW_NORMAL)

cap = cv2.VideoCapture(0)
print ("Capturing video...")

while(True):
	hasFrame, image = cap.read()
	if not hasFrame:
		print("Done processing !!!")
		cv2.waitKey(3000)
		cap.release()
		break
	#image = cv2.imread(args["image"])
	image = imutils.resize(image, width=600)

	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
	arucoParams = cv2.aruco.DetectorParameters_create()
	(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
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








