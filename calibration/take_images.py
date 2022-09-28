'''
pre-req: Create a folder named calib_images in the same location as the take_images.py file
How to save images:
1. Run: python3 take_images.py
2. Wait for the live feed from the camera to pop up
3. Press the spacebar to capture images. You can capture as many images as you want.
4. After taking sufficient number of pictures, press q to exit the code.
'''

import time
import cv2
# initialize the camera
print("Turning on Camera...")
print("Wait for Live Feed")
cam_port = 0
cam = cv2.VideoCapture(0)
i = 0
while True:
	ret, frame = cam.read()
	if ret is False:
		print("Counldn't read camera feed")
		break
  	
	cv2.imshow("Live Feed", frame)
	k = cv2.waitKey(1)

	# press the spacebar to capture an image and save it
	if k%256 == 32:
		print("Image ", i, " captured")
		image_name = "cal_img_" + str(i) + ".png"
		cv2.imwrite("./calib_images/" + image_name, frame)
		i += 1
	# press the 'q' key to exit
	if k & 0xFF == ord('q'):
		print("Exiting...")	
		break
cam.release()
cv2.destroyAllWindows()
print("Exited out of the code.")
