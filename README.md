
# Object detection using Aruco Markers

Window detection using Aruco markers. The project requires minimum of two aruco markers for detection. 
All the data files are saved in the data/ folder.
## Set up
    pip install imutils
    pip install opencv-python
    pip install opencv-contrib-python

## Generate Aruco Markers

    cd marker_generation/
    python3 gen_aruco.py --id 0
   ![marker id 0 image](https://github.com/aditiramadwar/object-detection-aruco-markers/blob/main/markers/0.png)
   Save the images and pdf and take clear print outs of these markers.
   
## Detect Aruco Markers

#### Detect the window in an image:
Code is designed to detect two markers of the same dimentions at the same time.

    cd marker_detection
    python3 aruco_image.py  -i ../data/img.png
![output image](https://github.com/aditiramadwar/object-detection-aruco-markers/blob/main/data/output.png)
#### Detect the window live:
Code is designed to detect two markers of the same dimentions at the same time.

    cd marker_detection
    python3 aruco_live.py

## Pose Estimation
### 1. Calibrate Camera

#### Take calibration images
    python3 take_images.py
![output image](https://github.com/aditiramadwar/object-detection-aruco-markers/blob/main/calibration/calib_images/cal_img_0.png)
#### Calibration
    python calibration.py --dir calib_images/ --square_size 0.03

### 2. Pose Estimation

Marker of length 11cm
	
    cd pose_detection
    python3 pose_estimation.py --marker_length 0.11
![pose_big_marker](https://github.com/aditiramadwar/object-detection-aruco-markers/blob/main/data/pose_big_marker.png)

Marker of length 20cm:

    cd pose_detection
    python3 pose_estimation.py --marker_length 0.2
![pose_small_marker](https://github.com/aditiramadwar/object-detection-aruco-markers/blob/main/data/pose_small_marker.png)

## References
1. [Generate Aruco Markers/](https://www.pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/)
2. [Detect Aruco Markers](https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/)
3. [OpenCV.rectangle()](https://docs.opencv.org/4.x/d6/d6e/group__imgproc__draw.html#gac865734d137287c0afb7682ff7b3db23)
4. [ArUCo-Markers-Pose-Estimation-Generation-Python](https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python)

