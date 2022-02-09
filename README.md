# Object detection using Aruco Markers

Window detection using Aruco markers. The project requires minimum of two aruco markers for detection. 
All the data files are saved in the data/ folder.
## Set up
    pip install imutils
    pip install opencv-python
    pip install opencv-contrib-python

## Run the files
#### Detect the window in an image:
    python3 aruco_image.py  -i img.png -t DICT_4X4_50
![output image](https://github.com/aditiramadwar/object-detection-aruco-markers/blob/main/data/output.png)
#### Detect the window live:
    python3 aruco_live.py -t DICT_4X4_50
add video
#### Generate makers: To generate new/multiple markers

    python3 gen_aruco.py  --output data/aruco_marker.png --id 0 --type DICT_4X4_50
   Save the images and pdf and take clear print outs of these markers.

### References
1. [pyimagesearch.com/generating-aruco-markers-with-opencv-and-python/](https://www.pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/)
2.  [pyimagesearch.com/detecting-aruco-markers-with-opencv-and-python/](https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/)
2. [OpenCV.rectangle()](https://docs.opencv.org/4.x/d6/d6e/group__imgproc__draw.html#gac865734d137287c0afb7682ff7b3db23)
