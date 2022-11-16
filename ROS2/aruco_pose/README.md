# Pose Estimation node

## Set up
Move aruco_pose folder in ros2_galactic/src/

## Run node for pose estimation

cd ros2_galactic/
colcon build --packages-select aruco_pose
ros2 run aruco_pose get_pose
