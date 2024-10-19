ros2 run gs_stereo_camera camera_node --ros-args -p device:=0 -p namespace:='/laptop'
ros2 launch gs_stereo_camera camera_node.launch.py device:=0 namespace:=/laptop