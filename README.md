# BLDCGazeboROS2

To Setup:

Clone into bldc_ws/src/bldc_gz_sim, then when in bldc_ws

colcon build --symnlink-install

IMPORTANT
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/bldc_ws/src/bldc_gz_sim/models:$(ros2 pkg prefix bldc_gz_sim)/share/bldc_gz_sim/models
