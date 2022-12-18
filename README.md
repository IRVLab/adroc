# adroc
ROS package for Autonomous Diver-Relative Operator Configuration for AUVs.

## How to run ADROC

1. Start the diver detector(`roslaunch darknet_ros diver_v4t.launch`) and body pose estimator nodes (`roslaunch trt_pose_pkg loco_trt_pose_estimator.launch`) (the body pose estimator node might need a venv active to work?). You can also launch both of the nodes together by running `roslaunch adroc diver_perception_nodes.launch` instead of running them seperately. 
2. Once those nodes are running, run `rosrun adroc drp_only.launch` to start the Diver-Relative-Pose estimator. 
3. With the DRP node running, start the target_following based PID controller: `roslaunch target_following reconfigure_drp_yaw_pitch_controller.launch`
4. Now, run the main ADROC node, `roslaunch adroc aoc_only.launch` which will listen to the DRP information, activate and deactivate the PID controller, etc. 
5. The ADROC node will start inactive. To get it to approach someone, send a message to `/adroc/goal` (I think that's the topic name). You should be able to just tab complete the message, and don't need to put any information into the message. Sending this message will activate the ADROC system.
