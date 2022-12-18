# adroc
ROS package for Autonomous Diver-Relative Operator Configuration for AUVs.

## How to run ADROC

1. Start the diver detector and body pose estimator nodes (the body pose estimator node might need a venv active to work?). 
2. Once those nodes are running, run `rosrun adroc drp_only.launch` to start the Diver-Relative-Pose estimator. 
3. With the DRP node running, start the target_following based PID controller: `roslaunch target_following reconfigure_drp_yaw_pitch_controller.launch`
4. Now, run the main ADROC node, which will listen to the DRP information, activate and deactivate the PID controller, etc. 
5. The ADROC node will start inactive. To get it to approach someone, send a message to `/adroc/goal` (I think that's the topic name). You should be able to just tab complete the message, and don't need to put any information into the message. Sending this message will activate the ADROC system.
