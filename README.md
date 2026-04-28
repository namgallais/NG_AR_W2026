Final assignment, option 1 : Pick and Place

I've created a action client node named pick_n_place in the pick_n_place package that is a client to both PoseCommand and ParallelGripperCommand actions. This node load ros2 parameters x_vec, y_vec, z_vec, roll_vec, pitch_vec, yaw_vec, grasp_vec, done_vec and next_vec from the yaml. The i-th value of those vectors describe the i-th pose and gripper target for the robot. The node iterate through those steps to reach each target one after another. When kinematic_controller is launched, the robot goes to it's home postion {x: 0.5, y: 0.0, z: 0.7, roll: 0.0, pitch: 0.0, yaw: 0.0}. Then the pick_n_place node gets it to :
pregrasp pose : "{x: 0.5, y: 0.0, z: 0.525, roll: 0.0, pitch: 0.0, yaw: 0.0}"
grasp : "{command: {name: ['right_finger_bottom_joint'], position: [0.65]}}"
lift : "{x: 0.5, y: 0.0, z: 0.7, roll: 0.0, pitch: 0.0, yaw: 0.0}" (lift first to dodge the cup)
turn above the cup :"{x: 0.445, y: 0.205, z: 0.7, roll: 0.0, pitch: 0.0, yaw: 0.0}"
ungrasp : "{command: {name: ['right_finger_bottom_joint'], position: [0.0]}}"

The done_vec is just : [false, false, false, false, true]
and the next vector tell the node which index is the next target next_vec:  [1, 2, 3, 4, 0], here it's not very fancy, but it could be useful for more complexe tasks with conditions to determine the next target. Those vectors can be as long as needed without touching the source code.

The launch file first launch kortex_sim, then waits 10s to launch kinematic_controller such that everything have the time to load, and then waits 5s to launch pick_n_place. With that configuration it worked every time for me.

My video for the cube : https://youtu.be/tTO6IRj6xqQ
