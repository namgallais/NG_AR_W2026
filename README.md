# Final Assignment – Option 1: Pick and Place

I created an action client node named `pick_n_place` in the `pick_n_place` package. This node acts as a client for both `PoseCommand` and `ParallelGripperCommand` actions.

The node loads ROS 2 parameters `x_vec`, `y_vec`, `z_vec`, `roll_vec`, `pitch_vec`, `yaw_vec`, `grasp_vec`, `done_vec`, and `next_vec` from a YAML file. The i-th value of these vectors describes the i-th pose and gripper target for the robot. The node iterates through these steps sequentially to reach each target one after another.

When `kinematic_controller` is launched, the robot moves to its home position:
{ x: 0.5, y: 0.0, z: 0.7, roll: 0.0, pitch: 0.0, yaw: 0.0 }

Then the `pick_n_place` node executes the following sequence:

- Pre-grasp pose:
  { x: 0.5, y: 0.0, z: 0.525, roll: 0.0, pitch: 0.0, yaw: 0.0 }

- Grasp:
  { command: { name: ['right_finger_bottom_joint'], position: [0.65] } }

- Lift (to avoid the cup):
  { x: 0.5, y: 0.0, z: 0.7, roll: 0.0, pitch: 0.0, yaw: 0.0 }

- Move above the cup:
  { x: 0.445, y: 0.205, z: 0.7, roll: 0.0, pitch: 0.0, yaw: 0.0 }

- Release:
  { command: { name: ['right_finger_bottom_joint'], position: [0.0] } }

The `done_vec` is simply:
[false, false, false, false, true]

The `next_vec` defines the next target index:
[1, 2, 3, 4, 0]

Here it's not very fancy, but this structure allows for more complex behaviors (e.g., conditional branching to determine the next target). These vectors can be extended without modifying the source code.

## Launch Sequence

The launch file:
1. Starts `kortex_sim`
2. Waits 10 seconds so that gazebo has the time to load.
3. Launches `kinematic_controller`
4. Waits 5 seconds
5. Launches `pick_n_place`

With this configuration, the system worked reliably in testing.

## Demo Video

https://youtu.be/tTO6IRj6xqQ
