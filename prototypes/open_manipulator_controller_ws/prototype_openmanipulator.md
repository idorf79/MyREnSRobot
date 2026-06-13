# Open Manipulator Prototype

Sources used:

<https://automaticaddison.com/configure-moveit-2-for-a-simulated-robot-arm-ros-2-jazzy/>

The question of this prototype: how can, with simple command line commands, make the robot arm move to certain locations and perform actions (open/close gripper).

Since were using a simulated environement, here the commands to start it:

## Simulation

```bash
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```

```bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=true
```

```bash
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py
```

After starting, in Rviz2 (MotionPlanning -> Joints)

- Move Joint 4 to 90 degrees;
- Move Joint 1 to 90 degrees;

Then (MotionPlanning -> Planning):

- "Plan & Execute"

## Start own arm controller

### Start controller

```bash
cd ~/projects/2526-MyRobot/prototypes/open_manipulator_controller_ws/
source install/setup.bash

ros2 launch open_manipulator_controller arm_controller.launch.py use_sim_time:=true
```

### Start client

```bash
cd ~/projects/2526-MyRobot/prototypes/open_manipulator_controller_ws/
source install/setup.bash

ros2 run open_manipulator_controller example_client
```
