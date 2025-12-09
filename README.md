# E0509 Robot with RH-P12-RN-A Gripper

Doosan E0509 robot arm integrated with ROBOTIS RH-P12-RN-A gripper for ROS2 Humble.

## Prerequisites

Add to your `~/.bashrc`:
```bash
export ROS_DISTRO='humble'
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/doosan_ws/install/rh_p12_rn_a_description/share:~/doosan_ws/install/dsr_description2/share
```

Then reload:
```bash
source ~/.bashrc
```

## Installation
```bash
cd ~/doosan_ws/src
git clone https://github.com/fhekwn549/e0509_gripper_description.git
cd ~/doosan_ws
colcon build --symlink-install --packages-select e0509_gripper_description
source install/setup.bash
```

## Usage

### RViz + Virtual Robot (Doosan Emulator)
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual host:=127.0.0.1 port:=12345
```

### RViz + Real Robot
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<ROBOT_IP> port:=12345
```

---

## Gazebo Simulation Setup

### Required: Modify dsr_controller2_gz.yaml

Before running Gazebo simulation, update the controller config:
```bash
cat > ~/doosan_ws/src/doosan-robot2/dsr_controller2/config/dsr_controller2_gz.yaml << 'YAML'
/**:
  controller_manager:
    ros__parameters:
      update_rate: 100
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
      joint_trajectory_controller:
        type: joint_trajectory_controller/JointTrajectoryController
      gripper_controller:
        type: position_controllers/JointGroupPositionController

  joint_trajectory_controller:
    ros__parameters:
      joints:
        - joint_1
        - joint_2
        - joint_3
        - joint_4
        - joint_5
        - joint_6
      command_interfaces:
        - position
      state_interfaces:
        - position
        - velocity

  gripper_controller:
    ros__parameters:
      joints:
        - gripper_rh_r1
        - gripper_rh_r2
        - gripper_rh_l1
        - gripper_rh_l2
YAML
```

Then rebuild dsr_controller2:
```bash
cd ~/doosan_ws
colcon build --symlink-install --packages-select dsr_controller2
source install/setup.bash
```

### Launch Gazebo Simulation
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01
```

To disable RViz:
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 gui:=false
```

---

## Control Commands

### Robot Arm Control
```bash
# Move to position
ros2 topic pub --once /dsr01/joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [{positions: [0.5, 0.3, 0.3, 0.0, 0.5, 0.0], time_from_start: {sec: 2}}]
}"
```

### Gripper Control (Gazebo)
```bash
# Open gripper
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# Close gripper
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1, 1.1, 1.1, 1.1]}"
```

## File Structure
```
e0509_gripper_description/
├── config/
│   └── gz_controllers.yaml      # Gazebo controller config
├── launch/
│   ├── bringup.launch.py        # RViz + real/virtual robot
│   ├── bringup_gazebo.launch.py # Gazebo + RViz simulation
│   └── gazebo.launch.py         # Gazebo only
├── urdf/
│   └── e0509_with_gripper.urdf.xacro
├── CMakeLists.txt
├── package.xml
└── README.md
```
