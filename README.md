# E0509 + RH-P12-RN-A Gripper Description

Doosan E0509 로봇팔과 ROBOTIS RH-P12-RN-A 그리퍼를 결합한 ROS2 패키지

## 개요

이 패키지는 Doosan E0509 6축 로봇팔에 ROBOTIS RH-P12-RN-A 그리퍼를 장착한 통합 로봇 시스템을 위한 URDF, launch 파일, 그리퍼 컨트롤러를 제공합니다.

## 특징

- ✅ E0509 + 그리퍼 결합 URDF/XACRO
- ✅ Doosan Virtual Robot (에뮬레이터) 지원
- ✅ ros2_control 기반 조인트 제어
- ✅ 그리퍼 stroke 기반 제어 (DART Platform 호환)
- ✅ RViz 시각화
- ✅ Gazebo 시뮬레이션 지원

## 의존성

- ROS2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- [doosan-robot2](https://github.com/doosan-robotics/doosan-robot2)
- [RH-P12-RN-A](https://github.com/ROBOTIS-GIT/RH-P12-RN-A)

## Docker 설치 (Virtual Mode 필수)

Virtual mode 에뮬레이터 사용을 위해 Docker가 필요합니다.

### 1. Docker 설치
```bash
# Docker 공식 설치 (https://docs.docker.com/engine/install/ubuntu/)
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 현재 사용자를 docker 그룹에 추가 (재로그인 필요)
sudo usermod -aG docker $USER
```

### 2. 에뮬레이터 설치
```bash
cd ~/doosan_ws/src/doosan-robot2
chmod +x ./install_emulator.sh
sudo ./install_emulator.sh
```


## 사전 설정

`~/.bashrc`에 다음 내용 추가:
```bash
export ROS_DISTRO='humble'
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/doosan_ws/install/rh_p12_rn_a_description/share:~/doosan_ws/install/dsr_description2/share
```

적용:
```bash
source ~/.bashrc
```

## 설치
```bash
# 워크스페이스 생성
mkdir -p ~/doosan_ws/src
cd ~/doosan_ws/src

# 의존 패키지 클론
git clone https://github.com/doosan-robotics/doosan-robot2.git
git clone https://github.com/ROBOTIS-GIT/RH-P12-RN-A.git

# 이 패키지 클론
git clone https://github.com/fhekwn549/e0509_gripper_description.git

# 빌드
cd ~/doosan_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 사용법 (RViz + Virtual Robot)

### 1. RViz 시각화 (조인트 슬라이더)
```bash
ros2 launch e0509_gripper_description display.launch.py
```

### 2. Virtual Robot 실행 (에뮬레이터)
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual host:=127.0.0.1 port:=12345
```

### 3. 로봇 제어
```bash
# 조인트 이동
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [30.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"

# 홈 위치
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 30.0}"
```

### 4. 그리퍼 제어 (RViz + Virtual Robot)
```bash
# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger

# Stroke 값으로 제어 (0=열림, 700=완전히 닫힘)
ros2 topic pub /dsr01/gripper/stroke std_msgs/msg/Int32 "{data: 350}" --once
```

---

## 사용법 (Gazebo 시뮬레이션)

### 1. dsr_controller2_gz.yaml 수정 (필수)

Gazebo 시뮬레이션 실행 전, 컨트롤러 설정 파일 수정:
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

수정 후 리빌드:
```bash
cd ~/doosan_ws
colcon build --symlink-install --packages-select dsr_controller2
source install/setup.bash
```

### 2. Gazebo 시뮬레이션 실행
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01
```

RViz 없이 실행:
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 gui:=false
```

### 3. 로봇 제어 (Gazebo)
```bash
# 로봇 팔 이동
ros2 topic pub --once /dsr01/joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [{positions: [0.5, 0.3, 0.3, 0.0, 0.5, 0.0], time_from_start: {sec: 2}}]
}"
```

### 4. 그리퍼 제어 (Gazebo)
```bash
# 그리퍼 열기
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# 그리퍼 닫기
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1, 1.1, 1.1, 1.1]}"

# 절반 닫기
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.55, 0.55, 0.55, 0.55]}"
```

---

## 그리퍼 제어 인터페이스

### RViz + Virtual Robot
| 인터페이스 | 타입 | 설명 |
|-----------|------|------|
| `/dsr01/gripper/open` | Service (Trigger) | 그리퍼 열기 |
| `/dsr01/gripper/close` | Service (Trigger) | 그리퍼 닫기 |
| `/dsr01/gripper/stroke` | Topic (Int32) | Stroke 값 (0~700) |

### Gazebo Simulation
| 인터페이스 | 타입 | 설명 |
|-----------|------|------|
| `/dsr01/gripper_controller/commands` | Topic (Float64MultiArray) | Joint position (0.0~1.1) |

---

## 파일 구조
```
e0509_gripper_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── gz_controllers.yaml          # Gazebo 컨트롤러 설정
├── urdf/
│   └── e0509_with_gripper.urdf.xacro
├── launch/
│   ├── display.launch.py            # RViz 시각화
│   ├── bringup.launch.py            # Virtual/Real 로봇 실행
│   ├── bringup_gazebo.launch.py     # Gazebo + RViz 시뮬레이션
│   └── gazebo.launch.py             # Gazebo 전용
├── scripts/
│   └── gripper_joint_publisher.py   # 그리퍼 컨트롤러
└── rviz/
    └── display.rviz
```

## 환경

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Fortress (Ignition Gazebo 6)

## License

Apache-2.0

## Author

fhekwn549
