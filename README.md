# E0509 + RH-P12-RN-A Gripper Description

Doosan E0509 로봇팔과 ROBOTIS RH-P12-RN-A 그리퍼를 결합한 ROS2 패키지

## 개요

이 패키지는 Doosan E0509 6축 로봇팔에 ROBOTIS RH-P12-RN-A 그리퍼를 장착한 통합 로봇 시스템을 위한 URDF, launch 파일, 그리퍼 컨트롤러를 제공합니다.

## 특징

- ✅ E0509 + 그리퍼 결합 URDF/XACRO
- ✅ Doosan Virtual Robot (에뮬레이터) 지원
- ✅ **실제 로봇 그리퍼 제어** (Tool Flange Serial + Modbus RTU)
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

### 2_1. 로봇 제어
```bash
# 조인트 이동
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [30.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"

# 홈 위치
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 30.0}"
```

### 2_2. 그리퍼 제어
```bash
# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger

# Stroke 값으로 제어 (0=열림, 700=완전히 닫힘)
ros2 topic pub /dsr01/gripper/stroke std_msgs/msg/Int32 "{data: 350}" --once
```

---

## 사용법 (Gazebo + Virtual Robot)

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

### 2. Gazebo 시각화 실행
이 모드에서는 ros2_control 토픽으로만 제어 가능합니다. (Doosan 서비스 사용 불가)
```bash
ros2 launch e0509_gripper_description gazebo.launch.py
```


### 2_1. 로봇 제어

```bash
ros2 topic pub --once /e0509_gripper/joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [{positions: [0.5, 0.3, 0.3, 0.0, 0.5, 0.0], time_from_start: {sec: 2}}]
}"
```

### 2_2. 그리퍼 제어
```bash
# 그리퍼 열기
ros2 topic pub --once /e0509_gripper/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# 그리퍼 닫기
ros2 topic pub --once /e0509_gripper/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1, 1.1, 1.1, 1.1]}"
```

### 3. Gazebo 시뮬레이션 실행
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01
```

RViz 함께 실행:
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 gui:=true
```

### 3_1. 로봇 제어
```bash
# 로봇 팔 이동
ros2 topic pub --once /dsr01/joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [{positions: [0.5, 0.3, 0.3, 0.0, 0.5, 0.0], time_from_start: {sec: 2}}]
}"
```

### 3_2. 그리퍼 제어
```bash
# 그리퍼 열기
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# 그리퍼 닫기
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1, 1.1, 1.1, 1.1]}"

# 절반 닫기
ros2 topic pub --once /dsr01/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.55, 0.55, 0.55, 0.55]}"
```

---

## 사용법 (실제 로봇 + 그리퍼)

실제 Doosan E0509 로봇에 RH-P12-RN-A 그리퍼를 연결하여 제어합니다.
그리퍼는 로봇의 **Tool Flange Serial** 포트를 통해 **Modbus RTU** 프로토콜로 통신합니다.

### 1. 실제 로봇 실행
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<로봇IP> port:=12345
```

### 2. 그리퍼 제어 (gripper.py)
```bash
# 그리퍼 열기
ros2 run e0509_gripper_description gripper.py open

# 그리퍼 닫기
ros2 run e0509_gripper_description gripper.py close

# 특정 위치로 이동 (0=열림, 700=닫힘)
ros2 run e0509_gripper_description gripper.py pos 350

# 다른 namespace 사용 시
ros2 run e0509_gripper_description gripper.py --ns dsr01 open
```

### 3. 그리퍼 통신 사양
| 항목 | 값 |
|------|-----|
| 프로토콜 | Modbus RTU |
| 통신 포트 | Tool Flange Serial |
| Baudrate | 57600 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Slave ID | 1 |

### 4. 주요 Modbus 레지스터
| 레지스터 | 주소 | 설명 |
|---------|------|------|
| Torque Enable | 256 (0x0100) | 1=활성화 |
| Goal Position | 282 (0x011A) | 0~700 (2 registers) |
| Goal Current | 275 (0x0113) | 기본값 400 |

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

## Digital Twin (Isaac Sim 연동)

실제 로봇의 joint_states를 Isaac Sim에서 실시간으로 시각화합니다.
ROS2와 Isaac Sim의 Python 버전이 다르므로 (ROS2: 3.10, Isaac Sim: 3.11) 파일 기반 통신을 사용합니다.

### 실행 방법

**터미널 1: 로봇 실행**
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual
```

**터미널 2: ROS2 Bridge 실행** (시스템 Python 사용)
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
cd ~/doosan_ws/src/e0509_gripper_description/scripts
python3 digital_twin_bridge.py
```

**터미널 3: Isaac Sim 디지털 트윈** (CoWriteBotRL 레포 필요)
[CoWriteBotRL](https://github.com/KERNEL3-2/CoWriteBotRL)
```bash
source ~/isaacsim_env/bin/activate
cd ~/CoWriteBotRL
python pen_grasp_rl/scripts/digital_twin.py
```

### 동작 원리
1. `digital_twin_bridge.py`: ROS2에서 `/dsr01/joint_states`를 구독하여 `/tmp/doosan_joint_states.json`에 저장
2. `digital_twin.py`: JSON 파일을 읽어 Isaac Sim 로봇에 적용

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
│   ├── gripper_joint_publisher.py   # RViz 시각화용 그리퍼 컨트롤러
│   ├── gripper.py                   # 실제 그리퍼 제어 (Modbus RTU)
│   └── digital_twin_bridge.py       # Isaac Sim 연동용 ROS2 브릿지
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
