# snu_sdx_controller
모바일 로봇 및 매니퓰레이터 제어기 프레임워크

# Requirement
- [mujoco_py](https://github.com/openai/mujoco-py)
- [ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

# Usage
## Build
```bash
cd ros2_ws # your ros2 workspace
colcon build --symlink-install
source install/setup.bash
```
## Setting
조이스틱의 버튼들의 index를 확인하여 mujoco_ros/config/teleop_joy.yaml을 수정

## Run
조이스틱을 활용하여 무조코에서 모바일 로봇을 구동
```bash
ros2 launch mujoco_ros mobile_w_joy_launch.py robot_name:=husky # husky, summit_xls, pcv
```

# TODO
- pcv 제어기 추가 수정
- 매니퓰레이터 제어기 추가
