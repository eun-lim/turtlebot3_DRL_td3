# turtlebot3_DRL_td3

### 환경

운영 체제: Ubuntu 20.04

ROS 버전: ROS 2 Foxy 

로봇: TurtleBot3

알고리즘: TD3 (Twin Delayed Deep Deterministic policy gradient)

Deep Reinforcement Learning algorithm (DRL) 
Twin Delayed Deep Deterministic Policy Gradient (TD3)
---

## 환경 세팅

아래의 노션 페이지를 참고하세요 
https://cyber-splash-3b4.notion.site/147dd9f6a5288050b615d959c0a787db?pvs=4



### ROS2 설치 

ros2 fozy 버전을 설치합니다. ROBOTIS turtlebot e-manule을 참고하면 쉽게 설치할 수 있습니다.

```jsx
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
sudo chmod 755 ./install_ros2_foxy.sh
bash ./install_ros2_foxy.sh
```

만약 설치가 되지 않는다면 아래 링크의 공식 홈페이지 설치 방법에 따라 설치를 진행하세요
 
https://docs.ros.org/en/foxy/Installation.html


### turtlebot3 설치
이 코드는 turtlebot3를 사용하고 있으므로 로봇 사용을 위한 패키지 설치가 필요합니다. 아래의 링크에서 foxy 버전에 해당하는 터틀봇 패키지를 설치하세요. 만약 다른 로봇을 사용하고 있다면 src안의 models와 world, launch의 해당하는 부분을 변경하면 됩니다. 

https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

이 Repository에서는 시뮬레이션을 통해 train을 진행하므로 터틀봇 시뮬레이션 패키지도 설치해야합니다. 아래의 패키지를 설치하였는지 확인하고, 자세한 설치는 위의 e-manual을 참고하세요

```jsx
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/

git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

```

```jsx
$ echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
$ source ~/.bashrc
```

Repository를 시작하기 전에 turtlebot3가 정상적으로 동작하는지를 확인합니다.

터미널 1

```jsx
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

터미널 2

```jsx
ros2 run turtlebot3_teleop teleop_keyboard
```


---
### 가상환경 세팅 

저장소에 가상환경(myenv_td3)을 같이 업로드하였습니다. 

myenv_td3 폴더가 설치된 위치를 확인하고 경로를 설정한 후 가상환경을 활성화하여 사용하세요
```jsx
cd
git clone -b myenv_td3 https://github.com/eun-lim/turtlebot3_DRL_td3.git
source ~/myenv_td3/bin/activate
```


사용하는 라이브러리의 버전은 다음과 같습니다. 

```jsx
Python 3.8.10
NumPy version: 1.24.4
TensorBoard version: 2.14.0
PyTorch version: 1.10.0
squaternion version: 2023.9.2
matplotlib version: 3.7.5
```



---
### 실행

Repository를 다운로드 합니다. 

```jsx
mkdir -p ros2_ws/src
cd ~/ros2_ws/src
git clone -b ttb_td3_simulation https://github.com/eun-lim/turtlebot3_DRL_td3.git
```

```jsx
cd ~/ros2_ws
colcon build --packages-select laser_geometry --allow-overriding laser_geometry
colcon build --packages-select td3
```

사용되는 패키지는 다음과 같습니다. 
1. td3 
   
   1.1 시뮬레이션 환경 (gazebo) 및 rviz 실행하는 launch 파일
   
     ```jsx
     ros2 launch td3 mytrain_sim.launch.py
     
     ```
![image](https://github.com/user-attachments/assets/2466b480-707a-42aa-992c-536ce6a2216e)

    1.2 training 파일
   
     ```jsx
     ros2 run td3 my_train.py
     
     ```



1.3 test 파일

로봇이 목적지에 도달하거나, 벽에 충돌하거나, 일정 시간이 지나면 시스템은 자동으로 로봇의 목적지를 재설정하고 리셋합니다. 이 과정에서 로봇은 랜덤한 새로운 목적지로 이동을 시작합니다.

목적지 변경 : 로봇이 목적지로 이동 중일 때 사용자가 GUI를 통해 새로운 목적지를 입력하면, 로봇은 즉시 설정된 새 목적지로 방향을 변경합니다.

만약 장애물 위치로 목적지를 변경한다면 랜덤한 목적지가 설정됩니다.

로봇 초기화: 사용자가 GUI에서 Reset Robot 버튼을 누르면, 로봇은 초기 위치인 (0, 1)로 이동하고 랜덤한 목적지가 다시 설정됩니다.

  ```bash
  ros2 run td3 my_test.py
  
  ```

![image](https://github.com/user-attachments/assets/7aea747e-5b05-4fb3-84a1-af80712e6a64)


1.4 test 시 사용되는 gui 파일

로봇의 위치 및 목적지의 위치를 gui를 통해 확인할 수 있습니다. (train 시에는 사용할 수 없습니다)

파란점 : 로봇의 위치

빨간점 : 목적지 위치

회색 벽 : 장애물 위치

 ```bash
 ros2 run td3 gui.py
 ```
   
![image](https://github.com/user-attachments/assets/b3b6f12c-3ca8-4ce2-8570-29590ac814e6)



2. laser_geometry
   
     학습 및 테스트 시 라이다 데이터 변환을 위한 라이브러리 입니다. (https://github.com/ros-perception/laser_geometry)
    따라서 패키지 설치 후 아래와 같이 빌드합니다.
      
      ```jsx
         colcon build --packages-select laser_geometry --allow-overriding laser_geometry
      ```
      
      ```jsx
      sudo apt update
      sudo apt install ros-foxy-sensor-msgs
      ```
      
      그 후 `package.xml` 에 아래의 코드를 추가합니다.  
      
      ```jsx
      <depend>sensor_msgs</depend>
      ```

   



### TensorBoard  

TensorBoard.dev는 2024년 1월 1일자로 종료되었습니다. (https://tensorboard.dev/#get-started)

아래의 방법을 따라 텐서보드를 로컬에서 실행할 수 있도록 준비할 수 있습니다

```bash
mkdir -p tensorboard_ws
cd tensorboard_ws/
git clone https://github.com/tensorflow/tensorboard.git

```

Bazel을 설치하여 TensorBoard 를 빌드합니다. 이때 Bazel 6.5.0를 설치해아합니다. 

```bash
# Bazel 설치 디렉토리 삭제
sudo rm -rf /usr/local/bin/bazel

# 원하는 버전 설치 (6.5.0)
wget https://github.com/bazelbuild/bazel/releases/download/6.5.0/bazel-6.5.0-installer-linux-x86_64.sh
chmod +x bazel-6.5.0-installer-linux-x86_64.sh
./bazel-6.5.0-installer-linux-x86_64.sh --user

# 버전 확인 
bazel --version

# 경로 추가 
echo 'export PATH="$PATH:/home/robo/bin"' >> ~/.bashrc
source ~/.bashrc

```

TensorBoard를 빌드 합니다. 

```bash
cd ~/tensorboard_ws/tensorboard
bazel build //tensorboard

```

학습 결과를 확인합니다. 

```python
cd ~/tensorboard_ws/tensorboard
./bazel-bin/tensorboard/tensorboard --logdir=/home/robo/foxy_ws/src/DRL_Navigation_Robot_ROS2_Foxy/src/td3/runs/train/tensorboard/
```

![image](https://github.com/user-attachments/assets/788632c1-1826-4479-8644-49fc97460fb8)



---
### 참고 자료
https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy



