#!/usr/bin/env python3

import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import rclpy
from rclpy.node import Node
import threading
from torch.utils.tensorboard import SummaryWriter
import math
import random
import tkinter as tk
from tkinter import messagebox

import point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import Point  # 추가
from std_msgs.msg import String


GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.2

last_odom = None
environment_dim = 20
velodyne_data = np.ones(environment_dim) * 10

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()

        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a

# td3 network
class td3(object):
    def __init__(self, state_dim, action_dim):
        # Initialize the Actor network
        self.actor = Actor(state_dim, action_dim).to(device)
    def get_action(self, state):
        # Function to get the action from the actor
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def load(self, filename, directory):
        # Function to load network parameters
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename))
        )
    
# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True

    if y > 3 and y < 4:
        goal_ok = False
    if x > 0.5 and x < 1.5 and y > 3 and y < 6:
        goal_ok = False
    if x > -1.5 and x < -0.5 and y > 3 and y < 6:
        goal_ok = False
    # Outer bounds check to prevent out-of-bounds positioning
    if x > 4.5 or x < -4.5 or y > 5.5 or y < 0.5:
        goal_ok = False

    return goal_ok

class GazeboEnv(Node):
    """Superclass for all Gazebo environments."""

    def __init__(self):
        super().__init__('env')

        self.environment_dim = 20
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.upper = 5.0
        self.lower = -5.0

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "r1"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        self.message_data = 'message'

        # Set up the ROS publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.set_state = self.create_publisher(ModelState, "gazebo/set_model_state", 10)

        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.pause = self.create_client(Empty, "/pause_physics")
        self.reset_proxy = self.create_client(Empty, "/reset_world")
        self.req = Empty.Request

        self.publisher = self.create_publisher(MarkerArray, "goal_point", 3)
        self.publisher2 = self.create_publisher(MarkerArray, "linear_velocity", 1)
        self.publisher3 = self.create_publisher(MarkerArray, "angular_velocity", 1)

        self.goal_pub = self.create_publisher(Point, "/goal_position", 10)  # 목표 위치 퍼블리셔 추가

        self.publisher_ = self.create_publisher(String, '/result_test', 10)


    def publish_message(self, message_data):
        msg = String()
        msg.data = message_data
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)     


    # Perform an action and read a new state
    def step(self, action):
        global velodyne_data
        target = False
        
        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        # Unpause physics
        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        # Pause physics
        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            pass
            self.pause.call_async(Empty.Request())
        except (rclpy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # read velodyne laser state
        done, collision, min_laser = self.observe_collision(velodyne_data)
        v_state = []
        v_state[:] = velodyne_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        self.odom_x = last_odom.pose.pose.position.x
        self.odom_y = last_odom.pose.pose.position.y
        quaternion = Quaternion(
            last_odom.pose.pose.orientation.w,
            last_odom.pose.pose.orientation.x,
            last_odom.pose.pose.orientation.y,
            last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            env.get_logger().info("GOAL is reached!")
            self.publish_message('GOAL is reached!')
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser)

        return state, reward, done, target

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        while not self.reset_proxy.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset : service not available, waiting again...')
            self.publish_message('reset : service not available, waiting again...')

        try:
            self.reset_proxy.call_async(Empty.Request())
            self.publish_message('reset')

        except rclpy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state



        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y

        # set a random goal in empty space in environment
        self.change_goal()
        # randomly scatter boxes in the environment
        self.random_box()
        self.publish_markers([0.0, 0.0])

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        try:
            self.pause.call_async(Empty.Request())
        except:
            print("/gazebo/pause_physics service call failed")

        v_state = []
        v_state[:] = velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        return state

    def publish_goal_position(self):
        # 목표 위치를 퍼블리시
        goal_msg = Point()
        goal_msg.x = self.goal_x
        goal_msg.y = self.goal_y
        goal_msg.z = 0.0
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Published new goal position: ({self.goal_x}, {self.goal_y})")

 

    def change_goal(self, x=None, y=None):
        # X와 Y 좌표의 맵 경계를 설정
        x_min, x_max = -4.5, 4.5
        y_min, y_max = 0.5, 5.5

        goal_ok = False

        while not goal_ok:
            if x is None or y is None:
                # 무작위로 x와 y 좌표를 선택하여 목표 위치 설정
                self.goal_x = float(random.uniform(x_min, x_max))
                self.goal_y = float(random.uniform(y_min, y_max))
            else:
                self.goal_x = x
                self.goal_y = y
                goal_ok = True

            self.get_logger().info(f"goal = ({self.goal_x}, {self.goal_y})")
            
            # 목표 위치가 장애물 구역에 있지 않도록 확인
            goal_ok = check_pos(self.goal_x, self.goal_y)
            self.publish_goal_position()  # 목표 위치 퍼블리시


 

    def set_goal(self, x, y):
        # Set the goal to specified coordinates
        self.change_goal(x, y)
        self.get_logger().info(f"New goal set to: ({x}, {y})")

    def random_box(self):
        # Randomly change the location of the boxes in the environment on each reset to randomize the training environment
        for i in range(4):
            name = "cardboard_box_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.0
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = float(abs(action[0]))
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = float(abs(action[1]))
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5.0
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0.0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            env.get_logger().info("Collision is detected!")
            env.publish_message('Collision is detected!')

            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        if target:
            env.get_logger().info("reward 500")
            return 500.0
        elif collision:
            env.get_logger().info("reward -100")
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2


class Odom_subscriber(Node):
    
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription

    def odom_callback(self, od_data):
        global last_odom
        last_odom = od_data

class Velodyne_subscriber(Node):

    def __init__(self):
        super().__init__('velodyne_subscriber')
        # LaserScan 구독 설정
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",  # TurtleBot의 기본 LaserScan 토픽 이름
            self.laser_to_pointcloud_callback,
            10
        )
        self.subscription

        # LaserScan을 PointCloud2로 변환하는 LaserProjection 객체 생성
        self.projector = LaserProjection()

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / environment_dim]]
        for m in range(environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / environment_dim]
            )
        self.gaps[-1][-1] += 0.03

    def laser_to_pointcloud_callback(self, scan):
        # LaserScan을 PointCloud2로 변환
        # if any(np.isinf(r) for r in scan.ranges):
        #     self.get_logger().warn("LaserScan 데이터에 inf 값이 포함되어 있습니다.")

        # 라이다 inf 값을 5로 설정 
        scan.ranges = [r if np.isfinite(r) else 5.0 for r in scan.ranges]
        cloud = self.projector.projectLaser(scan)
       
        # 변환된 cloud 데이터가 정상인지 확인
        # if not cloud:
        #     self.get_logger().error("변환된 PointCloud2 데이터가 비어 있습니다.")
        # elif self.has_inf_in_cloud(cloud):
        #     self.get_logger().error("PointCloud2 데이터에 inf 값이 포함되어 있습니다.")
        # else:
        #     self.get_logger().info(f"PointCloud2 데이터 변환 성공: ")  # {cloud}

            # 변환된 PointCloud2 데이터를 기존의 velodyne_callback 함수에서 처리
        self.velodyne_callback(cloud)


    def velodyne_callback(self, v):
        global velodyne_data
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        velodyne_data = np.ones(environment_dim) * 10
        for i in range(len(data)):
            if data[i][2] > -0.2:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        velodyne_data[j] = min(velodyne_data[j], dist)
                        break

if __name__ == '__main__':

    rclpy.init(args=None)

    # Set the parameters for the implementation
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
    seed = 0  # Random seed number
    max_ep = 500  # maximum number of steps per episode
    file_name = "td3_velodyne"  # name of the file to load the policy from
    environment_dim = 20
    robot_dim = 4

    torch.manual_seed(seed)
    np.random.seed(seed)
    state_dim = environment_dim + robot_dim
    action_dim = 2
    finish_episode_reward = 0
    # Create the network
    network = td3(state_dim, action_dim) 
    try:
        network.load(file_name, "/home/oem/my_DRL_td3_ws/src/turtlebot3_DRL_td3/td3/runs3/train/pytorch_models")
    except:
        raise ValueError("Could not load the stored model parameters")
    
    writer = SummaryWriter(log_dir="/home/oem/my_DRL_td3_ws/src/turtlebot3_DRL_td3/td3/runs3/test/tensorboard")

    done = True
    episode_timesteps = 0
    episode_test = 0
    episode_get_goal = 0
    episode_get_collision = 0
    episode_get_timeout = 0
    test_accuracy = 0
    # Create the testing environment
    env = GazeboEnv()
    odom_subscriber = Odom_subscriber()
    velodyne_subscriber = Velodyne_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_subscriber)
    executor.add_node(velodyne_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = odom_subscriber.create_rate(2)

    # Tkinter GUI to set goal and reset the robot
    def create_gui(env):
        def set_goal():
            try:
                x = float(entry_x.get())
                y = float(entry_y.get())
                env.set_goal(x, y)
            except ValueError:
                messagebox.showerror("Invalid Input", "Please enter valid numeric values for X and Y.")
        
        def reset_robot():
            env.reset()

        root = tk.Tk()
        root.title("TurtleBot Goal Setter")

        tk.Label(root, text="Goal X: ").grid(row=0, column=0)
        entry_x = tk.Entry(root)
        entry_x.grid(row=0, column=1)

        tk.Label(root, text="Goal Y: ").grid(row=1, column=0)
        entry_y = tk.Entry(root)
        entry_y.grid(row=1, column=1)

        set_goal_button = tk.Button(root, text="Set Goal", command=set_goal)
        set_goal_button.grid(row=2, column=0, columnspan=2, pady=5)

        reset_button = tk.Button(root, text="Reset Robot", command=reset_robot)
        reset_button.grid(row=3, column=0, columnspan=2, pady=5)

        root.mainloop()

    # Start the Tkinter GUI in a separate thread
    gui_thread = threading.Thread(target=create_gui, args=(env,), daemon=True)
    gui_thread.start()

    # Begin the testing loop
    while rclpy.ok():

        # On termination of episode
        if done:
            state = env.reset()
            action = network.get_action(np.array(state))
            # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
            a_in = [(action[0] + 1) / 2, action[1]]
            next_state, reward, done, target = env.step(a_in)
            done = 1 if episode_timesteps + 1 == max_ep else int(done)

            done = False
            episode_timesteps = 0

            if finish_episode_reward == 100 :               
                episode_get_goal = episode_get_goal + 1
                episode_test = episode_test + 1
                test_accuracy = episode_get_goal/episode_test
            if finish_episode_reward == -100 :
                episode_get_collision = episode_get_collision + 1 
                episode_test = episode_test + 1               
                test_accuracy = episode_get_goal/episode_test

            env.get_logger().info(f"goal : {episode_get_goal}")
            # env.get_logger().info(f"collision : {episode_get_collision}")
            # env.get_logger().info(f"episode : {episode_test}")
            # env.get_logger().info(f"accuracy : {test_accuracy}")

            writer.add_scalar("accuracy", test_accuracy , episode_test)

            # writer.add_scalar("Get Goal", episode_get_goal , episode_test)
            # writer.add_scalar("Get Collision ", episode_get_collision, episode_test)

        else:
            action = network.get_action(np.array(state))
            # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
            a_in = [(action[0] + 1) / 2, action[1]]
            next_state, reward, done, target = env.step(a_in)
            finish_episode_reward = reward 
            done = 1 if episode_timesteps + 1 == max_ep else int(done)
            state = next_state
            episode_timesteps += 1

    rclpy.shutdown()