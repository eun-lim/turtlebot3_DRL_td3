#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tkinter as tk

class RobotPoseSubscriber(Node):
    def __init__(self):
        super().__init__('robot_pose_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',  
            self.pose_callback,
            10)
        self.goal_subscription = self.create_subscription(
            Point,
            'goal_position',  # 목표 지점을 나타내는 토픽
            self.goal_callback,
            10)


        self.x = 0.0
        self.y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.velodyne_data = []

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y

class RobotVisualizerApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.canvas = tk.Canvas(root, width=1000, height=600, bg='white')
        self.draw_grid()  # Draw grid lines on the canvas
        self.canvas.pack()
        self.robot = self.canvas.create_oval(245, 245, 255, 255, fill='blue')
        self.update_position()

    def draw_grid(self):
        # Draw restricted areas in light pink
        self.canvas.create_rectangle(500 + 0.9 * 100, 600 - 6 * 100, 500 + 1.1 * 100, 600 - 3.6 * 100, fill='lightgray', outline='', tags='restricted_area')
        self.canvas.create_rectangle(500 - 1.2 * 100, 600 - 6 * 100, 500 - 1.0 * 100, 600 - 3.6 * 100, fill='lightgray', outline='', tags='restricted_area')
        self.canvas.create_rectangle(0, 0, 1000, 0.2 * 100, fill='lightgray', outline='', tags='restricted_area')  # Top border
        self.canvas.create_rectangle(0, 600 - 0.2 * 100, 1000, 600, fill='lightgray', outline='', tags='restricted_area')  # Bottom border
        self.canvas.create_rectangle(0, 0, 0.2 * 100, 600, fill='lightgray', outline='', tags='restricted_area')  # Left border
        self.canvas.create_rectangle(1000 - 0.2 * 100, 0, 1000, 600, fill='lightgray', outline='', tags='restricted_area')  # Right border
        self.canvas.create_rectangle(0, 600 - 3.6 * 100, 500 - 2.8 * 100, 600 - 3.8 * 100, fill='lightgray', outline='', tags='restricted_area')
        self.canvas.create_rectangle(500 + 2.8 * 100, 600 - 3.6 * 100, 1000, 600 - 3.8 * 100, fill='lightgray', outline='', tags='restricted_area')
        

        for i in range(0, 1000, 100):
            self.canvas.create_line([(i, 0), (i, 600)], tag='grid_line', fill='lightgray', width=1)
            self.canvas.create_line([(0, i), (1000, i)], tag='grid_line', fill='lightgray', width=1)
            self.canvas.create_line([(0, i), (500, i)], tag='grid_line', fill='lightgray', width=1)

    def update_position(self):
        # ROS 2 노드에서 현재 로봇 위치를 가져옴
        x = self.node.x
        y = self.node.y

        # 캔버스 중심을 기준으로 좌표 변환
        canvas_x = 500 + x * 100
        canvas_y = 600 - y * 100

        # 로봇의 위치 업데이트
        self.canvas.coords(self.robot, canvas_x - 5, canvas_y - 5, canvas_x + 5, canvas_y + 5)
        
        # 100ms마다 업데이트 반복
        if self.node.goal_x is not None and self.node.goal_y is not None:
            goal_canvas_x = 500 + self.node.goal_x * 100
            goal_canvas_y = 600 - self.node.goal_y * 100
            self.canvas.create_oval(goal_canvas_x - 5, goal_canvas_y - 5, goal_canvas_x + 5, goal_canvas_y + 5, fill='red', outline='red', tags='goal')
        self.canvas.delete('goal')
        if self.node.goal_x is not None and self.node.goal_y is not None:
            goal_canvas_x = 500 + self.node.goal_x * 100
            goal_canvas_y = 600 - self.node.goal_y * 100
            self.canvas.create_oval(goal_canvas_x - 5, goal_canvas_y - 5, goal_canvas_x + 5, goal_canvas_y + 5, fill='red', outline='red', tags='goal')
        if self.node.velodyne_data:
            # Velodyne 데이터를 시각화 (예시: 포인트 클라우드를 캔버스에 표시)
            for point in self.node.velodyne_data:
                canvas_x = 500 + point.x * 100
                canvas_y = 600 - point.y * 100
                self.canvas.create_oval(canvas_x - 2, canvas_y - 2, canvas_x + 2, canvas_y + 2, fill='green', tags='velodyne')
        if self.node.goal_x is not None and self.node.goal_y is not None:
            goal_ok = True
            if 3 < self.node.goal_y < 4:
                goal_ok = False
            if 0.5 < self.node.goal_x < 1.5 and 3 < self.node.goal_y < 6:
                goal_ok = False
            if -1.5 < self.node.goal_x < -0.5 and 3 < self.node.goal_y < 6:
                goal_ok = False
            if self.node.goal_x > 4.5 or self.node.goal_x < -4.5 or self.node.goal_y > 5.5 or self.node.goal_y < 0.5:
                goal_ok = False
            color = 'red' if goal_ok else 'orange'
            self.canvas.create_oval(goal_canvas_x - 5, goal_canvas_y - 5, goal_canvas_x + 5, goal_canvas_y + 5, fill=color, outline=color, tags='goal')
        self.root.after(100, self.update_position)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseSubscriber()

    root = tk.Tk()
    root.title("Robot Movement Visualizer")

    app = RobotVisualizerApp(root, node)

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(100, ros_spin)

    root.after(100, ros_spin)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
