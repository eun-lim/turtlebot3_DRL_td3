#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tkinter as tk
from std_msgs.msg import String

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
        
        self.result_subscription = self.create_subscription(
            String ,
            '/result_test',  # 결과 데이터를 나타내는 토픽
            self.result_callback,
            10)

        self.x = 0.0
        self.y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.result_x = None
        self.result_y = None
        self.msgs = None
        self.msg_lst = []


    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y

    def result_callback(self, msg):
        self.msgs = msg.data
        self.msg_lst.append(self.msgs)
        if len(self.msg_lst) > 5:
            self.msg_lst.pop(0)


class RobotVisualizerApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.canvas = tk.Canvas(root, width=1000, height=600, bg='white')
        self.draw_grid()  # Draw grid lines on the canvas
        self.canvas.pack()
        self.robot = self.canvas.create_oval(245, 245, 255, 255, fill='blue')

        # 텍스트 박스 추가
        self.text_box = tk.Text(root, height=11, bg="white", state=tk.DISABLED)
        self.text_box.pack(fill=tk.X)

        self.update_position()
        self.update_text()

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
        self.canvas.delete('goal')
        if self.node.goal_x is not None and self.node.goal_y is not None:
            goal_canvas_x = 500 + self.node.goal_x * 100
            goal_canvas_y = 600 - self.node.goal_y * 100
            self.canvas.create_oval(goal_canvas_x - 5, goal_canvas_y - 5, goal_canvas_x + 5, goal_canvas_y + 5, fill='red', outline='red', tags='goal')
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

    def update_text(self):
        # 텍스트 박스에 로봇과 목표 지점 위치를 갱신하여 표시
        self.text_box.config(state=tk.NORMAL)
        self.text_box.delete(1.0, tk.END)  # 이전 텍스트 삭제

        # 현재 로봇 위치 출력
        self.text_box.insert(tk.END, f"Robot Position:\n")
        self.text_box.insert(tk.END, f"  X: {self.node.x:.2f}\n")
        self.text_box.insert(tk.END, f"  Y: {self.node.y:.2f}\n\n")

        # 목표 위치 출력
        if self.node.goal_x is not None and self.node.goal_y is not None:
            self.text_box.insert(tk.END, f"Goal Position:\n")
            self.text_box.insert(tk.END, f"  X: {self.node.goal_x:.2f}\n")
            self.text_box.insert(tk.END, f"  Y: {self.node.goal_y:.2f}\n\n")
        else:
            self.text_box.insert(tk.END, "Goal Position: Not Set\n\n")

        if self.node.msg_lst:
            self.text_box.insert(tk.END, "Result Data (Last two messages):\n")

            # 최신 메시지 출력
            if len(self.node.msg_lst) >= 1:
                self.text_box.insert(tk.END, f"  Latest Message: {self.node.msg_lst[-1]}\n")

            # 그 이전 메시지 출력 (존재할 경우)
            if len(self.node.msg_lst) >= 2:
                self.text_box.insert(tk.END, f"  Previous Message: {self.node.msg_lst[-2]}\n")
        else:
            self.text_box.insert(tk.END, "Result message: Not Set\n")

        self.text_box.config(state=tk.DISABLED)  # 읽기 전용으로 설정
        self.root.after(500, self.update_text)

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