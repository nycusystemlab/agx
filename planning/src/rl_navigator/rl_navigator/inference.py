#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
import tf2_ros

# ROS 2 訊息格式
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

# Python 標準庫
import numpy as np
import math
import threading
from collections import deque
import sys
import os
import time
import yaml
import onnxruntime as ort

# 數學工具
from squaternion import Quaternion 
from filterpy.kalman import KalmanFilter 


# ====================================================================
# [路徑修正] 自動搜尋 ppo_nn.py
# 解決 ROS 2 colcon build 後找不到同目錄模組的問題
# ====================================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# =====================
# Constants
# =====================
V_MAX             = 1.0     
W_MAX             = 1.0
TIME_DELTA        = 0.1             
LIDAR_MAX_OBSDIS  = 10.0
FRONT_FOV_RAD     = math.pi       
ROBOT_RADIUS      = 0.6
ELEV_FOV_MIN      = -15 
ELEV_FOV_MAX      = 15  
ORIGINAL_SEGEMNTS = 16      
VERTICAL_LINES    = 180
Z_INGNORE         = 6       
HORIZONTAL        = ORIGINAL_SEGEMNTS - Z_INGNORE  
STATE_DIM         = 11 
# ====================================================================
# [主節點] RL Inference Node (ROS 2)
# ====================================================================
class RLInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')
        
        # 1. 設定路徑, 載入模型 (動態絕對路徑) 
        home_dir = os.path.expanduser("~")
        base_path = os.path.join(home_dir, "ros2_ws/src/models")
        onnx_path = os.path.join(base_path, "ugv_policy_actor.onnx")
        yaml_path = os.path.join(base_path, "policy_vecnormalize.yaml")
        
        # 2. 載入ONNX Session
        try:
            self.session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
            self.get_logger().info(f"✅ ONNX Model loaded from {onnx_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX: {e}")
            self.session = None

        # 3. 載入 YAML 標準化參數
        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)['vecnormalize']
            self.norm_mean = np.array(config['mean'], dtype=np.float32)
            self.norm_std = np.array(config['std'], dtype=np.float32)
            self.clip_obs = config.get('clip_obs', 10.0)
            self.get_logger().info(f"✅ YAML Normalization parameters (5411 dims) loaded.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.norm_mean = None

        # 2. 初始化模組
        self.lock = threading.Lock()
        self.raw = {"odom": None, "imu": None, "lidar": None, "mobile": None}        
        self.distance_map = np.full((VERTICAL_LINES, HORIZONTAL, 1), LIDAR_MAX_OBSDIS, np.float32)
        self.lidar_history = deque(maxlen=3)
        self.min_laser = 5.0 # 初始給較遠值
        for _ in range(3):
            self.lidar_history.append(np.zeros((VERTICAL_LINES, HORIZONTAL, 1), dtype=np.float32))

        # =========================================================
        # [FilterPy] 狀態估計
        # =========================================================
        self.kf = self._init_kalman()
        
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.odom_roll = self.odom_pitch = 0.0
        self.sensor_height = 0.0
        self.imu_xy = 0.0
        self.mobile = np.zeros(2, np.float32)
        
        """# 目標點 (測試用)"""
        self.stage_goal = [(3.0, -0.5, 0.04)] 

        # 3. ROS 2 通訊介面
        # Best Effort 用於感測器以降低延遲
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Odometry, '/odom', self._cb_odom, qos_reliable)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, qos_sensor)
        self.create_subscription(PointCloud2, '/velodyne_points', self._cb_lidar, qos_sensor)
        self.create_subscription(Odometry, '/mobile_odom', self._cb_mobile, qos_sensor)
        
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_reliable)
        self.goal_pub = self.create_publisher(MarkerArray, 'goal_marker', qos_reliable)

        # Main Loop Timer
        self.create_timer(TIME_DELTA, self.timer_process_cb)

        self.get_logger().info("RL Inference Node (Full Feature + FilterPy) Started!")

    def _init_kalman(self):
        kf = KalmanFilter(dim_x=6, dim_z=6)
        kf.x = np.zeros(6)
        kf.F = np.eye(6)
        kf.H = np.eye(6)
        kf.P *= 1.0
        kf.Q = np.eye(6) * 0.05
        kf.R = np.eye(6) * 0.8
        return kf
    # ==========================
    # Callbacks
    # ==========================
    def _cb_odom(self, msg):   
        with self.lock: self.raw["odom"] = msg
    def _cb_imu(self, msg):    
        with self.lock: self.raw["imu"] = msg
    def _cb_lidar(self, msg): 
        with self.lock: self.raw["lidar"] = msg
    def _cb_mobile(self, msg):
        with self.lock: self.raw["mobile"] = msg

    # ==========================
    # Main Loop
    # ==========================
    def timer_process_cb(self):
        with self.lock:
            od, imu, lid_msg = self.raw["odom"], self.raw["imu"], self.raw["lidar"]
            mob_msg = self.raw["mobile"]
        
        if not all([od, imu, lid_msg]):
            return

        # 1. IMU KF Update
        q = Quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
        roll, pitch, yaw = q.to_euler()
        
        z_meas = np.array([
            imu.linear_acceleration.z, -imu.linear_acceleration.x, -imu.linear_acceleration.y,
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z
        ])
        
        self.kf.predict()
        self.kf.update(z_meas)
        
        # 使用 filterpy 標準屬性 (.x)
        self.imu_xy = np.sqrt(self.kf.x[0]**2 + self.kf.x[1]**2)
        self.odom_roll, self.odom_pitch, self.odom_yaw = roll, -pitch, yaw
        self.odom_x, self.odom_y, self.sensor_height = od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z

        if mob_msg:
            self.mobile[0] = mob_msg.twist.twist.linear.x
            self.mobile[1] = mob_msg.twist.twist.angular.z

        # 2. LiDAR 處理 (Vectorized)
        self.process_lidar_and_track(lid_msg)

        # 3. 準備 Observation
        obs, dist_to_goal= self.get_observation()

        # A. 拼接成 5411 維 (與訓練時對齊)
        obs_combined = np.concatenate([obs["lidar"].flatten(), obs["state"].flatten()])

        # B. 標準化與裁切（使用YAML參數）
        obs_norm = np.clip((obs_combined - self.norm_mean) / (self.norm_std + 1e-8), -self.clip_obs, self.clip_obs)

        # C. 拆分回 ONNX 需要的輸入格式
        # 假設 ONNX 輸入名為 "lidar" 與 "state"
        lidar_input = obs_norm[:5400].reshape(1, 180, 10, 3).astype(np.float32)
        state_input = obs_norm[5400:].reshape(1, 11).astype(np.float32)

        # 4. ONNX 推論
        onnx_inputs = {"lidar": lidar_input, "state": state_input}
        action_outputs = self.session.run(None, onnx_inputs) # None 代表獲取所有輸出，或指定 ["action"]
        action = np.clip(action_outputs[0][0], -1.0, 1.0)

        # 5. 發佈速度指令
        cmd = Twist()
        cmd.linear.x = float(max(0, action[0] * 0.9))
        cmd.angular.z = float(np.clip(action[1], -W_MAX, W_MAX))

        if self.min_laser <= 0.2: # 安全停止距離
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        
        if dist_to_goal < 0.5:   # 到達目標點
            self.get_logger().info("🎯 Goal Reached!")
            cmd = Twist() # 停止機器人
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.vel_pub.publish(cmd)
            return # 跳過後續的模型推論

        self.vel_pub.publish(cmd)

    def process_lidar_and_track(self, lid_msg):
        # ROS 2 讀取點雲
        gen = pc2.read_points(lid_msg, skip_nans=True, field_names=("x", "y", "z"))
        # pts = np.array(list(gen), dtype=np.float32)
        

        pts_raw = np.array(list(gen))
        if pts.size == 0: return

        # pts = np.zeros((len(pts_raw), 3), dtype=np.float32)
        pts = np.empty((pts_raw.shape[0], 3), dtype=np.float32)
        pts[:, 0] = pts_raw['x']
        pts[:, 1] = pts_raw['y']
        pts[:, 2] = pts_raw['z']

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]


        # 向量化計算
        xy_sq = x**2 + y**2
        dists = np.sqrt(xy_sq + z**2)
        betas = np.arctan2(y, x)
        thetas = np.arctan2(z, np.sqrt(xy_sq))

        # FOV Filtering
        mask = (betas >= -FRONT_FOV_RAD/2) & (betas < FRONT_FOV_RAD/2) & (thetas > -3*np.pi/180)
        x, y, dists, betas, thetas = x[mask], y[mask], dists[mask], betas[mask], thetas[mask]
        if dists.size == 0: return

        # Grid Mapping
        j = ((betas + FRONT_FOV_RAD/2) * VERTICAL_LINES / FRONT_FOV_RAD).astype(np.int32)
        k = ((thetas - (ELEV_FOV_MIN*np.pi/180)) * ORIGINAL_SEGEMNTS / ((ELEV_FOV_MAX-ELEV_FOV_MIN)*np.pi/180)).astype(np.int32)
        j, k = np.clip(j, 0, VERTICAL_LINES-1), np.clip(k, 0, ORIGINAL_SEGEMNTS-1)

        d_clear = np.clip(dists - ROBOT_RADIUS, 0.01, LIDAR_MAX_OBSDIS)

        # Lexsort 優化
        idx_sort = np.lexsort((d_clear, k, j))
        j_s, k_s, d_s = j[idx_sort], k[idx_sort], d_clear[idx_sort]
        x_s, y_s = x[idx_sort], y[idx_sort]
        _, first_idx = np.unique(j_s * 100 + k_s, return_index=True)

        fmap = np.full((VERTICAL_LINES, ORIGINAL_SEGEMNTS, 1), LIDAR_MAX_OBSDIS, np.float32)
        # f_xy = np.zeros((VERTICAL_LINES, ORIGINAL_SEGEMNTS, 2), np.float32)
        
        fmap[j_s[first_idx], k_s[first_idx], 0] = d_s[first_idx]
        # f_xy[j_s[first_idx], k_s[first_idx], 0] = x_s[first_idx]
        # f_xy[j_s[first_idx], k_s[first_idx], 1] = y_s[first_idx]

        self.distance_map = fmap[:, Z_INGNORE:, :]
        self.lidar_history.append(self.distance_map.copy())
        self.min_laser = float(np.min(self.distance_map[:, :, 0]))

    def get_observation(self):
        skew_x = self.stage_goal[0][0] - self.odom_x
        skew_y = self.stage_goal[0][1] - self.odom_y
        x_local = skew_x * math.cos(-self.odom_yaw) - skew_y * math.sin(-self.odom_yaw)
        y_local = skew_x * math.sin(-self.odom_yaw) + skew_y * math.cos(-self.odom_yaw)
        
        dist_to_goal = float(math.hypot(x_local, y_local))
        beta = math.atan2(y_local, x_local)
        
        state = np.array([
            self.imu_xy, 
            self.kf.x[2], 
            self.kf.x[3], 
            self.kf.x[4], 
            self.odom_roll, 
            self.odom_pitch, 
            dist_to_goal,
            beta, 
            self.stage_goal[0][2] - self.sensor_height, 
            self.mobile[0], 
            self.mobile[1]
        ], dtype=np.float32)

        lidar_stack = np.concatenate(list(self.lidar_history), axis=2)
    
        return {
            "lidar": lidar_stack, 
            "state": state
        }, dist_to_goal

def main(args=None):
    rclpy.init(args=args)
    node = RLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()