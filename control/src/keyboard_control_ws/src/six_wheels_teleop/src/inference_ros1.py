#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
# from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2  # ROS 1 的 point_cloud2
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


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
from squaternion import Quaternion 

# ====================================================================
# [LiteKalmanFilter] 保持不變 (數學邏輯與 ROS 版本無關)
# ====================================================================
class LiteKalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.R = np.eye(dim_z)
        self.F = np.eye(dim_x)
        self.H = np.zeros((dim_z, dim_x))

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        try:
            K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            return
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.dim_x)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)


# =====================
# Constants
# =====================
V_MAX             = 1.0     
W_MAX             = 1.0
TIME_DELTA        = 0.05
LIDAR_MAX_OBSDIS  = 10.0
FRONT_FOV_RAD     = math.pi       
ROBOT_RADIUS      = 0.6
ELEV_FOV_MIN      = -15 
ELEV_FOV_MAX      = 15  
ORIGINAL_SEGEMNTS = 16      
VERTICAL_LINES    = 180
Z_INGNORE         = 6       
HORIZONTAL        = ORIGINAL_SEGEMNTS - Z_INGNORE  

# ====================================================================
# [主類別] RL Inference Node (ROS 1)
# ====================================================================
class RLInferenceNode:
    def __init__(self):
        # 1. ROS 1 初始化
        rospy.init_node('rl_inference_node', anonymous=False)
        
        # 2. 設定路徑, 載入模型
        home_dir = os.path.expanduser("~")
        # 注意：ROS1 workspace 路徑可能不同 (例如 catkin_ws)，請確認
        # /home/systemlab/test_RL_ws/src/adaptiveON/src/sim2real
        base_path = os.path.join(home_dir, "/root/keyboard_control_ws/src/models") 
        onnx_path = os.path.join(base_path, "ugv_policy_actor.onnx")
        yaml_path = os.path.join(base_path, "policy_vecnormalize.yaml")
        
        # 3. 載入 ONNX
        try:
            self.session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
            rospy.loginfo(f"✅ ONNX Model loaded from {onnx_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load ONNX: {e}")
            self.session = None

        # 4. 載入 YAML
        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)['vecnormalize']
            self.norm_mean = np.array(config['mean'], dtype=np.float32)
            self.norm_std = np.array(config['std'], dtype=np.float32)
            self.clip_obs = config.get('clip_obs', 10.0)
            rospy.loginfo(f"✅ YAML Normalization parameters loaded.")
        except Exception as e:
            rospy.logerr(f"Failed to load YAML: {e}")
            self.norm_mean = None

        # 5. 初始化變數
        self.lock = threading.Lock()
        self.raw = {"odom": None, "imu": None, "lidar": None, "mobile": None}        
        self.distance_map = np.full((VERTICAL_LINES, HORIZONTAL, 1), LIDAR_MAX_OBSDIS, np.float32)
        self.lidar_history = deque(maxlen=3)
        self.min_laser = 5.0
        for _ in range(3):
            self.lidar_history.append(np.zeros((VERTICAL_LINES, HORIZONTAL, 1), dtype=np.float32))
            
        # Filter
        self.kf = self._init_kalman()
        
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.odom_roll = self.odom_pitch = 0.0
        self.sensor_height = 0.0
        self.imu_xy = 0.0
        self.mobile = np.zeros(2)

        # Outdoor goal
        self.stage_goal = [(-2.0, -6.0, 0.04)] 

        # B1 gaol
        # self.stage_goal =[(5.0, 0, 0)]

        # === 推論計數器 ===
        self.inf_count = 0

        # 6. ROS 1 通訊介面 (移除 QoS, 加入 queue_size)
        rospy.Subscriber('/odom', Odometry, self._cb_odom, queue_size=1)
        rospy.Subscriber('/imu/data', Imu, self._cb_imu, queue_size=1)
        rospy.Subscriber('/velodyne_points', PointCloud2, self._cb_lidar, queue_size=1)
        rospy.Subscriber('rl_motion_array', Float32MultiArray, self._cb_mobile, queue_size=1)

        self.vel_pub = rospy.Publisher('mobile_cmd_vel', Twist, queue_size=10) # 模擬機器人速度 topic
        # self.goal_pub = rospy.Publisher('goal_marker', MarkerArray, queue_size=10)
        self.min_laser_pub = rospy.Publisher("/min_laser", Float32, queue_size=10) # 發布最短安全距離給紀錄程式

        self.yaw_rate =0.0

        # 7. Timer (ROS 1 Timer)
        rospy.Timer(rospy.Duration(TIME_DELTA), self.timer_process_cb)

        rospy.loginfo("RL Inference Node (ROS 1 + LiteKalman) Started!")

        self.goal_cloud_pub = rospy.Publisher(
        "/goal_point", PointCloud2, queue_size=1
        )

    def publish_goal_disk(self, radius=0.5, z_offset=0.3):
        gx, gy, gz = self.stage_goal[0]

        pts = []
        for r in np.linspace(0.0, radius, 30):        # ← 原本 10
            for th in np.linspace(0, 2*np.pi, 180):   # ← 原本 60
                x = gx + r * np.cos(th)
                y = gy + r * np.sin(th)
                z = gz + z_offset
                pts.append([x, y, z])

        header = rospy.Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()

        cloud = pc2.create_cloud_xyz32(header, pts)
        self.goal_cloud_pub.publish(cloud)

    def _init_kalman(self):
        kf = LiteKalmanFilter(dim_x=6, dim_z=6)
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
    def timer_process_cb(self, event): # ROS1 Timer callback 接受 event 參數
        t0 = time.time()
        with self.lock:
            od, imu, lid_msg = self.raw["odom"], self.raw["imu"], self.raw["lidar"]
            mob_msg = self.raw["mobile"]
            # print("ada")
        
        if not all([od, imu, lid_msg]):
            return

        if od is None:
            print("❌ odom missing")
            return
        if imu is None:
            print("❌ imu missing")
            return
        if lid_msg is None:
            print("❌ lidar missing")
            return

        self.publish_goal_disk(0.75)

        # 1. IMU KF Update
        q = Quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
        roll, pitch, yaw = q.to_euler()
        
        imu_acc_x = imu.linear_acceleration.z
        imu_acc_y = -imu.linear_acceleration.x
        imu_acc_z = -imu.linear_acceleration.y

        imu_roll_rate = imu.angular_velocity.z
        imu_pitch_rate = -imu.angular_velocity.x
        imu_yaw_rate = -imu.angular_velocity.y
        
        # z_meas = np.array([
        #     imu.linear_acceleration.z, -imu.linear_acceleration.x, -imu.linear_acceleration.y,
        #     imu.angular_velocity.x, -imu.angular_velocity.y, -imu.angular_velocity.z
        # ])

        z_meas = np.array([
            imu_acc_x, imu_acc_y, imu_acc_z,
            imu_roll_rate, imu_pitch_rate, imu_yaw_rate
        ])
        
        self.kf.predict()
        self.kf.update(z_meas)


        self.imu_xy = np.sqrt(self.kf.x[0]**2 + self.kf.x[1]**2)
        # self.odom_roll, self.odom_pitch= roll, -pitch 

        self.odom_roll, self.odom_pitch= pitch, roll+1.58
        # self.yaw_rate = imu.angular_velocity.z
        self.yaw_rate = imu_yaw_rate



        # print("ada")
        # 1. HDL odom Update
        self.odom_x, self.odom_y, self.sensor_height = od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z
        ori = od.pose.pose.orientation
        q_odom = Quaternion(ori.w, ori.x, ori.y, ori.z)
        _, _, odom_yaw = q_odom.to_euler()
        self.odom_yaw = odom_yaw  # 這是關鍵！必須用 odom 的 yaw 來轉相對座標

        if mob_msg:
            self.mobile[0] = mob_msg.data[0]
            self.mobile[1] = mob_msg.data[1]

        # 2. LiDAR 處理
        self.process_lidar_and_track(lid_msg)

        # === [修改開始] 每 10 次推論印出一次 Feature Map ===
        self.inf_count += 1
        if self.inf_count % 10 == 0:
            rospy.loginfo(f"\n========== Inference Step: {self.inf_count} ==========")
            
            # 設定 Numpy 顯示選項：小數點後2位，不使用科學記號，顯示完整陣列不省略(threshold=sys.maxsize)
            # 注意：這會印出非常長的數據
            # with np.printoptions(precision=2, suppress=True, threshold=sys.maxsize, linewidth=200):
            #     print("Feature Map Shape:", self.distance_map.shape)
            #     print("Min Dist:", np.min(self.distance_map), "Max Dist:", np.max(self.distance_map))
                
            #     # 這裡印出目前的 Feature Map (去除最後一個維度以便顯示)
            #     # self.distance_map 形狀是 (180, 10, 1)，我們印出 (180, 10)
            #     print(self.distance_map[:, :, 0]) 
                
                # 如果你想看包含歷史資訊的堆疊 (Stack) 資料，可以印出這個：
                # print("Stacked Observation (History):")
                # print(np.concatenate(list(self.lidar_history), axis=2))
        # === [修改結束] =====================================

        # 3. 準備 Observation
        obs, dist_to_goal = self.get_observation()
        
        if self.session is None or self.norm_mean is None:
            return

        # Inference 與之前相同
        obs_combined = np.concatenate([obs["lidar"].flatten(), obs["state"].flatten()])
        obs_norm = np.clip((obs_combined - self.norm_mean) / (self.norm_std + 1e-8), -self.clip_obs, self.clip_obs)

        lidar_input = obs_norm[:5400].reshape(1, 180, 10, 3).astype(np.float32)
        state_input = obs_norm[5400:].reshape(1, 11).astype(np.float32)

        onnx_inputs = {"lidar": lidar_input, "state": state_input}
        
        try:
            action_outputs = self.session.run(None, onnx_inputs)
            action = np.clip(action_outputs[0][0], -1.0, 1.0)
            print(f"Action: {action}, Dist to Goal: {dist_to_goal:.2f}, Min LiDAR: {self.min_laser:.2f}")
        except Exception as e:
            rospy.logerr(f"Inference Error: {e}")
            return

        cmd = Twist()
        cmd.linear.x = float(max(0, action[0] * 0.9))
        cmd.angular.z = float(np.clip(action[1], -W_MAX, W_MAX))
        # print(cmd)

        # 距離 1 公尺要減速(0.5m車體前半身)
        if self.min_laser <= 1.0: 
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        # 0.5
        if dist_to_goal < 0.75:   
            rospy.loginfo_throttle(2, "🎯 Goal Reached!") # throttle 避免洗頻
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.vel_pub.publish(cmd)

            # rospy.sleep(0.5)
            rospy.signal_shutdown("Goal Reached")
            sys.exit(0)
            return

        self.vel_pub.publish(cmd)
        
        t1 = time.time()
        # print("Inference time:", (t1 - t0)*1000, "ms")


    def process_lidar_and_track(self, lid_msg):
        # [ROS 1 修正] 使用標準 read_points 並轉為 NumPy
        # 注意: ROS 1 沒有 read_points_numpy，我們必須手動轉換
        
        # Generator 轉 List 再轉 NumPy (格式: N x 3)
        gen = pc2.read_points(lid_msg, skip_nans=True, field_names=("x", "y", "z"))
        pts_raw = np.array(list(gen), dtype=np.float32)

        if pts_raw.size == 0: return

        # 因為 pts_raw 是 (N, 3) 的矩陣，直接取 Column
        x = pts_raw[:, 0]
        y = pts_raw[:, 1]
        z = pts_raw[:, 2]

        # 向量化計算 (保持不變)
        xy_sq = x**2 + y**2
        dists = np.sqrt(xy_sq + z**2)
        betas = np.arctan2(y, x)
        thetas = np.arctan2(z, np.sqrt(xy_sq))

        mask = (betas >= -FRONT_FOV_RAD/2) & (betas < FRONT_FOV_RAD/2) & (thetas > -3*np.pi/180)
        x, y, dists, betas, thetas = x[mask], y[mask], dists[mask], betas[mask], thetas[mask]
        
        if dists.size == 0: return

        j = ((betas + FRONT_FOV_RAD/2) * VERTICAL_LINES / FRONT_FOV_RAD).astype(np.int32)
        k = ((thetas - (ELEV_FOV_MIN*np.pi/180)) * ORIGINAL_SEGEMNTS / ((ELEV_FOV_MAX-ELEV_FOV_MIN)*np.pi/180)).astype(np.int32)
        j, k = np.clip(j, 0, VERTICAL_LINES-1), np.clip(k, 0, ORIGINAL_SEGEMNTS-1)

        d_clear = np.clip(dists - ROBOT_RADIUS, 0.01, LIDAR_MAX_OBSDIS)

        idx_sort = np.lexsort((d_clear, k, j))
        j_s, k_s, d_s = j[idx_sort], k[idx_sort], d_clear[idx_sort]
        _, first_idx = np.unique(j_s * 100 + k_s, return_index=True)

        fmap = np.full((VERTICAL_LINES, ORIGINAL_SEGEMNTS, 1), LIDAR_MAX_OBSDIS, np.float32)
        fmap[j_s[first_idx], k_s[first_idx], 0] = d_s[first_idx]

        self.distance_map = fmap[:, Z_INGNORE:, :]
        self.lidar_history.append(self.distance_map.copy())
        self.min_laser = float(np.min(self.distance_map[:, :, 0]))

        # front = self.distance_map[60:120, :, 0] # 

        self.min_laser_pub.publish(Float32(self.min_laser))

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
            self.yaw_rate

        ], dtype=np.float32)

        # print(f"beta: {beta:.2f}")
        # print(f"roll:{self.odom_roll:.2f}, pitch:{self.odom_pitch:.2f}, dist={dist_to_goal:.2f}, beta: {beta:.2f}, ")
        print(state)
        lidar_stack = np.concatenate(list(self.lidar_history), axis=2)
        
        return {
            "lidar": lidar_stack, 
            "state": state
        }, dist_to_goal

if __name__ == '__main__':
    try:
        node = RLInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass