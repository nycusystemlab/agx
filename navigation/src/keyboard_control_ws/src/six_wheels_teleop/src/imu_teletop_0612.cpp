#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <fstream>
#include <set>
#include <vector>
#include <numeric>
#include <tf/tf.h>                    
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h" 

#define KEY_W 'w'
#define KEY_S 's'
#define KEY_A 'a'
#define KEY_D 'd'
#define KEY_Q 'q'
#define KEY_E 'e'
#define KEY_X 'x'

#define KEY_1 '1'
#define KEY_2 '2'
#define KEY_3 '3'
#define KEY_4 '4'
#define KEY_5 '5'
#define KEY_6 '6'
#define KEY_7 '7'
#define KEY_8 '8'
#define KEY_9 '9'

class TeleopKeyboard {
public:
    TeleopKeyboard();
    ~TeleopKeyboard();

private:
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void keyboardCallback(const ros::TimerEvent&);
    void saveImuAndMotionData();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher imu_float_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber motion_array_sub_;
    ros::Timer keyboard_timer_;
    std_msgs::UInt8 cmd_;
    std::string filename_;

    // -------- 線性加速度卡爾曼濾波變數 --------
    bool kf_initialized_x_;      // 是否初始化 X 軸線性加速度濾波
    float x_kf_acc_x_;              // X 軸濾波後的線性加速度估計值 (m/s^2)
    float P_kf_x_;              // X 軸濾波的協方差 (不確定性)
    bool kf_initialized_y_;      // 是否初始化 Y 軸線性加速度濾波
    float x_kf_acc_y_;              // Y 軸濾波後的線性加速度估計值 (m/s^2)
    float P_kf_y_;              // Y 軸濾波的協方差
    bool kf_initialized_z_;      // 是否初始化 Z 軸線性加速度濾波
    float x_kf_acc_z_;              // Z 軸濾波後的線性加速度估計值 (m/s^2)
    float P_kf_z_;              // Z 軸濾波的協方差

    // -------- 角速度卡爾曼濾波變數 --------
    bool kf_initialized_roll_;   // 是否初始化 Roll 軸角速度濾波
    float x_kf_roll_rate;           // Roll 軸濾波後的角速度估計值 (rad/s)
    float P_kf_roll_;           // Roll 軸濾波的協方差
    bool kf_initialized_pitch_;  // 是否初始化 Pitch 軸角速度濾波
    float x_kf_pitch_rate;          // Pitch 軸濾波後的角速度估計值 (rad/s)
    float P_kf_pitch_;          // Pitch 軸濾波的協方差
    bool kf_initialized_yaw_;   // 是否初始化 Yaw 軸角速度濾波
    float x_kf_yaw_rate;            // Yaw 軸濾波後的角速度估計值 (rad/s)
    float P_kf_yaw_;            // Yaw 軸濾波的協方差
    bool kf_initialized_omega_fused_; // 是否初始化融合角速度濾波
    float x_kf_omega_fused_;    // 融合角速度濾波後的估計值 (rad/s)
    float P_kf_omega_fused_;    // 融合角速度濾波的協方差

    // -------- 卡爾曼濾波參數 --------
    const float Q_kf_linear_;   // 線性加速度的過程雜訊方差，表示模型對動態變化的信任度
    const float R_kf_linear_;   // 線性加速度的測量雜訊方差，表示傳感器測量的不確定性
    const float Q_kf_angular_;  // 角速度的過程雜訊方差，表示模型對角速度變化的信任度
    const float R_kf_angular_;  // 角速度的測量雜訊方差，表示角速度測量的不確定性

    // -------- IMU 數據 --------
    double imu_acc_x_, imu_acc_y_, imu_acc_z_; // 原始線性加速度 (m/s^2)
    double imu_roll_rate, imu_pitch_rate, imu_yaw_rate; // 原始角速度 (rad/s)
    
    // -------- 當前位置和姿態 --------
    double current_pose_x = 0.0; // 當前 X 位置 (m)
    double current_pose_y = 0.0; // 當前 Y 位置 (m)
    double current_pose_z = 0.0; // 當前 Z 位置 (m)
    double pose_roll = 0.0, pose_pitch = 0.0, pose_yaw = 0.0; // 姿態角 (rad)

    // -------- Motion array 數據 --------
    double motion_x_, motion_y_, motion_th_; // 運動學位置和方向
    double motion_v_real_, motion_omega_fused_; // 真實線速度和融合角速度
    double motion_a_real_, vl_real, vr_real, comp_L, comp_R, LLPWM, RRPWM; // 運動控制數據
    double differential_angular_v; // 差分角速度
    double low_filter_imu_acc_x, low_filter_imu_acc_y; // 低通濾波後的加速度

    // -------- 角加速度和時間戳 --------
    float angular_acceleration_roll_ = 0.0f;  // Roll 軸角加速度 (rad/s^2)
    float angular_acceleration_pitch_ = 0.0f; // Pitch 軸角加速度 (rad/s^2)
    float angular_acceleration_yaw_ = 0.0f;   // Yaw 軸角加速度 (rad/s^2)
    float angular_acceleration_fused_ = 0.0f; // 融合角加速度 (rad/s^2)
    float pre_x_kf_roll_rate = 0.0f;              // 前一時刻濾波後的 Roll 角速度
    float pre_x_kf_pitch_rate = 0.0f;             // 前一時刻濾波後的 Pitch 角速度
    float pre_x_kf_yaw_rate = 0.0f;               // 前一時刻濾波後的 Yaw 角速度
    float pre_x_kf_omega_fused_ = 0.0f;       // 前一時刻濾波後的融合角速度
    ros::Time last_imu_time_;                 // 上一 IMU 消息的時間戳
    ros::Time last_motion_time_;              // 上一 Motion array 消息的時間戳
    float last_imu_dt_ = 0.0f;                // IMU 消息的時間間隔 (s)
    float last_motion_dt_ = 0.0f;             // Motion array 消息的時間間隔 (s)

    struct termios raw_, cooked_;
    static int key_counter_;
    std::set<char> valid_keys_;
    static bool header_written_;
};

int TeleopKeyboard::key_counter_ = 0;
bool TeleopKeyboard::header_written_ = false;

TeleopKeyboard::TeleopKeyboard() : 
    private_nh_("~"), 
    kf_initialized_x_(false),
    kf_initialized_y_(false),
    kf_initialized_z_(false),
    x_kf_acc_x_(0.0f),
    P_kf_x_(1.0f),
    x_kf_acc_y_(0.0f),
    P_kf_y_(1.0f),
    x_kf_acc_z_(0.0f),
    P_kf_z_(1.0f),
    kf_initialized_roll_(false),
    x_kf_roll_rate(0.0f),
    P_kf_roll_(1.0f),
    kf_initialized_pitch_(false),
    x_kf_pitch_rate(0.0f),
    P_kf_pitch_(1.0f),
    kf_initialized_yaw_(false),
    x_kf_yaw_rate(0.0f),
    P_kf_yaw_(1.0f),
    kf_initialized_omega_fused_(false),
    x_kf_omega_fused_(0.0f),
    P_kf_omega_fused_(1.0f),
    Q_kf_linear_(0.0001f),  // 線性加速度過程雜訊方差，較小值表示信任模型動態
    R_kf_linear_(0.01f),    // 線性加速度測量雜訊方差，根據 IMU 雜訊特性設定
    Q_kf_angular_(0.00000003f), // 角速度過程雜訊方差，稍大以適應角速度的快速變化
    R_kf_angular_(0.000008f),   // 角速度測量雜訊方差，角速度通常比線性加速度雜訊更大
    last_imu_time_(0),
    last_motion_time_(0)
{
    private_nh_.param<std::string>("save_filename", filename_, "/home/systemlabagx/disk/0616_test.csv");
    cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("processed_imu", 10);
    imu_float_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("processed_imu_float", 10);
    imu_sub_ = nh_.subscribe("/imu/data", 100, &TeleopKeyboard::imuCallback, this); // 增大隊列以應對高負載
    motion_array_sub_ = nh_.subscribe("motion_array", 100, &TeleopKeyboard::motionArrayCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &TeleopKeyboard::odomCallback, this);

    keyboard_timer_ = nh_.createTimer(ros::Duration(0.1), &TeleopKeyboard::keyboardCallback, this);
    tcgetattr(STDIN_FILENO, &cooked_);
    raw_ = cooked_;
    raw_.c_lflag &= ~(ICANON | ECHO);
    raw_.c_cc[VMIN] = 0;
    raw_.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
    valid_keys_ = {KEY_W, KEY_S, KEY_A, KEY_D, KEY_Q, KEY_E, KEY_X, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9};

    motion_x_ = motion_y_ = motion_th_ = 0.0;
    motion_v_real_ = motion_omega_fused_ = motion_a_real_ = 0.0;
    vl_real = vr_real = 0.0;

    std::ofstream ofs(filename_.c_str(), std::ios::app);
    if (ofs.is_open()) {
        ofs.seekp(0, std::ios::end);
        if (ofs.tellp() == 0) {
            ofs << "timestamp,"
                << "imu_x,imu_y,imu_z,"
                << "imu_filtered_x,imu_filtered_y,imu_filtered_z,"
                << "imu_roll,imu_pitch,imu_yaw,"
                << "motion_x,motion_y,motion_th,"
                << "motion_v_real,"
                << "motion_a_real,vl_real,vr_real,comp_L,comp_R,LLPWM,RRPWM,"
                << "omega_a_real,omega_a_diff,differential_angular_v,low_filter_imu_acc_x,low_filter_imu_acc_y,"
                << "filtered_roll_rate,filtered_pitch_rate,filtered_yaw_rate,filtered_omega_fused,"
                // << "angular_acceleration_roll,angular_acceleration_pitch,angular_acceleration_yaw,"
                << "angular_acceleration_fused,"
                << "imu_dt,motion_dt,"
                << "current_pose_x,current_pose_y,current_pose_z,pose_roll,pose_pitch,pose_yaw"
                << std::endl;
            header_written_ = true;
        }
        ofs.close();
    } else {
        ROS_WARN("無法打開檔案以檢查標題: %s", filename_.c_str());
    }
}

TeleopKeyboard::~TeleopKeyboard() {
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked_);
}

void TeleopKeyboard::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    current_pose_x = msg->pose.pose.position.x;
    current_pose_y = msg->pose.pose.position.y;
    current_pose_z = msg->pose.pose.position.z;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(pose_roll, pose_pitch, pose_yaw);
}

float pre_imu_z = 0, diff_imu_z = 0;
void TeleopKeyboard::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    ros::Rate loop_rate(20);
    ROS_DEBUG("收到 IMU 數據: accel_z=%f, yaw=%f", 
              msg->linear_acceleration.z, msg->angular_velocity.z);

    imu_acc_x_ = msg->linear_acceleration.z;
    imu_acc_y_ = (msg->linear_acceleration.x) * -1;
    imu_acc_z_ = (msg->linear_acceleration.y) * -1;
    imu_roll_rate = msg->angular_velocity.z;
    imu_pitch_rate = (msg->angular_velocity.x) * -1;
    imu_yaw_rate = (msg->angular_velocity.y) * -1;

    diff_imu_z = imu_acc_z_ - pre_imu_z;

    // -------- 線性加速度卡爾曼濾波 --------
    // X 軸線性加速度濾波
    if (!kf_initialized_x_) {
        x_kf_acc_x_ = static_cast<float>(imu_acc_x_); // 初始化為首次測量值
        P_kf_x_ = 1.0f;                          // 初始化協方差
        kf_initialized_x_ = true;
    } else {
        // 預測步驟
        float x_pred = x_kf_acc_x_;                   // 預測狀態（假設無控制輸入）
        float P_pred = P_kf_x_ + Q_kf_linear_;    // 預測協方差，加入過程雜訊
        // 測量更新
        float z = static_cast<float>(imu_acc_x_); // 當前測量值
        float K = P_pred / (P_pred + R_kf_linear_); // 卡爾曼增益
        x_kf_acc_x_ = x_pred + K * (z - x_pred);      // 更新狀態估計
        P_kf_x_ = (1.0f - K) * P_pred;            // 更新協方差
    }

    // Y 軸線性加速度濾波
    if (!kf_initialized_y_) {
        x_kf_acc_y_ = static_cast<float>(imu_acc_y_);
        P_kf_y_ = 1.0f;
        kf_initialized_y_ = true;
    } else {
        float x_pred = x_kf_acc_y_;
        float P_pred = P_kf_y_ + Q_kf_linear_;
        float z = static_cast<float>(imu_acc_y_);
        float K = P_pred / (P_pred + R_kf_linear_);
        x_kf_acc_y_ = x_pred + K * (z - x_pred);
        P_kf_y_ = (1.0f - K) * P_pred;
    }

    // Z 軸線性加速度濾波
    if (!kf_initialized_z_) {
        x_kf_acc_z_ = static_cast<float>(imu_acc_z_);
        P_kf_z_ = 1.0f;
        kf_initialized_z_ = true;
    } else {
        float x_pred = x_kf_acc_z_;
        float P_pred = P_kf_z_ + Q_kf_linear_;
        float z = static_cast<float>(imu_acc_z_);
        float K = P_pred / (P_pred + R_kf_linear_);
        x_kf_acc_z_ = x_pred + K * (z - x_pred);
        P_kf_z_ = (1.0f - K) * P_pred;
    }

    // -------- 角速度卡爾曼濾波 --------
    // Roll 軸角速度濾波
    if (!kf_initialized_roll_) {
        x_kf_roll_rate = static_cast<float>(imu_roll_rate); // 初始化為首次測量值
        P_kf_roll_ = 1.0f;                             // 初始化協方差
        kf_initialized_roll_ = true;
        pre_x_kf_roll_rate = x_kf_roll_rate;                   // 初始化前一濾波值
    } else {
        // 預測步驟
        float x_pred = x_kf_roll_rate;
        float P_pred = P_kf_roll_ + Q_kf_angular_;     // 使用角速度過程雜訊
        // 測量更新
        float z = static_cast<float>(imu_roll_rate);
        float K = P_pred / (P_pred + R_kf_angular_);   // 使用角速度測量雜訊
        x_kf_roll_rate = x_pred + K * (z - x_pred);
        P_kf_roll_ = (1.0f - K) * P_pred;
    }

    // Pitch 軸角速度濾波
    if (!kf_initialized_pitch_) {
        x_kf_pitch_rate = static_cast<float>(imu_pitch_rate);
        P_kf_pitch_ = 1.0f;
        kf_initialized_pitch_ = true;
        pre_x_kf_pitch_rate = x_kf_pitch_rate;
    } else {
        float x_pred = x_kf_pitch_rate;
        float P_pred = P_kf_pitch_ + Q_kf_angular_;
        float z = static_cast<float>(imu_pitch_rate);
        float K = P_pred / (P_pred + R_kf_angular_);
        x_kf_pitch_rate = x_pred + K * (z - x_pred);
        P_kf_pitch_ = (1.0f - K) * P_pred;
    }

    // Yaw 軸角速度濾波
    if (!kf_initialized_yaw_) {
        x_kf_yaw_rate = static_cast<float>(imu_yaw_rate);
        P_kf_yaw_ = 1.0f;
        kf_initialized_yaw_ = true;
        pre_x_kf_yaw_rate = x_kf_yaw_rate;
    } else {
        float x_pred = x_kf_yaw_rate;
        float P_pred = P_kf_yaw_ + Q_kf_angular_;
        float z = static_cast<float>(imu_yaw_rate);
        float K = P_pred / (P_pred + R_kf_angular_);
        x_kf_yaw_rate = x_pred + K * (z - x_pred);
        P_kf_yaw_ = (1.0f - K) * P_pred;
    }

    // -------- 計算角加速度 --------
    ros::Time current_time = msg->header.stamp;
    if (!last_imu_time_.isZero()) {
        float dt = (current_time - last_imu_time_).toSec();
        if (dt > 0.0f && dt < 0.2f) { // 檢查 dt 合理性（200ms 對應 5Hz 以下）
            angular_acceleration_roll_ = (x_kf_roll_rate - pre_x_kf_roll_rate) / dt;
            angular_acceleration_pitch_ = (x_kf_pitch_rate - pre_x_kf_pitch_rate) / dt;
            angular_acceleration_yaw_ = (x_kf_yaw_rate - pre_x_kf_yaw_rate) / dt;
            last_imu_dt_ = dt;
        } else {
            ROS_WARN("IMU dt abnormal: %f seconds, skipping angular acceleration", dt);
            angular_acceleration_roll_ = 0.0f;
            angular_acceleration_pitch_ = 0.0f;
            angular_acceleration_yaw_ = 0.0f;
        }
    }
    pre_x_kf_roll_rate = x_kf_roll_rate;
    pre_x_kf_pitch_rate = x_kf_pitch_rate;
    pre_x_kf_yaw_rate = x_kf_yaw_rate;
    last_imu_time_ = current_time;

    // 發布濾波後的 IMU 數據
    std_msgs::Float32MultiArray float_msg;
    // float_msg.data = {static_cast<float>(x_kf_acc_x_),
    //                   static_cast<float>(x_kf_acc_y_),
    //                   static_cast<float>(x_kf_acc_z_),
    //                   static_cast<float>(x_kf_roll_rate),
    //                   static_cast<float>(x_kf_pitch_rate),
    //                   static_cast<float>(x_kf_yaw_rate)};
    float_msg.data = {static_cast<float>(angular_acceleration_roll_),
                      static_cast<float>(angular_acceleration_pitch_),
                      static_cast<float>(angular_acceleration_yaw_),
                      static_cast<float>(x_kf_roll_rate),
                      static_cast<float>(x_kf_pitch_rate),
                      static_cast<float>(x_kf_yaw_rate)};
    imu_float_pub_.publish(float_msg);
    pre_imu_z = imu_acc_z_;
    loop_rate.sleep();
}

void TeleopKeyboard::motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg) {
    if (msg->data.size() == 15) {
        motion_x_ = msg->data[0];
        motion_y_ = msg->data[1];
        motion_th_ = msg->data[2];
        motion_v_real_ = msg->data[3];
        motion_omega_fused_ = msg->data[4];
        motion_a_real_ = msg->data[5];   
        vl_real = msg->data[6]; 
        vr_real = msg->data[7];
        comp_L = msg->data[8];  
        comp_R = msg->data[9];
        LLPWM = msg->data[10];  
        RRPWM = msg->data[11];
        differential_angular_v = msg->data[12];
        low_filter_imu_acc_x = msg->data[13];
        low_filter_imu_acc_y = msg->data[14];

        // 融合角速度卡爾曼濾波
        if (!kf_initialized_omega_fused_) {
            x_kf_omega_fused_ = static_cast<float>(motion_omega_fused_); // 初始化為首次測量值
            P_kf_omega_fused_ = 1.0f;                                   // 初始化協方差
            kf_initialized_omega_fused_ = true;
            pre_x_kf_omega_fused_ = x_kf_omega_fused_;
        } else {
            // 預測步驟
            float x_pred = x_kf_omega_fused_;
            float P_pred = P_kf_omega_fused_ + Q_kf_angular_;
            // 測量更新
            float z = static_cast<float>(motion_omega_fused_);
            float K = P_pred / (P_pred + R_kf_angular_);
            x_kf_omega_fused_ = x_pred + K * (z - x_pred);
            P_kf_omega_fused_ = (1.0f - K) * P_pred;
        }

        // 計算融合角加速度
        ros::Time current_time = ros::Time::now();
        if (!last_motion_time_.isZero()) {
            float dt = (current_time - last_motion_time_).toSec();
            if (dt > 0.0f && dt < 0.2f) {
                angular_acceleration_fused_ = (x_kf_omega_fused_ - pre_x_kf_omega_fused_) / dt;
                last_motion_dt_ = dt;
            } else {
                ROS_WARN("Motion array dt abnormal: %f seconds, skipping angular acceleration", dt);
                angular_acceleration_fused_ = 0.0f;
            }
        }
        pre_x_kf_omega_fused_ = x_kf_omega_fused_;
        last_motion_time_ = current_time;

        ROS_DEBUG("收到 motion_array 數據: x=%f, y=%f, th=%f, v=%f, omega=%f, a=%f, vl_real=%f, vr_real=%f, comp_L=%f, comp_R=%f, LLPWM=%f, RRPWM=%f, differential_angular_v=%f",
                  motion_x_, motion_y_, motion_th_, motion_v_real_, motion_omega_fused_,
                  motion_a_real_, vl_real, vr_real, comp_L, comp_R, LLPWM, RRPWM, differential_angular_v);
    } else {
        ROS_WARN("motion_array 數據長度錯誤: %zu", msg->data.size());
    }
    saveImuAndMotionData();
}

void TeleopKeyboard::keyboardCallback(const ros::TimerEvent&) {
    struct pollfd fd = {STDIN_FILENO, POLLIN, 0};
    int ret = poll(&fd, 1, 10);
    ROS_DEBUG("Poll return: %d", ret);
    if (ret > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            ROS_INFO("Pressed key: %c", c);
            if (valid_keys_.find(c) != valid_keys_.end()) {
                switch (c) {
                    case KEY_W: cmd_.data = 1; break;
                    case KEY_S: cmd_.data = 5; break;
                    case KEY_A: cmd_.data = 3; break;
                    case KEY_D: cmd_.data = 4; break;
                    case KEY_X: cmd_.data = 2; break;
                    case KEY_Q: cmd_.data = 6; break;
                    case KEY_E: cmd_.data = 7; break;
                    case KEY_1: cmd_.data = 8; break;
                    case KEY_2: cmd_.data = 9; break;
                    case KEY_3: cmd_.data = 10; break;
                    case KEY_4: cmd_.data = 11; break;
                    case KEY_5: cmd_.data = 12; break;
                    case KEY_6: cmd_.data = 13; break;
                    case KEY_7: cmd_.data = 14; break;
                    case KEY_8: cmd_.data = 15; break;
                    case KEY_9: cmd_.data = 16; break;
                    default: return;
                }
                cmd_pub_.publish(cmd_);
                ROS_INFO("Published cmd_vel: %d", cmd_.data);
            } else {
                ROS_WARN("Invalid key: %c", c);
            }
        } else {
            ROS_DEBUG("Read failed or no data");
        }
    }
}

void TeleopKeyboard::saveImuAndMotionData() {
    std::ofstream ofs(filename_.c_str(), std::ios::app);
    if (ofs.is_open()) {
        if (!header_written_) {
            ofs << "timestamp,"
                << "imu_x,imu_y,imu_z,"
                << "imu_filtered_x,imu_filtered_y,imu_filtered_z,"
                << "imu_roll,imu_pitch,imu_yaw,"
                << "motion_x,motion_y,motion_th,"
                << "motion_v_real,"
                << "motion_a_real,vl_real,vr_real,comp_L,comp_R,LLPWM,RRPWM,"
                << "omega_a_real,omega_a_diff,differential_angular_v,low_filter_imu_acc_x,low_filter_imu_acc_y,"
                << "filtered_roll_rate,filtered_pitch_rate,filtered_yaw_rate,filtered_omega_fused,"
                // << "angular_acceleration_roll,angular_acceleration_pitch,angular_acceleration_yaw,"
                << "angular_acceleration_fused,"
                << "imu_dt,motion_dt,"
                << "current_pose_x,current_pose_y,current_pose_z,pose_roll,pose_pitch,pose_yaw"
                << std::endl;
            header_written_ = true;
        }
        ros::Time current_time = ros::Time::now();
        double timestamp = current_time.toSec();
        ofs << std::fixed << std::setprecision(3)
            << timestamp << ","
            << imu_acc_x_ << "," << imu_acc_y_ << "," << imu_acc_z_ << ","
            << x_kf_acc_x_ << "," << x_kf_acc_y_ << "," << x_kf_acc_z_ << ","
            << imu_roll_rate << "," << imu_pitch_rate << "," << imu_yaw_rate << ","
            << motion_x_ << "," << motion_y_ << "," << motion_th_ << ","
            << motion_v_real_ << "," 
            << motion_a_real_ << "," << vl_real << "," << vr_real << ","
            << comp_L << "," << comp_R << "," << LLPWM << "," << RRPWM << ","
            << motion_v_real_ * x_kf_yaw_rate << "," << diff_imu_z/0.05 << "," 
            << differential_angular_v << ","
            << low_filter_imu_acc_x << "," << low_filter_imu_acc_y << ","
            << x_kf_roll_rate << "," << x_kf_pitch_rate << "," << x_kf_yaw_rate << "," << x_kf_omega_fused_ << ","
            // << angular_acceleration_roll_ << "," << angular_acceleration_pitch_ << "," << angular_acceleration_yaw_ << "," 
            << angular_acceleration_fused_ << ","
            << last_imu_dt_ << "," << last_motion_dt_ << ","
            << current_pose_x << "," << current_pose_y << "," << current_pose_z << "," 
            << pose_roll << "," << pose_pitch << "," << pose_yaw       
            << std::endl;
        ofs.close();
    } else {
        ROS_WARN("無法打開檔案: %s", filename_.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_teleop");
    TeleopKeyboard teleop;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
