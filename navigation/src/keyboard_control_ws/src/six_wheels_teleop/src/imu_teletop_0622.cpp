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
#include <cmath> // 為 std::isfinite 和 std::sin/max/min
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <map>

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

// 卡爾曼濾波器類
class KalmanFilter {
public:
    KalmanFilter(float Q = 1e-3f, float R = 1e-2f) 
      : Q_(Q), R_(R), 
        P_(1.0f), x_(0.0f), initialized_(false)
    {}
    
    // KalmanFilter(float Q, float R, float initial_P = 1.0f)
    //     : Q_(Q), R_(R), P_(initial_P), x_(0.0f), initialized_(false) {}

    void init(float initial_x) {
        x_ = initial_x;
        P_ = 1.0f;
        initialized_ = true;
    }

    float update(float z) {
        if (!std::isfinite(z)) {
            ROS_WARN("Invalid measurement: %f, skipping KF update", z);
            return x_;
        }
        if (!initialized_) {
            init(z);
            return x_;
        }

        // 預測步驟
        float x_pred = x_;
        float P_pred = P_ + Q_;

        // 更新步驟
        float K = P_pred / (P_pred + R_);
        x_ = x_pred + K * (z - x_pred);
        P_ = (1.0f - K) * P_pred;

        ROS_DEBUG("KF update: z=%f, x=%f, P=%f", z, x_, P_);
        return x_;
    }

    float getState() const { return x_; }
    float getCovariance() const { return P_; }

private:
    float Q_;           // 過程雜訊方差
    float R_;           // 測量雜訊方差
    float P_;           // 協方差
    float x_;           // 狀態估計
    bool initialized_;  // 是否初始化
};

class TeleopKeyboard {
public:
    TeleopKeyboard();
    ~TeleopKeyboard();

private:
    enum class FilterType { ACC_X, ACC_Y, ACC_Z, ROLL, PITCH, YAW, MOTION_A, OMEGA_FUSED };

    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void keyboardCallback(const ros::TimerEvent&);
    void distanceTimerCallback(const ros::TimerEvent&); // 新增：定時器回調
    void angularAccTimerCallback(const ros::TimerEvent&);
    void saveImuAndMotionData();
    float computeAngularAcceleration(float current_rate, float prev_rate, ros::Time current_time, ros::Time& last_time, float& last_dt);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher imu_float_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber motion_array_sub_;
    ros::Timer keyboard_timer_;
    ros::Timer distance_timer_;
    ros::Time last_roll_time_, last_pitch_time_, last_yaw_time_;    std_msgs::UInt8 cmd_;
    std::string filename_;
    float last_roll_dt_, last_pitch_dt_, last_yaw_dt_;
    // 卡爾曼濾波器
    std::map<FilterType, KalmanFilter> filters_;

    // 濾波參數（從 ROS 參數伺服器載入）
    float Q_kf_linear_, R_kf_linear_;
    float Q_kf_angular_, R_kf_angular_;
    float Q_kf_motion_a_, R_kf_motion_a_;

    // IMU 數據
    double imu_acc_x_, imu_acc_y_, imu_acc_z_;
    double imu_roll_rate, imu_pitch_rate, imu_yaw_rate;

    // 濾波後的數據
    float x_kf_acc_x_, x_kf_acc_y_, x_kf_acc_z_;
    float x_kf_roll_rate, x_kf_pitch_rate, x_kf_yaw_rate;
    float x_kf_motion_a_, x_kf_omega_fused_;

    // 當前位置和姿態
    double current_pose_x = 0.0;
    double current_pose_y = 0.0;
    double current_pose_z = 0.0;
    double pose_roll = 0.0, pose_pitch = 0.0, pose_yaw = 0.0;

    // Motion array 數據
    double motion_x_, motion_y_, motion_th_;
    double motion_v_real_, motion_omega_fused_, motion_v_real_kf;
    double motion_a_real_, vl_real, vr_real, comp_L, comp_R, LLPWM, RRPWM;
    double differential_angular_v;
    double low_filter_imu_acc_x, low_filter_imu_acc_y;

    // 角加速度和時間戳
    float angular_acceleration_roll_ = 0.0f;
    float angular_acceleration_pitch_ = 0.0f;
    float angular_acceleration_yaw_ = 0.0f;
    float angular_acceleration_fused_ = 0.0f;
    float pre_x_kf_roll_rate = 0.0f;
    float pre_x_kf_pitch_rate = 0.0f;
    float pre_x_kf_yaw_rate = 0.0f;
    float pre_x_kf_omega_fused_ = 0.0f;
    ros::Time last_imu_time_;
    ros::Time last_motion_time_;
    ros::Time last_angular_acc_time_;  
    ros::Time angular_acc_timer_;

    float     last_angular_acc_dt_ = 0; 

    float last_imu_dt_ = 0.0f;
    float last_motion_dt_ = 0.0f;

    float Power_regression = 0.0f;
    float Power_regression_test = 0.0f;
    struct termios raw_, cooked_;
    static int key_counter_;
    std::set<char> valid_keys_;
    static bool header_written_;
    float pre_imu_z = 0.0f, diff_imu_z = 0.0f;

    // 新增：里程計算相關變數
    double last_pose_x_ = 0.0, last_pose_y_ = 0.0; // 上一次位姿
    double total_distance_ = 0.0; // 累積里程距離
    bool first_odom_received_ = false; // 是否收到首次 Odometry 數據
};

int TeleopKeyboard::key_counter_ = 0;
bool TeleopKeyboard::header_written_ = false;

TeleopKeyboard::TeleopKeyboard() : private_nh_("~"), last_imu_time_(0), last_motion_time_(0) {
    // 從 ROS 參數伺服器載入濾波參數
    private_nh_.param("kf/Q_linear", Q_kf_linear_, 0.0001f);
    private_nh_.param("kf/R_linear", R_kf_linear_, 0.01f);
    private_nh_.param("kf/Q_angular", Q_kf_angular_, 0.0005f);
    private_nh_.param("kf/R_angular", R_kf_angular_, 0.000008f);
    private_nh_.param("kf/Q_motion_a", Q_kf_motion_a_, 0.1f);
    private_nh_.param("kf/R_motion_a", R_kf_motion_a_, 0.75f);

    // 初始化卡爾曼濾波器
    // filters_ = {
    //     {FilterType::ACC_X, KalmanFilter(Q_kf_linear_, R_kf_linear_)},
    //     {FilterType::ACC_Y, KalmanFilter(Q_kf_linear_, R_kf_linear_)},
    //     {FilterType::ACC_Z, KalmanFilter(Q_kf_linear_, R_kf_linear_)},
    //     {FilterType::ROLL, KalmanFilter(Q_kf_angular_, R_kf_angular_)},
    //     {FilterType::PITCH, KalmanFilter(Q_kf_angular_, R_kf_angular_)},
    //     {FilterType::YAW, KalmanFilter(Q_kf_angular_, R_kf_angular_)},
    //     {FilterType::MOTION_A, KalmanFilter(Q_kf_motion_a_, R_kf_motion_a_)},
    //     {FilterType::OMEGA_FUSED, KalmanFilter(Q_kf_angular_, R_kf_angular_)}
    // };
    filters_.emplace(FilterType::ACC_X, KalmanFilter(Q_kf_linear_, R_kf_linear_));
    filters_.emplace(FilterType::ACC_Y, KalmanFilter(Q_kf_linear_, R_kf_linear_));
    filters_.emplace(FilterType::ACC_Z, KalmanFilter(Q_kf_linear_, R_kf_linear_));
    filters_.emplace(FilterType::ROLL, KalmanFilter(Q_kf_angular_, R_kf_angular_));
    filters_.emplace(FilterType::PITCH, KalmanFilter(Q_kf_angular_, R_kf_angular_));
    filters_.emplace(FilterType::YAW, KalmanFilter(Q_kf_angular_, R_kf_angular_));
    filters_.emplace(FilterType::MOTION_A, KalmanFilter(Q_kf_motion_a_, R_kf_motion_a_));
    filters_.emplace(FilterType::OMEGA_FUSED, KalmanFilter(Q_kf_angular_, R_kf_angular_));
    

    private_nh_.param<std::string>("save_filename", filename_, "/home/systemlabagx/disk/0628_test.csv");
    cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("processed_imu", 10);
    imu_float_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("processed_imu_float", 10);
    imu_sub_ = nh_.subscribe("/imu/data", 100, &TeleopKeyboard::imuCallback, this);
    motion_array_sub_ = nh_.subscribe("motion_array", 100, &TeleopKeyboard::motionArrayCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &TeleopKeyboard::odomCallback, this);

    keyboard_timer_ = nh_.createTimer(ros::Duration(0.1), &TeleopKeyboard::keyboardCallback, this);
    distance_timer_ = nh_.createTimer(ros::Duration(0.05), &TeleopKeyboard::distanceTimerCallback, this); // 新增：20Hz 定時器
    angular_acc_timer_ = nh_.createTimer(ros::Duration(0.05), &TeleopKeyboard::angularAccTimerCallback, this);

    tcgetattr(STDIN_FILENO, &cooked_);
    raw_ = cooked_;
    raw_.c_lflag &= ~(ICANON | ECHO);
    raw_.c_cc[VMIN] = 0;
    raw_.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
    valid_keys_ = {KEY_W, KEY_S, KEY_A, KEY_D, KEY_Q, KEY_E, KEY_X, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9};

    motion_x_ = motion_y_ = motion_th_ = 0.0;
    motion_v_real_ = motion_v_real_kf = motion_omega_fused_ = motion_a_real_ = 0.0;
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
                << "motion_v_real,motion_v_real_kf,"
                << "motion_a_real,motion_a_real_kf,vl_real,vr_real,comp_L,comp_R,LLPWM,RRPWM,"
                << "omega_a_real,omega_a_diff,differential_angular_v,low_filter_imu_acc_x,low_filter_imu_acc_y,"
                << "filtered_roll_rate,filtered_pitch_rate,filtered_yaw_rate,filtered_omega_fused,"
                << "angular_acceleration_roll,angular_acceleration_pitch,angular_acceleration_yaw,angular_acceleration_fused,"
                << "imu_dt,motion_dt,"
                << "current_pose_x,current_pose_y,current_pose_z,pose_roll,pose_pitch,pose_yaw,"
                << "Power_regression,Power_regression_test"
                << "total_distance" 
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
// lidar
float odometry = 0;
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

    // 初始化首次位姿
    if (!first_odom_received_) {
        last_pose_x_ = current_pose_x;
        last_pose_y_ = current_pose_y;
        first_odom_received_ = true;
    }
}

float TeleopKeyboard::computeAngularAcceleration(float current_rate, float prev_rate, ros::Time current_time, ros::Time& last_time, float& last_dt) {
    float angular_acc = 0.0f;
    if (!last_time.isZero()) {
        float dt = (current_time - last_time).toSec();
        if (dt > 0.0f && dt < 0.2f) {
            angular_acc = (current_rate - prev_rate) / dt;
            last_dt = dt;
        } else {
            ROS_WARN("Abnormal dt: %f seconds, skipping angular acceleration", dt);
        }
    }
    last_time = current_time;
    return angular_acc;
}
void TeleopKeyboard::angularAccTimerCallback(const ros::TimerEvent& event) {
    ros::Time current_time = event.current_real;

    // 計算所有角加速度
    angular_acceleration_roll_  = computeAngularAcceleration(
    x_kf_roll_rate,  pre_x_kf_roll_rate,  msg->header.stamp, last_roll_time_,  last_roll_dt_);
angular_acceleration_pitch_ = computeAngularAcceleration(
    x_kf_pitch_rate, pre_x_kf_pitch_rate, msg->header.stamp, last_pitch_time_, last_pitch_dt_);
angular_acceleration_yaw_   = computeAngularAcceleration(
    x_kf_yaw_rate,   pre_x_kf_yaw_rate,   msg->header.stamp, last_yaw_time_,   last_yaw_dt_);

    // 更新前一次速率
    pre_x_kf_roll_rate = x_kf_roll_rate;
    pre_x_kf_pitch_rate = x_kf_pitch_rate;
    pre_x_kf_yaw_rate = x_kf_yaw_rate;

    // ROS_DEBUG("Angular accelerations: roll=%f, pitch=%f, yaw=%f, fused=%f",
    //           angular_acceleration_roll_, angular_acceleration_pitch_, angular_acceleration_yaw_, angular_acceleration_fused_);
}

void TeleopKeyboard::distanceTimerCallback(const ros::TimerEvent&) {
    // 若尚未收到首次 Odometry 數據，則跳過
    if (!first_odom_received_) {
        return;
    }

    // 計算兩次位姿之間的距離
    double dx = current_pose_x - last_pose_x_;
    double dy = current_pose_y - last_pose_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 累加到總里程
    total_distance_ += distance;

    // 更新上一次位姿
    last_pose_x_ = current_pose_x;
    last_pose_y_ = current_pose_y;

    ROS_DEBUG("Calculated distance: %f, Total distance: %f", distance, total_distance_);
}

void TeleopKeyboard::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    ros::Rate loop_rate(20);
    ROS_DEBUG("收到 IMU 數據: accel_z=%f, yaw=%f", msg->linear_acceleration.z, msg->angular_velocity.z);

    imu_acc_x_ = msg->linear_acceleration.z;
    imu_acc_y_ = (msg->linear_acceleration.x) * -1;
    imu_acc_z_ = (msg->linear_acceleration.y) * -1;
    imu_roll_rate = msg->angular_velocity.z;
    imu_pitch_rate = (msg->angular_velocity.x) * -1;
    imu_yaw_rate = (msg->angular_velocity.y) * -1;

    diff_imu_z = imu_acc_z_ - pre_imu_z;

    // 卡爾曼濾波更新
    x_kf_acc_x_ = filters_[FilterType::ACC_X].update(static_cast<float>(imu_acc_x_));
    x_kf_acc_y_ = filters_[FilterType::ACC_Y].update(static_cast<float>(imu_acc_y_));
    x_kf_acc_z_ = filters_[FilterType::ACC_Z].update(static_cast<float>(imu_acc_z_));
    x_kf_roll_rate = filters_[FilterType::ROLL].update(static_cast<float>(imu_roll_rate));
    x_kf_pitch_rate = filters_[FilterType::PITCH].update(static_cast<float>(imu_pitch_rate));
    x_kf_yaw_rate = filters_[FilterType::YAW].update(static_cast<float>(imu_yaw_rate));

    // 計算角加速度
    angular_acceleration_roll_ = computeAngularAcceleration(
        x_kf_roll_rate, pre_x_kf_roll_rate, msg->header.stamp, last_imu_time_, last_imu_dt_);
    angular_acceleration_pitch_ = computeAngularAcceleration(
        x_kf_pitch_rate, pre_x_kf_pitch_rate, msg->header.stamp, last_imu_time_, last_imu_dt_);
    angular_acceleration_yaw_ = computeAngularAcceleration(
        x_kf_yaw_rate, pre_x_kf_yaw_rate, msg->header.stamp, last_imu_time_, last_imu_dt_);

    pre_x_kf_roll_rate = x_kf_roll_rate;
    pre_x_kf_pitch_rate = x_kf_pitch_rate;
    pre_x_kf_yaw_rate = x_kf_yaw_rate;

    // 發布濾波後的 IMU 數據
    std_msgs::Float32MultiArray float_msg;
    float_msg.data = {angular_acceleration_roll_,
                      angular_acceleration_pitch_,
                      angular_acceleration_yaw_,
                      x_kf_roll_rate,
                      x_kf_pitch_rate,
                      x_kf_yaw_rate};
    imu_float_pub_.publish(float_msg);
    pre_imu_z = imu_acc_z_;
    loop_rate.sleep();
}

void TeleopKeyboard::motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg) {
    if (msg->data.size() == 15) {
        motion_x_ = msg->data[0];
        motion_y_ = msg->data[1];
        motion_v_real_ = msg->data[2];
        motion_v_real_kf = msg->data[3];
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

        // 功率回歸計算
        float cofficient[9] = {28.92f, 300.96f, 92.8f, -3.5f, 298.6f, 11.13f, -8.16f, 950.88f, 850.38f};
        float c1_2nd = 90.5f;

        if (cmd_.data == 6 || cmd_.data == 7) {
            Power_regression = c1_2nd +
                               cofficient[1] * fabs(motion_v_real_) +
                               cofficient[2] * std::max(motion_a_real_, 0.0) * fabs(motion_v_real_) +
                               cofficient[3] * std::min(motion_a_real_, 0.0) * fabs(motion_v_real_) +
                               cofficient[4] * fabs(imu_yaw_rate) +
                               cofficient[5] * std::max(angular_acceleration_yaw_, 0.0f) * fabs(imu_yaw_rate) +
                               cofficient[6] * std::min(angular_acceleration_yaw_, 0.0f) * fabs(imu_yaw_rate) +
                               cofficient[7] * std::sin(-pose_yaw) +
                               cofficient[8] * std::sin(pose_yaw);
        } else {
            Power_regression = cofficient[0] +
                               cofficient[1] * fabs(motion_v_real_) +
                               cofficient[2] * std::max(motion_a_real_, 0.0) * fabs(motion_v_real_) +
                               cofficient[3] * std::min(motion_a_real_, 0.0) * fabs(motion_v_real_) +
                               cofficient[4] * fabs(imu_yaw_rate) +
                               cofficient[5] * std::max(angular_acceleration_yaw_, 0.0f) * fabs(imu_yaw_rate) +
                               cofficient[6] * std::min(angular_acceleration_yaw_, 0.0f) * fabs(imu_yaw_rate) +
                               cofficient[7] * std::sin(-pose_yaw) +
                               cofficient[8] * std::sin(pose_yaw);
        }
            Power_regression_test = 0;

        // 卡爾曼濾波更新
        x_kf_motion_a_ = filters_[FilterType::MOTION_A].update(static_cast<float>(motion_a_real_));
        x_kf_omega_fused_ = filters_[FilterType::OMEGA_FUSED].update(static_cast<float>(motion_omega_fused_));

        // 計算融合角加速度
        angular_acceleration_fused_ = computeAngularAcceleration(
            x_kf_omega_fused_, pre_x_kf_omega_fused_, ros::Time::now(), last_motion_time_, last_motion_dt_);
        pre_x_kf_omega_fused_ = x_kf_omega_fused_;

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
        ros::Time current_time = ros::Time::now();
        double timestamp = current_time.toSec();
        ofs << std::fixed << std::setprecision(3)
            << timestamp << ","
            << imu_acc_x_ << "," << imu_acc_y_ << "," << imu_acc_z_ << ","
            << x_kf_acc_x_ << "," << x_kf_acc_y_ << "," << x_kf_acc_z_ << ","
            << imu_roll_rate << "," << imu_pitch_rate << "," << imu_yaw_rate << ","
            << motion_x_ << "," << motion_y_ << "," << motion_th_ << ","
            << motion_v_real_ << "," << motion_v_real_kf << ","
            << motion_a_real_ << "," << x_kf_motion_a_ << "," << vl_real << "," << vr_real << ","
            << comp_L << "," << comp_R << "," << LLPWM << "," << RRPWM << ","
            << motion_v_real_ * x_kf_yaw_rate << "," << diff_imu_z / 0.05 << ","
            << differential_angular_v << ","
            << low_filter_imu_acc_x << "," << low_filter_imu_acc_y << ","
            << x_kf_roll_rate << "," << x_kf_pitch_rate << "," << x_kf_yaw_rate << "," << x_kf_omega_fused_ << ","
            << angular_acceleration_roll_ << "," << angular_acceleration_pitch_ << "," << angular_acceleration_yaw_ << "," << angular_acceleration_fused_ << ","
            << last_imu_dt_ << "," << last_motion_dt_ << ","
            << current_pose_x << "," << current_pose_y << "," << current_pose_z << ","
            << pose_roll << "," << pose_pitch << "," << pose_yaw << ","
            << Power_regression << "," << Power_regression_test<<","
            << total_distance_ // 新增：儲存總里程
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
