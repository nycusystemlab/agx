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

    // -------- KF 相关成员 --------
    bool   kf_initialized_x_, kf_initialized_y_, kf_initialized_z_;
    float  x_kf_x_, P_kf_x_;    // X 轴滤波后的加速度估计 和 协方差
    float  x_kf_y_, P_kf_y_;    // Y 轴
    float  x_kf_z_, P_kf_z_;    // Z 轴

    const float Q_kf_;  // 过程噪声方差（同用于三轴，可根据实验再调）
    const float R_kf_;  // 测量噪声方差（同用于三轴，可根据实验再调）

    // IMU數據
    double imu_acc_x_, imu_acc_y_, imu_acc_z_;
    double imu_roll_rate, imu_pitch_rate, imu_yaw_rate;
    // double imu_roll, imu_pitch, imu_yaw;
    
    // class 外面先放目前位置變數
    double current_pose_x = 0.0;
    double current_pose_y = 0.0;
    double current_pose_z = 0.0;
    double pose_roll = 0.0, pose_pitch = 0.0, pose_yaw = 0.0;

    // 濾波後IMU數據
    // double imu_filtered_x;
    // double imu_filtered_y;
    // double imu_filtered_z;

    // Motion array數據
    double motion_x_, motion_y_, motion_th_;
    double motion_v_real_, motion_omega_fused_;
    double motion_a_real_, vl_real, vr_real, comp_L, comp_R, LLPWM, RRPWM;
    double differential_angular_v;
    double low_filter_imu_acc_x,low_filter_imu_acc_y;
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
    x_kf_x_(0.0f),
    P_kf_x_(1.0f),
    x_kf_y_(0.0f),
    P_kf_y_(1.0f),
    x_kf_z_(0.0f),
    P_kf_z_(1.0f),
    Q_kf_(0.0001f),  // 过程噪声方差，可根据实际实验调整
    R_kf_(0.01f)   // 测量噪声方差，可根据实际实验调整
    {
    private_nh_.param<std::string>("save_filename", filename_, "/home/systemlabagx/disk/0610_test.csv");
    cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("processed_imu", 10);
    imu_float_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("processed_imu_float", 10);
    imu_sub_ = nh_.subscribe("/imu/data", 10, &TeleopKeyboard::imuCallback, this);
    motion_array_sub_ = nh_.subscribe("motion_array", 10, &TeleopKeyboard::motionArrayCallback, this);
    odom_sub_         = nh_.subscribe("/odom",       10, &TeleopKeyboard::odomCallback, this);

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
    // filtered_imu_acc_x_ = filtered_imu_acc_y_ = filtered_imu_acc_z_ = 0.0;

    std::ofstream ofs(filename_.c_str(), std::ios::app);
    if (ofs.is_open()) {
        ofs.seekp(0, std::ios::end);
        if (ofs.tellp() == 0) {
            ofs << "timestamp,"
                << "imu_x,imu_y,imu_z,"
                << "imu_filtered_x,imu_filtered_y,imu_filtered_z,"
                << "imu_roll,imu_pitch,imu_yaw,imu_magnitude,"
                << "motion_x,motion_y,motion_th,motion_v_real,motion_omega_fused,"
                << "motion_a_real,vl_real,vr_real,comp_L,comp_R,LLPWM,RRPWM,omega_a_real,omega_a_diff,differential_angular_v,low_filter_imu_acc_x,low_filter_imu_acc_y,"
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
    // 取出位置
    current_pose_x = msg->pose.pose.position.x;
    current_pose_y = msg->pose.pose.position.y;
    current_pose_z = msg->pose.pose.position.z;

    // 取出朝向並轉 Euler
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

    // 2) 若尚未初始化，就用第一帧观测做初始化
    if (!kf_initialized_x_) {
        x_kf_x_ = static_cast<float>(imu_acc_x_);
        P_kf_x_ = 1.0f;
        kf_initialized_x_ = true;
    }
    if (!kf_initialized_y_) {
        x_kf_y_ = static_cast<float>(imu_acc_y_);
        P_kf_y_ = 1.0f;
        kf_initialized_y_ = true;
    }
    if (!kf_initialized_z_) {
        x_kf_z_ = static_cast<float>(imu_acc_z_);
        P_kf_z_ = 1.0f;
        kf_initialized_z_ = true;
    }
    // Predict
    {
    float x_pred = x_kf_x_;
    float P_pred = P_kf_x_ + Q_kf_;
    // Measurement
    float z = static_cast<float>(imu_acc_x_);
    float R_use = R_kf_;
    // Update
    float K = P_pred / (P_pred + R_use);
    x_kf_x_ = x_pred + K * (z - x_pred);
    P_kf_x_ = (1.0f - K) * P_pred;
    }
    {
    float x_pred = x_kf_y_;
    float P_pred = P_kf_y_ + Q_kf_;
    float z = static_cast<float>(imu_acc_y_);
    float R_use = R_kf_;
    float K = P_pred / (P_pred + R_use);
    x_kf_y_ = x_pred + K * (z - x_pred);
    P_kf_y_ = (1.0f - K) * P_pred;
    }
    {
    float x_pred = x_kf_z_;
    float P_pred = P_kf_z_ + Q_kf_;
    float z = static_cast<float>(imu_acc_z_);
    float R_use = R_kf_;
    float K = P_pred / (P_pred + R_use);
    x_kf_z_ = x_pred + K * (z - x_pred);
    P_kf_z_ = (1.0f - K) * P_pred;
    }

    // ------------ 四元數 → Euler ------------  
    // tf::Quaternion q;
    // tf::quaternionMsgToTF(msg->orientation, q);
    // double roll, pitch, yaw;
    // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // imu_roll  = roll;
    // imu_pitch = pitch;
    // imu_yaw   = yaw;

    // sensor_msgs::Imu imu_msg;
    // imu_msg.header = msg->header;
    // imu_msg.linear_acceleration.x = imu_acc_x_;
    // imu_msg.linear_acceleration.y = imu_acc_y_;
    // imu_msg.linear_acceleration.z = imu_acc_z_;
    // imu_msg.angular_velocity.x = imu_roll_rate;
    // imu_msg.angular_velocity.y = imu_pitch_rate;
    // imu_msg.angular_velocity.z = imu_yaw_rate;
    // imu_msg.orientation = msg->orientation;
    // imu_pub_.publish(imu_msg);

    std_msgs::Float32MultiArray float_msg;
    float_msg.data = {static_cast<float>(x_kf_x_),
                      static_cast<float>(x_kf_y_),
                      static_cast<float>(x_kf_z_),
                      static_cast<float>(imu_roll_rate),
                      static_cast<float>(imu_pitch_rate),
                      static_cast<float>(imu_yaw_rate)};
    imu_float_pub_.publish(float_msg);
    pre_imu_z = imu_acc_z_;
    loop_rate.sleep();
    // saveImuAndMotionData();
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
        low_filter_imu_acc_x   = msg->data[13];
        low_filter_imu_acc_y   = msg->data[14];

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
            ofs << "timestamp,imu_x,imu_y,imu_z,"
                << "imu_filtered_x,imu_filtered_y,imu_filtered_z,"
                << "imu_roll,imu_pitch,imu_yaw,imu_magnitude,"
                << "motion_x,motion_y,motion_th,motion_v_real,motion_omega_fused,"
                << "motion_a_real,vl_real,vr_real,comp_L,comp_R,LLPWM,RRPWM,omega_a_real,omega_a_diff,differential_angular_v,low_filter_imu_acc_x,low_filter_imu_acc_y,"
                << "current_pose_x,current_pose_y,current_pose_z,pose_roll,pose_pitch,pose_yaw"
                << std::endl;
            header_written_ = true;
        }
        // 獲取當前時間戳
        ros::Time current_time = ros::Time::now();
        
        double timestamp = current_time.toSec(); // 時間戳以秒為單位

        ofs << std::fixed << std::setprecision(3)
            << timestamp << "," // 加入時間戳
            << imu_acc_x_ << "," << imu_acc_y_ << "," << imu_acc_z_ << ","
            << x_kf_x_ << "," << x_kf_y_ << "," << x_kf_z_ << ","
            << imu_roll_rate << "," << imu_pitch_rate << "," << imu_yaw_rate << ","
            << sqrt(pow(imu_roll_rate, 2) + pow(imu_pitch_rate, 2)) << ","
            << motion_x_ << "," << motion_y_ << "," << motion_th_ << ","
            << motion_v_real_ << "," << motion_omega_fused_ << ","
            << motion_a_real_ << "," << vl_real << "," << vr_real << ","
            << comp_L << "," << comp_R << ","<< LLPWM << "," << RRPWM << ","<< motion_v_real_ * imu_yaw_rate<< "," << diff_imu_z/0.05 << "," << differential_angular_v << ","
            
            << low_filter_imu_acc_x << "," << low_filter_imu_acc_x    << ","   

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