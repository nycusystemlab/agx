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


class MovingAverage {
private:
    std::vector<float> buffer;
    size_t window_size;
    size_t index;

public:
    MovingAverage(size_t size) : window_size(size), index(0) {
        buffer.resize(size, 0.0f);
    }

    float add_sample(float sample) {
        buffer[index] = sample;
        index = (index + 1) % window_size;
        return std::accumulate(buffer.begin(), buffer.end(), 0.0f) / window_size;
    }
};

class TeleopKeyboard {
public:
    TeleopKeyboard();
    ~TeleopKeyboard();

private:
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void keyboardCallback(const ros::TimerEvent&);
    void saveImuAndMotionData();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher imu_float_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber motion_array_sub_;
    ros::Timer keyboard_timer_;
    std_msgs::UInt8 cmd_;
    std::string filename_;

    // IMU數據
    double imu_x_, imu_y_, imu_z_;
    double imu_roll_, imu_pitch_, imu_yaw_;
    // 濾波後IMU數據
    MovingAverage filter_x_;
    MovingAverage filter_y_;
    MovingAverage filter_z_;
    double filtered_imu_x_;
    double filtered_imu_y_;
    double filtered_imu_z_;
    // Motion array數據
    double motion_x_, motion_y_, motion_th_;
    double motion_v_real_, motion_omega_fused_;
    double motion_a_real_, vl_real, vr_real, comp_L, comp_R, LLPWM, RRPWM;
    struct termios raw_, cooked_;
    static int key_counter_;
    std::set<char> valid_keys_;
    static bool header_written_;
};

int TeleopKeyboard::key_counter_ = 0;
bool TeleopKeyboard::header_written_ = false;

TeleopKeyboard::TeleopKeyboard() : private_nh_("~"), filter_x_(10), filter_y_(10), filter_z_(10) {
    private_nh_.param<std::string>("save_filename", filename_, "/home/systemlabagx/disk/0529_test.csv");
    cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("processed_imu", 10);
    imu_float_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("processed_imu_float", 10);
    imu_sub_ = nh_.subscribe("/imu/data", 10, &TeleopKeyboard::imuCallback, this);
    motion_array_sub_ = nh_.subscribe("motion_array", 10, &TeleopKeyboard::motionArrayCallback, this);
    
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
    filtered_imu_x_ = filtered_imu_y_ = filtered_imu_z_ = 0.0;

    std::ofstream ofs(filename_.c_str(), std::ios::app);
    if (ofs.is_open()) {
        ofs.seekp(0, std::ios::end);
        if (ofs.tellp() == 0) {
            ofs << "timestamp,imu_x,imu_y,imu_z,filtered_imu_x,filtered_imu_y,filtered_imu_z,"
                << "imu_roll,imu_pitch,imu_yaw,imu_magnitude,"
                << "motion_x,motion_y,motion_th,motion_v_real,motion_omega_fused,"
                << "motion_a_real,vl_real,vr_real,comp_L,comp_R,LLPWM,RRPWM"
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

void TeleopKeyboard::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    ros::Rate loop_rate(20);
    ROS_DEBUG("收到 IMU 數據: accel_z=%f, yaw=%f", 
              msg->linear_acceleration.z, msg->angular_velocity.z);

    imu_x_ = msg->linear_acceleration.z;
    imu_y_ = (msg->linear_acceleration.x) * -1;
    imu_z_ = (msg->linear_acceleration.y) * -1;
    imu_roll_ = msg->angular_velocity.z;
    imu_pitch_ = (msg->angular_velocity.x) * -1;
    imu_yaw_ = (msg->angular_velocity.y) * -1;

    filtered_imu_x_ = filter_x_.add_sample(imu_x_);
    filtered_imu_y_ = filter_y_.add_sample(imu_y_);
    filtered_imu_z_ = filter_z_.add_sample(imu_z_);

    sensor_msgs::Imu imu_msg;
    imu_msg.header = msg->header;
    imu_msg.linear_acceleration.x = filtered_imu_x_;
    imu_msg.linear_acceleration.y = filtered_imu_y_;
    imu_msg.linear_acceleration.z = filtered_imu_z_;
    imu_msg.angular_velocity.x = imu_roll_;
    imu_msg.angular_velocity.y = imu_pitch_;
    imu_msg.angular_velocity.z = imu_yaw_;
    imu_msg.orientation = msg->orientation;
    imu_pub_.publish(imu_msg);

    std_msgs::Float32MultiArray float_msg;
    float_msg.data = {static_cast<float>(filtered_imu_x_),
                      static_cast<float>(filtered_imu_y_),
                      static_cast<float>(filtered_imu_z_),
                      static_cast<float>(imu_roll_),
                      static_cast<float>(imu_pitch_),
                      static_cast<float>(imu_yaw_)};
    imu_float_pub_.publish(float_msg);
    loop_rate.sleep();
    // saveImuAndMotionData();
}

void TeleopKeyboard::motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg) {
    if (msg->data.size() == 12) {
        motion_x_ = msg->data[0];
        motion_y_ = msg->data[1];
        motion_th_ = msg->data[2];
        motion_v_real_ = msg->data[3];
        motion_omega_fused_ = msg->data[4];
        motion_a_real_ = msg->data[5];      // a_real_TS
        vl_real = msg->data[6];  // a_real_5TS
        vr_real = msg->data[7];

        comp_L = msg->data[8];  // a_real_5TS
        comp_R = msg->data[9];
        LLPWM = msg->data[10];  // a_real_5TS
        RRPWM = msg->data[11];
        
        ROS_DEBUG("收到 motion_array 數據: x=%f, y=%f, th=%f, v=%f, omega=%f, a=%f, vl_real=%f, vr_real=%f, comp_L=%f, comp_R=%f, LLPWM=%f, RRPWM=%f",
                  motion_x_, motion_y_, motion_th_, motion_v_real_, motion_omega_fused_,
                  motion_a_real_, vl_real, vr_real, comp_L, comp_R, LLPWM, RRPWM);
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
            ofs << "timestamp,imu_x,imu_y,imu_z,filtered_imu_x,filtered_imu_y,filtered_imu_z,"
                << "imu_roll,imu_pitch,imu_yaw,imu_magnitude,"
                << "motion_x,motion_y,motion_th,motion_v_real,motion_omega_fused,"
                << "motion_a_real,vl_real,vr_real,comp_L,comp_R,LLPWM,RRPWM"
                << std::endl;
            header_written_ = true;
        }
        // 獲取當前時間戳
        ros::Time current_time = ros::Time::now();
        
        double timestamp = current_time.toSec(); // 時間戳以秒為單位

        ofs << std::fixed << std::setprecision(3)
            << timestamp << "," // 加入時間戳
            << imu_x_ << "," << imu_y_ << "," << imu_z_ << ","
            << filtered_imu_x_ << "," << filtered_imu_y_ << "," << filtered_imu_z_ << ","
            << imu_roll_ << "," << imu_pitch_ << "," << imu_yaw_ << ","
            << sqrt(pow(filtered_imu_x_, 2) + pow(filtered_imu_y_, 2)) << ","
            << motion_x_ << "," << motion_y_ << "," << motion_th_ << ","
            << motion_v_real_ << "," << motion_omega_fused_ << ","
            << motion_a_real_ << "," << vl_real << "," << vr_real << ","
            << comp_L << "," << comp_R << ","<< LLPWM << "," << RRPWM 
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