#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <fstream>
#include <set>

#define KEY_W 'w'
#define KEY_S 's'
#define KEY_A 'a'
#define KEY_D 'd'
#define KEY_Q 'q'
#define KEY_E 'e'
#define KEY_X 'x'

class TeleopKeyboard {
public:
    TeleopKeyboard();
    ~TeleopKeyboard();

private:
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void keyboardCallback(const ros::TimerEvent&);
    void saveImuData();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher imu_pub_;        // 新增：發布 sensor_msgs::Imu
    ros::Publisher imu_float_pub_;  // 新增：發布 std_msgs::Float32MultiArray
    ros::Subscriber imu_sub_;
    ros::Timer keyboard_timer_;
    std_msgs::UInt8 cmd_;
    std::string filename_;
    double imu_x_, imu_y_, imu_z_;
    double imu_roll_, imu_pitch_, imu_yaw_;
    struct termios raw_, cooked_;
    static int key_counter_;
    std::set<char> valid_keys_;
};

int TeleopKeyboard::key_counter_ = 0;

TeleopKeyboard::TeleopKeyboard() : private_nh_("~") {
    private_nh_.param<std::string>("save_filename", filename_, "/home/systemlabagx/disk/keyboard_imu.csv");
    cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("processed_imu", 10);
    imu_float_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("processed_imu_float", 10);
    // imu_sub_ = nh_.subscribe("/camera/imu", 10, &TeleopKeyboard::imuCallback, this); // 原始imu的
    imu_sub_ = nh_.subscribe("/imu/data", 10, &TeleopKeyboard::imuCallback, this);      // 過濾掉重力的
    keyboard_timer_ = nh_.createTimer(ros::Duration(0.1), &TeleopKeyboard::keyboardCallback, this);
    tcgetattr(STDIN_FILENO, &cooked_);
    raw_ = cooked_;
    raw_.c_lflag &= ~(ICANON | ECHO);
    raw_.c_cc[VMIN] = 0;
    raw_.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
    valid_keys_ = {KEY_W, KEY_S, KEY_A, KEY_D, KEY_Q, KEY_E, KEY_X};
}

TeleopKeyboard::~TeleopKeyboard() {
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked_);
}

void TeleopKeyboard::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    ros::Rate loop_rate(15);   // 設定此函式迴圈的頻率 15 是極限不能再大
    ROS_DEBUG("收到 IMU 數據: accel_z=%f, yaw=%f", 
              msg->linear_acceleration.z, msg->angular_velocity.z);
    imu_x_ = msg->linear_acceleration.z;
    imu_y_ = (msg->linear_acceleration.x) * -1;
    imu_z_ = (msg->linear_acceleration.y) * -1;
    imu_roll_ = msg->angular_velocity.z;
    imu_pitch_ = (msg->angular_velocity.x) * -1;
    imu_yaw_ = (msg->angular_velocity.y) * -1;

    // 發布 sensor_msgs::Imu
    sensor_msgs::Imu imu_msg;
    imu_msg.header = msg->header;
    imu_msg.linear_acceleration.x = abs(imu_x_) ;
    imu_msg.linear_acceleration.y = imu_y_ ;
    imu_msg.linear_acceleration.z = imu_z_ ;  // imu + 補償 + 反重力值
    imu_msg.angular_velocity.x = imu_roll_;
    imu_msg.angular_velocity.y = imu_pitch_;
    imu_msg.angular_velocity.z = imu_yaw_;
    imu_msg.orientation = msg->orientation;
    imu_pub_.publish(imu_msg);
    saveImuData();
    // 發布 std_msgs::Float32MultiArray
    // std_msgs::Float32MultiArray float_msg;
    // float_msg.data = {static_cast<float>(imu_x_ + 0.33),
    //                   static_cast<float>(imu_y_ - 0.40),
    //                   static_cast<float>(imu_z_ + 0.12),
    //                   static_cast<float>(imu_roll_),
    //                   static_cast<float>(imu_pitch_),
    //                   static_cast<float>(imu_yaw_)};
    // imu_float_pub_.publish(float_msg);
    loop_rate.sleep(); //停一下才繼續, 防止資料量過多
}

void TeleopKeyboard::keyboardCallback(const ros::TimerEvent&) {
    struct pollfd fd = {STDIN_FILENO, POLLIN, 0};
    int ret = poll(&fd, 1, 10); // 10ms 超時
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
                    default: return;
                }
                cmd_pub_.publish(cmd_);
                ROS_INFO("Published cmd_vel: %d", cmd_.data);
                // if (key_counter_++ % 10 == 0) {
                //     saveImuData();
                // }
            } else {
                ROS_WARN("Invalid key: %c", c);
            }
        } else {
            ROS_DEBUG("Read failed or no data");
        }
    }
}


void TeleopKeyboard::saveImuData() {
    std::ofstream ofs(filename_.c_str(), std::ios::app);
    if (ofs.is_open()) {
        ofs << std::fixed << std::setprecision(3)
            << abs(imu_x_) << "," << abs(imu_y_) << "," << abs(imu_z_)<< ","
            << imu_roll_ << "," << imu_pitch_ << "," << imu_yaw_ << "," << sqrt(pow(imu_x_ , 2) + pow(imu_y_, 2)) <<std::endl;
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