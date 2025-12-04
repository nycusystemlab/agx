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
    void motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg); // 處理motion_array數據

    void keyboardCallback(const ros::TimerEvent&);
    void saveImuAndMotionData();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher imu_pub_;        // 新增：發布 sensor_msgs::Imu
    ros::Publisher imu_float_pub_;  // 新增：發布 std_msgs::Float32MultiArray

    ros::Subscriber imu_sub_;
    ros::Subscriber motion_array_sub_;                                 // 訂閱motion_array數據
    ros::Timer keyboard_timer_;
    std_msgs::UInt8 cmd_;
    std::string filename_;

    // IMU數據
    double imu_x_, imu_y_, imu_z_;
    double imu_roll_, imu_pitch_, imu_yaw_;
    // Motion array數據
    double motion_x_, motion_y_, motion_th_;                           // 位置 (m) 和航向 (rad)
    double motion_v_real_, motion_omega_fused_, motion_a_real_;        // 線速度 (m/s), 角速度
    struct termios raw_, cooked_;
    static int key_counter_;
    std::set<char> valid_keys_;
};

int TeleopKeyboard::key_counter_ = 0;

TeleopKeyboard::TeleopKeyboard() : private_nh_("~") {
    // 從參數伺服器獲取CSV檔案路徑，預設為指定路徑
    // private_nh_.param<std::string>("save_filename", filename_, "/home/systemlabagx/disk/keyboard_imu.csv");
    private_nh_.param<std::string>("save_filename", filename_, "/home/systemlabagx/disk/0505_power_test/v=1.0_a=0.5.csv");
    // 初始化發布者和訂閱者
    cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("processed_imu", 10);
    imu_float_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("processed_imu_float", 10);
    // imu_sub_ = nh_.subscribe("/camera/imu", 10, &TeleopKeyboard::imuCallback, this); // 原始imu的
    imu_sub_ = nh_.subscribe("/imu/data", 10, &TeleopKeyboard::imuCallback, this);      // 過濾掉重力的
    
    motion_array_sub_ = nh_.subscribe("motion_array", 10, &TeleopKeyboard::motionArrayCallback, this); // 訂閱motion_array數據
    
    keyboard_timer_ = nh_.createTimer(ros::Duration(0.1), &TeleopKeyboard::keyboardCallback, this);
    tcgetattr(STDIN_FILENO, &cooked_);
    raw_ = cooked_;
    raw_.c_lflag &= ~(ICANON | ECHO);
    raw_.c_cc[VMIN] = 0;
    raw_.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
    valid_keys_ = {KEY_W, KEY_S, KEY_A, KEY_D, KEY_Q, KEY_E, KEY_X};

    // 初始化motion_array數據
    motion_x_ = motion_y_ = motion_th_ = 0.0;
    motion_v_real_ = motion_omega_fused_ = motion_a_real_ = 0.0;
}

TeleopKeyboard::~TeleopKeyboard() {
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked_);
}

void TeleopKeyboard::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    //ros::Rate loop_rate(15);   // 設定此函式迴圈的頻率 15 是極限不能再大
    ROS_DEBUG("收到 IMU 數據: accel_z=%f, yaw=%f", 
              msg->linear_acceleration.z, msg->angular_velocity.z);
    // 提取並調整IMU數據（座標系校正）
    imu_x_ = msg->linear_acceleration.z;
    imu_y_ = (msg->linear_acceleration.x) * -1;
    imu_z_ = (msg->linear_acceleration.y) * -1;
    imu_roll_ = msg->angular_velocity.z;
    imu_pitch_ = (msg->angular_velocity.x) * -1;
    imu_yaw_ = (msg->angular_velocity.y) * -1;

    // 發布 sensor_msgs::Imu
    sensor_msgs::Imu imu_msg;
    imu_msg.header = msg->header;
    imu_msg.linear_acceleration.x = imu_x_ ;
    imu_msg.linear_acceleration.y = imu_y_ ;
    imu_msg.linear_acceleration.z = imu_z_ ;  // imu + 補償 + 反重力值
    imu_msg.angular_velocity.x = imu_roll_;
    imu_msg.angular_velocity.y = imu_pitch_;
    imu_msg.angular_velocity.z = imu_yaw_;
    imu_msg.orientation = msg->orientation;
    imu_pub_.publish(imu_msg);

    // 儲存IMU和motion_array數據
    
    // 發布 std_msgs::Float32MultiArray
    std_msgs::Float32MultiArray float_msg;
    float_msg.data = {static_cast<float>(imu_x_),
                      static_cast<float>(imu_y_),
                      static_cast<float>(imu_z_),
                      static_cast<float>(imu_roll_),
                      static_cast<float>(imu_pitch_),
                      static_cast<float>(imu_yaw_)};
    imu_float_pub_.publish(float_msg);
    //loop_rate.sleep(); //停一下才繼續, 防止資料量過多
    saveImuAndMotionData();
}

// motion_array回調：處理接收到的運動數據
void TeleopKeyboard::motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg) {
    // 檢查數據長度是否正確（應為6：x, y, th, V_real, omega_fused, a_real）
    if (msg->data.size() == 6) {
        motion_x_ = msg->data[0];           // 位置x (m)
        motion_y_ = msg->data[1];           // 位置y (m)
        motion_th_ = msg->data[2];          // 航向theta (rad)
        motion_v_real_ = msg->data[3];      // 線速度 (m/s)
        motion_omega_fused_ = msg->data[4]; // 角速度 (rad/s)
        motion_a_real_ = msg->data[5];      // 線加速度 (m/s²)
        ROS_DEBUG("收到 motion_array 數據: x=%f, y=%f, th=%f, v=%f, omega=%f, a=%f",
                  motion_x_, motion_y_, motion_th_, motion_v_real_, motion_omega_fused_, motion_a_real_);
    } else {
        ROS_WARN("motion_array 數據長度錯誤: %zu", msg->data.size());
    }
}

// 鍵盤回調：處理鍵盤輸入並發布運動命令
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

// 儲存IMU和motion_array數據到CSV檔案
void TeleopKeyboard::saveImuAndMotionData() {
    std::ofstream ofs(filename_.c_str(), std::ios::app);
    if (ofs.is_open()) {
        // 儲存格式：IMU加速度(x,y,z)、角速度(roll,pitch,yaw)、平面加速度模、motion_array數據(x,y,th,v,omega,a)
        ofs << std::fixed << std::setprecision(3)
            << imu_x_ << "," << imu_y_ << "," << imu_z_ << ","
            << imu_roll_ << "," << imu_pitch_ << "," << imu_yaw_ << ","
            << sqrt(pow(imu_x_, 2) + pow(imu_y_, 2)) << ","
            << motion_x_ << "," << motion_y_ << "," << motion_th_ << ","
            << motion_v_real_ << "," << motion_omega_fused_ << "," << motion_a_real_
            << std::endl;
        ofs.close();
    } else {
        ROS_WARN("無法打開檔案: %s", filename_.c_str());
    }
}

// void TeleopKeyboard::saveImuData() {
//     std::ofstream ofs(filename_.c_str(), std::ios::app);
//     if (ofs.is_open()) {
//         ofs << std::fixed << std::setprecision(3)
//             << abs(imu_x_) << "," << abs(imu_y_) << "," << abs(imu_z_)<< ","
//             << imu_roll_ << "," << imu_pitch_ << "," << imu_yaw_ << "," << sqrt(pow(imu_x_ , 2) + pow(imu_y_, 2)) <<std::endl;
//     }
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_teleop");
    TeleopKeyboard teleop;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}