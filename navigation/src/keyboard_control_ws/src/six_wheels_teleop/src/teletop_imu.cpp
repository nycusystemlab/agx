#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <fstream>
#include <iomanip>

// 按鍵定義
#define KEY_W 0x77  // W 鍵：前進
#define KEY_S 0x73  // S 鍵：後退
#define KEY_X 0x78  // X 鍵：停止
#define KEY_A 0x61  // A 鍵：左轉
#define KEY_D 0x64  // D 鍵：右轉
#define KEY_Q 0x71  // Q 鍵：左前
#define KEY_E 0x65  // E 鍵：右前

class TeleopKeyboard
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher imu_pub_arduino;
    ros::Subscriber imu_sub_;
    std_msgs::UInt8 cmd_;
    std::string filename_;
    
    // Data variables
    double imu_x = 0.0;
    double imu_y = 0.0;
    double imu_z = 0.0;
    double imu_roll = 0.0;
    double imu_pitch = 0.0;
    double imu_yaw = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    bool recevice_once = false;

    //void imuCallback(const sensor_msgs::ImuConstPtr &msg);

public:
    TeleopKeyboard() : private_nh_("~")
    {
        // Initialize publishers
        cmd_pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 1);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("processed_imu", 1);
        
        imu_pub_arduino = nh_.advertise<std_msgs::Float32MultiArray>("imu_pub_arduino", 1);

        // Initialize IMU subscriber
        imu_sub_ = nh_.subscribe("/camera/imu", 100, &TeleopKeyboard::imuCallback, this);
        
        // Get filename parameter
        private_nh_.param<std::string>("save_filename", filename_, std::string("/home/systemlabagx/disk/imu_1.csv"));
    }

    void keyboardLoop()
    {
        char c;
        struct termios raw, cooked;

        // 獲取終端設置
        tcgetattr(STDIN_FILENO, &cooked);
        raw = cooked;

        // 設置終端為非規範模式（不需要按下 Enter 鍵）
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        ROS_INFO("Press W: forward, S: backward, X: stop, A: left rotate, D: right rotate, Q: function1, E: function2");
        
        ros::Rate rate(50);
        while (ros::ok())
        {
            c = getchar();  // 讀取按鍵
            switch (c)
            {
            case KEY_W:
                cmd_.data = 1;  // 發送數值 1 (前進)
                break;
            case KEY_X:
                cmd_.data = 2;  // 發送數值 2 (後退)
                break;
            case KEY_S:
                cmd_.data = 5;  // 發送數值 5 (停止)
                break;
            case KEY_A:
                cmd_.data = 3;  // 發送數值 3 (左轉)
                break;
            case KEY_D:
                cmd_.data = 4;  // 發送數值 4 (右轉)
                break;
            case KEY_Q:
                cmd_.data = 6;  // 發送數值 6 (左前)
                break;
            case KEY_E:
                cmd_.data = 7;  // 發送數值 7 (右前)
                break;
            default:
                ROS_WARN("無效按鍵: %c", c);  // 提示無效按鍵
                continue;  // 忽略其他按鍵
            }
            //ros::spin();
            //ros::spinOnce();

            // 發布訊息
            cmd_pub_.publish(cmd_);
            ROS_INFO("mission: %d", cmd_.data);
        }
        ros::spinOnce();  // 處理 IMU 訂閱 callback
        rate.sleep();
        // 恢復終端設置
        tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        // Process IMU data
        imu_x = msg->linear_acceleration.z;
        imu_y = (msg->linear_acceleration.x) * -1;
        imu_z = (msg->linear_acceleration.y) * -1;

        imu_roll = msg->angular_velocity.z;
        imu_pitch = (msg->angular_velocity.x) * -1;
        imu_yaw = (msg->angular_velocity.y) * -1;

        tf::Quaternion q(msg->orientation.x,
                         msg->orientation.y,
                         msg->orientation.z,
                         msg->orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // Publish processed IMU data
        // 原始的
        // sensor_msgs::Imu imu_msg;
        // imu_msg.header = msg->header;
        // imu_msg.linear_acceleration.x = imu_x;  // Adjusted axes
        // imu_msg.linear_acceleration.y = imu_y;
        // imu_msg.linear_acceleration.z = imu_z;
        // imu_msg.angular_velocity.x = imu_roll;
        // imu_msg.angular_velocity.y = imu_pitch;
        // imu_msg.angular_velocity.z = imu_yaw;

        //補償的
        sensor_msgs::Imu imu_msg;
        imu_msg.header = msg->header;
        imu_msg.linear_acceleration.x = imu_x +0.33;  // Adjusted axes
        imu_msg.linear_acceleration.y = imu_y -0.40;
        imu_msg.linear_acceleration.z = imu_z + 0.12;
        imu_msg.angular_velocity.x = imu_roll;
        imu_msg.angular_velocity.y = imu_pitch;
        imu_msg.angular_velocity.z = imu_yaw;
        imu_pub_.publish(imu_msg);

        // std_msgs::Float32MultiArray array_msg;
        // std::vector<float> imu_data;
        // imu_data.push_back(static_cast<float>(imu_x_ + 0.33));
        // imu_data.push_back(static_cast<float>(imu_y_ - 0.40));
        // imu_data.push_back(static_cast<float>(imu_z_ + 0.12));
        // imu_data.push_back(static_cast<float>(imu_roll_));
        // imu_data.push_back(static_cast<float>(imu_pitch_));
        // imu_data.push_back(static_cast<float>(imu_yaw_));
        // array_msg.data = imu_data;
        // imu_pub_arduino_.publish(array_msg);

        // 要發到arduino的
        std_msgs::Float32MultiArray array_msg;
        array_msg.data = {static_cast<float>(imu_x + 0.33),
                          static_cast<float>(imu_y - 0.40),
                          static_cast<float>(imu_z + 0.12),
                          static_cast<float>(imu_roll),
                          static_cast<float>(imu_pitch),
                          static_cast<float>(imu_yaw)};
        imu_pub_arduino.publish(array_msg);


        // Log to CSV
        std::ofstream ofs(filename_.c_str(), std::ios::app);
        if (!recevice_once)
        {
            ofs << "acc_x,acc_y,acc_z,roll_rate,pitch_rate,yaw_rate,roll,pitch,yaw" << std::endl;
            ofs << std::fixed << std::setprecision(3)
                << imu_x << "," << imu_y << "," << imu_z
                << "," << imu_roll << "," << imu_pitch << "," << imu_yaw
                << "," << roll << "," << pitch << "," << yaw << std::endl;
            recevice_once = true;
        }
        else
        {
            ofs << std::fixed << std::setprecision(3)
                << imu_x << "," << imu_y << "," << imu_z
                << "," << imu_roll << "," << imu_pitch << "," << imu_yaw
                << "," << roll << "," << pitch << "," << yaw << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    // 初始化 ROS 節點
    ros::init(argc, argv, "keyboard_teleop");

    size_t n = 8;

    // 0 會作為可用的執行序
    // 1 個執行緒來處理所有 callback
    // ros::AsyncSpinner spinner(n);
    // spinner.start();

    // 創建 TeleopKeyboard 對象
    TeleopKeyboard teleop;
    // 進入按鍵循環
    teleop.keyboardLoop();

    //ros::spin();

    return 0;
}