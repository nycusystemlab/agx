#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>  // 使用 std_msgs::UInt8 代替 geometry_msgs::Twist
#include <stdio.h>

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
    ros::Publisher pub_;       // 發布者
    std_msgs::UInt8 cmd_;      // 使用 std_msgs::UInt8 作為訊息類型

public:
    TeleopKeyboard()
    {
        // 初始化發布者，發布到 "cmd_vel" 主題
        pub_ = nh_.advertise<std_msgs::UInt8>("cmd_vel", 1);
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

        ROS_INFO("Press W: forward, S: backward, X: stop, A: left roatate, D: right roatate, Q: function1, E: function2");

        while (ros::ok())
        {
            c = getchar();  // 讀取按鍵
            // 處理按鍵...
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

            // 發布訊息
            pub_.publish(cmd_);
            ROS_INFO("mission: %d", cmd_.data);
        }

        // 恢復終端設置
        tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
    }
};

int main(int argc, char **argv)
{
    // 初始化 ROS 節點
    ros::init(argc, argv, "keyboard_teleop");

    // 創建 TeleopKeyboard 對象
    TeleopKeyboard teleop;

    // 進入按鍵循環
    teleop.keyboardLoop();

    return 0;
}