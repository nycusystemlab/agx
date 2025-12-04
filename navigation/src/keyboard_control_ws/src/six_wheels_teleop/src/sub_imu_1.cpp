#include <ros/ros.h>
#include <fstream>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h> // 新增鍵盤命令

double current_pose_x = 0.0;  
double current_pose_y = 0.0;
double current_pose_z = 0.0;
double imu_x = 0.0;
double imu_y = 0.0;
double imu_z = 0.0;
double imu_roll = 0.0;
double imu_pitch = 0.0;
double imu_yaw = 0.0;
double roll = 0, pitch = 0, yaw = 0;
uint8_t current_command = 0; // 記錄鍵盤命令

class WaypointSaver 
{
public:
    WaypointSaver();
private:
    void imuCallback(const sensor_msgs::ImuConstPtr &msg); 
    void current_pose_callback(const nav_msgs::OdometryConstPtr &msg);
    void commandCallback(const std_msgs::UInt8ConstPtr &msg); // 新增

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber imu_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber cmd_sub; // 新增
    std::string filename_;

    double prev_imu_roll = 0.0;
    double prev_imu_pitch = 0.0;
    double prev_imu_yaw = 0.0;
    double alpha_roll = 0.0;
    double alpha_pitch = 0.0;
    double alpha_yaw = 0.0;
    ros::Time prev_time;
    bool first_imu_received = false;
};

WaypointSaver::WaypointSaver() : private_nh_("~")
{
    private_nh_.param<std::string>("save_filename", filename_, std::string("/home/systemlabagx/disk/imu_lidar.csv"));
    imu_sub = nh_.subscribe("/camera/imu", 100, &WaypointSaver::imuCallback, this);  
    pose_sub = nh_.subscribe("/odom", 100, &WaypointSaver::current_pose_callback, this);
    cmd_sub = nh_.subscribe("/cmd_vel", 100, &WaypointSaver::commandCallback, this); // 新增
}

void WaypointSaver::commandCallback(const std_msgs::UInt8ConstPtr &msg)
{
    current_command = msg->data; // 記錄命令（1-7）
}

void WaypointSaver::current_pose_callback(const nav_msgs::OdometryConstPtr &msg)
{
    current_pose_x = msg->pose.pose.position.x;
    current_pose_y = msg->pose.pose.position.y;
    current_pose_z = msg->pose.pose.position.z;

    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

void WaypointSaver::imuCallback(const sensor_msgs::ImuConstPtr &msg)  
{
    std::ofstream ofs(filename_.c_str(), std::ios::app);
    static bool recevice_once = false;

    imu_x = msg->linear_acceleration.z;
    imu_y = (msg->linear_acceleration.x) * -1;
    imu_z = (msg->linear_acceleration.y) * -1;
    imu_roll = msg->angular_velocity.z;
    imu_pitch = (msg->linear_acceleration.x) * -1;
    imu_yaw = (msg->angular_velocity.y) * -1;

    // 計算角加速度
    if (first_imu_received) {
        ros::Duration delta_t = msg->header.stamp - prev_time;
        std::cout << "header.stamp: " << msg->header.stamp << ", delta_t: " << delta_t <<std::endl;
        

        double dt = delta_t.toSec();
        if (dt > 1e-6) {
            alpha_roll = (imu_roll - prev_imu_roll) / dt;
            alpha_pitch = (imu_pitch - prev_imu_pitch) / dt;
            alpha_yaw = (imu_yaw - prev_imu_yaw) / dt;
        } else {
            alpha_roll = 0.0;
            alpha_pitch = 0.0;
            alpha_yaw = 0.0;
        }
    } else {
        alpha_roll = 0.0;
        alpha_pitch = 0.0;
        alpha_yaw = 0.0;
        first_imu_received = true;
    }

    prev_imu_roll = imu_roll;
    prev_imu_pitch = imu_pitch;
    prev_imu_yaw = imu_yaw;
    prev_time = msg->header.stamp;

    // 假設功耗數據（需替換為實際感測器數據）
    double power = 0.0; // 單位：W，需從電流/電壓感測器獲取

    if (!recevice_once) {
        ofs << "x,y,z,acc_x,acc_y,acc_z,roll_rate,pitch_rate,yaw_rate,roll,pitch,yaw,"
            << "alpha_roll,alpha_pitch,alpha_yaw,command,power" << std::endl;
        recevice_once = true;
    }

    ofs << std::fixed << std::setprecision(3) 
        << current_pose_x << "," << current_pose_y << "," << current_pose_z
        << "," << imu_x << "," << imu_y << "," << imu_z 
        << "," << imu_roll << "," << imu_pitch << "," << imu_yaw 
        << "," << roll << "," << pitch << "," << yaw 
        << "," << alpha_roll << "," << alpha_pitch << "," << alpha_yaw 
        << "," << (int)current_command << "," << power
        << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_imu");
    ROS_INFO("sub_imu_start");
    WaypointSaver ws;
    ros::spin();
    return 0;
}
