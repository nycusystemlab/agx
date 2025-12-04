#include <ros/ros.h>
#include <fstream>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
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
    ros::Publisher imu_pub_;
    ros::Publisher imu_pub_arduino;
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

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/processed_imu", 100);
    imu_pub_arduino = nh_.advertise<std_msgs::Float32MultiArray>("/imu_pub_arduino", 1);
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
    imu_pitch = (msg->angular_velocity.x) * -1;
    imu_yaw = (msg->angular_velocity.y) * -1;

    // 發布處理後的 IMU 數據
    // sensor_msgs::Imu imu_msg;
    // imu_msg.header = msg->header; // 繼承原始消息的 header（時間戳和框架）
    // imu_msg.linear_acceleration.x = imu_x;
    // imu_msg.linear_acceleration.y = imu_y;
    // imu_msg.linear_acceleration.z = imu_z;
    // imu_msg.angular_velocity.x = imu_roll;
    // imu_msg.angular_velocity.y = imu_pitch;
    // imu_msg.angular_velocity.z = imu_yaw;
    // imu_msg.orientation = msg->orientation; // 保留原始四元数
    // imu_pub_.publish(imu_msg);

    //補償的
    sensor_msgs::Imu imu_msg;
    imu_msg.header = msg->header;
    imu_msg.linear_acceleration.x = imu_x + 0.33;  // Adjusted axes
    imu_msg.linear_acceleration.y = imu_y - 0.40;
    imu_msg.linear_acceleration.z = imu_z + 0.12;
    imu_msg.angular_velocity.x = imu_roll;
    imu_msg.angular_velocity.y = imu_pitch;
    imu_msg.angular_velocity.z = imu_yaw;
    imu_pub_.publish(imu_msg);

    std_msgs::Float32MultiArray array_msg;
    array_msg.data = {static_cast<float>(imu_x + 0.33),
                        static_cast<float>(imu_y - 0.40),
                        static_cast<float>(imu_z + 0.12),
                        static_cast<float>(imu_roll),
                        static_cast<float>(imu_pitch),
                        static_cast<float>(imu_yaw)};
    imu_pub_arduino.publish(array_msg);

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

    if(!recevice_once) //bool為false 則符合條件
    {    
        ofs << "x,y,z,acc_x,acc_y,acc_z,roll_rate,pitch_rate,yaw_rate,roll,pitch,yaw"<< std::endl;  
        ofs << std::fixed << std::setprecision(3) 
        << current_pose_x << "," << current_pose_y << "," << current_pose_z
        << "," << imu_x << "," << imu_y << "," << imu_z 
        << "," << imu_roll << "," << imu_pitch << "," << imu_yaw << "," 
        << roll << "," << pitch << "," << yaw << std::endl;    

        recevice_once = true;
    }
    else
    {
        ofs << std::fixed << std::setprecision(3) 
        << current_pose_x << "," << current_pose_y << "," << current_pose_z
        << "," << imu_x << "," << imu_y << "," << imu_z 
        << "," << imu_roll << "," << imu_pitch << "," << imu_yaw 
        << "," << roll << "," << pitch << "," << yaw << std::endl; 
    }

    // ofs << std::fixed << std::setprecision(3) 
    //     << current_pose_x << "," << current_pose_y << "," << current_pose_z
    //     << "," << imu_x << "," << imu_y << "," << imu_z 
    //     << "," << imu_roll << "," << imu_pitch << "," << imu_yaw 
    //     << "," << roll << "," << pitch << "," << yaw 
    //     << "," << alpha_roll << "," << alpha_pitch << "," << alpha_yaw 
    //     << "," << (int)current_command 
    //     << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_imu");
    ROS_INFO("sub_imu_start");
    WaypointSaver ws;
    ros::spin();
    return 0;
}
