#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include <iomanip>
#include <cmath>
#include <std_msgs/Float32.h>

// =======================================================
// DataCollect : 純資料蒐集 + 特徵工程 + 能耗估測
// =======================================================
class DataCollect {
public:
    DataCollect();
    ~DataCollect();

private:
    // ROS callback
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void timerCallback(const ros::TimerEvent&);
    void minLaserCallback(const std_msgs::Float32ConstPtr& msg);

    void saveData();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber min_laser_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber motion_array_sub_;
    ros::Publisher imu_float_pub_;
    ros::Timer timer_20hz_;

    std::string filename_;
    std::ofstream ofs_;

    // ===================== KF 參數 =====================
    const float Q_kf_linear_  = 0.01f;
    const float R_kf_linear_  = 0.01f;
    const float Q_kf_angular_ = 0.05f;
    const float R_kf_angular_ = 0.8f;

    bool  kf_init_acc_[3];
    float x_kf_acc_[3];
    float P_kf_acc_[3];

    bool  kf_init_gyro_[3];
    float x_kf_gyro_[3];
    float P_kf_gyro_[3];

    // ===================== IMU =====================
    double imu_acc_[3];
    double imu_gyro_[3];
    double imu_rpy_[3];

    // ===================== Odom =====================
    double pose_xyz_[3];
    double pose_rpy_[3];

    // ===================== Motion (4D) =====================
    double v_real_;
    double a_real_;

    //====================== LiDAR ======================
    double min_laser_ = 10.0;

    // ===================== Angular acceleration =====================
    float ang_acc_[3];
    float prev_kf_gyro_[3];

    // ===================== Power =====================
    float power_regression_;
    float power_regression_imu_;
    float power_regression_test_;
    float power_regression_test1_;
    float power_regression_test2_;
};

// =======================================================
// Constructor
// =======================================================

DataCollect::DataCollect() : private_nh_("~") {
    private_nh_.param<std::string>(                  //存檔
        "save_filename",
        filename_,
        "/root/outdoor_inference_data_collect.csv"
    );

    ofs_.open(filename_, std::ios::app);
    // ofs_.open(filename_, std::ios::out | std::ios::trunc);
    if (ofs_.tellp() == 0) {
        ofs_ << "timestamp,"
             << "imu_ax,imu_ay,imu_az,"
             << "kf_ax,kf_ay,kf_az,"
             << "gyro_r,gyro_p,gyro_y,"
             << "kf_gyro_r,kf_gyro_p,kf_gyro_y,"
             << "ang_acc_r,ang_acc_p,ang_acc_y,"
             << "v_real,a_real,"
             << "pose_x,pose_y,pose_yaw,"
            //  << "power_imu,power_test,power_test1,power_test2,"
             << "power_imu,"
             << "min_laser\n";
    }

    for (int i = 0; i < 3; ++i) {
        kf_init_acc_[i]  = false;
        kf_init_gyro_[i] = false;
        x_kf_acc_[i]     = 0.0f;
        x_kf_gyro_[i]    = 0.0f;
        P_kf_acc_[i]     = 1.0f;
        P_kf_gyro_[i]    = 1.0f;
        prev_kf_gyro_[i] = 0.0f;
        ang_acc_[i]      = 0.0f;
    }

    imu_sub_ = nh_.subscribe("/imu/data", 200, &DataCollect::imuCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 50, &DataCollect::odomCallback, this);
    motion_array_sub_ = nh_.subscribe("rl_motion_array", 50, &DataCollect::motionArrayCallback, this);
    min_laser_sub_ = nh_.subscribe("/min_laser", 50, &DataCollect::minLaserCallback, this);
    
    imu_float_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
        "processed_imu_float", 10
    );

    timer_20hz_ = nh_.createTimer(
        ros::Duration(0.05),
        &DataCollect::timerCallback,
        this
    );
}

DataCollect::~DataCollect() {
    if (ofs_.is_open()) ofs_.close();
}

// =======================================================
// IMU callback : only KF update
// =======================================================
void DataCollect::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    imu_acc_[0] =  msg->linear_acceleration.z;  // imu x 方向線加速度
    imu_acc_[1] = -msg->linear_acceleration.x;  // imu y 方向線加速度
    imu_acc_[2] = -msg->linear_acceleration.y;  // imu z 方向線加速度

    imu_gyro_[0] =  msg->angular_velocity.z;    // imu roll rate
    imu_gyro_[1] = -msg->angular_velocity.x;    // imu pitch rate
    imu_gyro_[2] = -msg->angular_velocity.y;    // imu yaw rate

    for (int i = 0; i < 3; ++i) {
        if (!kf_init_acc_[i]) {
            x_kf_acc_[i] = imu_acc_[i];
            kf_init_acc_[i] = true;
        } else {
            float Pp = P_kf_acc_[i] + Q_kf_linear_;
            float K  = Pp / (Pp + R_kf_linear_);
            x_kf_acc_[i] += K * (imu_acc_[i] - x_kf_acc_[i]);
            P_kf_acc_[i]  = (1 - K) * Pp;
        }

        if (!kf_init_gyro_[i]) {
            x_kf_gyro_[i] = imu_gyro_[i];
            kf_init_gyro_[i] = true;
        } else {
            float Pp = P_kf_gyro_[i] + Q_kf_angular_;
            float K  = Pp / (Pp + R_kf_angular_);
            x_kf_gyro_[i] += K * (imu_gyro_[i] - x_kf_gyro_[i]);
            P_kf_gyro_[i]  = (1 - K) * Pp;
        }
    }

    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf::Matrix3x3(q).getRPY(
        imu_rpy_[0], imu_rpy_[1], imu_rpy_[2]
        // IMU roll pith yaw
    );
}

// =======================================================
void DataCollect::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    pose_xyz_[0] = msg->pose.pose.position.x;
    pose_xyz_[1] = msg->pose.pose.position.y;
    pose_xyz_[2] = msg->pose.pose.position.z;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3(q).getRPY(
        pose_rpy_[0], pose_rpy_[1], pose_rpy_[2]
    );
}

void DataCollect::minLaserCallback(const std_msgs::Float32ConstPtr& msg)
{
    min_laser_ = msg->data;
}

// =======================================================
void DataCollect::motionArrayCallback(const std_msgs::Float32MultiArrayConstPtr &msg) {
    if (msg->data.size() != 2) return;

    v_real_  = msg->data[0];
    a_real_  = msg->data[1];

}

// =======================================================
// Timer @ 20 Hz : derivative + power + log
// =======================================================
void DataCollect::timerCallback(const ros::TimerEvent&) {
    const float dt = 0.05f;
    float c0_set = 0;
    for (int i = 0; i < 3; ++i) {
        ang_acc_[i] = (x_kf_gyro_[i] - prev_kf_gyro_[i]) / dt;
        prev_kf_gyro_[i] = x_kf_gyro_[i];
    }

    float coeff[9] = {28.92f, 300.96f, 92.8f, -3.5f, 298.6f, 11.13f, -8.16f, 1037.04f, 956.38f};
    float v = std::fabs(v_real_);
    float slope = std::sin(imu_rpy_[0] + 1.62);
    float c1_2nd = 87.5f, c1_3rd = 75.5;

    if(v > 0 && fabs(x_kf_gyro_[2]) > 0){        // differential steering
            c0_set = c1_3rd;
    }else{
        c0_set = coeff[0];
    }
    
    power_regression_ =
        c0_set +
        coeff[1] * v +
        coeff[4] * std::fabs(x_kf_gyro_[2]) +
        coeff[5] * std::max(ang_acc_[2], 0.0f) * std::fabs(x_kf_gyro_[2]) +
        coeff[6] * std::min(ang_acc_[2], 0.0f) * std::fabs(x_kf_gyro_[2]) +
        coeff[8] * std::min(slope, 0.0f) * v;

    power_regression_imu_ =
        power_regression_ +
        coeff[2] * std::max(a_real_, 0.0) * v +
        coeff[3] * std::min(a_real_, 0.0) * v +
        coeff[7] * std::max(slope, 0.0f) * v;

    // power_regression_test_  = power_regression_imu_;
    // power_regression_test1_ = power_regression_imu_;
    // power_regression_test2_ = power_regression_imu_;

    std_msgs::Float32MultiArray msg;
    msg.data = {
        ang_acc_[0], ang_acc_[1], ang_acc_[2],
        x_kf_gyro_[0], x_kf_gyro_[1], x_kf_gyro_[2]
    };
    imu_float_pub_.publish(msg);

    saveData();
}

// =======================================================
void DataCollect::saveData() {
    ofs_ << std::fixed << std::setprecision(4)
         << ros::Time::now().toSec() << ","
         << imu_acc_[0] << "," << imu_acc_[1] << "," << imu_acc_[2] << ","
         << x_kf_acc_[0] << "," << x_kf_acc_[1] << "," << x_kf_acc_[2] << ","
         << imu_gyro_[0] << "," << imu_gyro_[1] << "," << imu_gyro_[2] << ","
         << x_kf_gyro_[0] << "," << x_kf_gyro_[1] << "," << x_kf_gyro_[2] << ","
         << ang_acc_[0] << "," << ang_acc_[1] << "," << ang_acc_[2] << ","
         << v_real_ << "," << a_real_ << "," 
         << pose_xyz_[0] << "," << pose_xyz_[1] << "," << pose_rpy_[2] << ","
         << power_regression_imu_ << ","
        //  << power_regression_test_ << ","
        //  << power_regression_test1_ << ","
        //  << power_regression_test2_ << ","
         << min_laser_ 
         << "\n";
}

// =======================================================
int main(int argc, char **argv) {
    ros::init(argc, argv, "data_collection_node");
    DataCollect node;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
