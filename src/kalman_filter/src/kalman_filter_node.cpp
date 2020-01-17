#include <iostream>
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


// #include "local_trajectory_builder_options.pb.h"
// #include "kalman_local_trajectory_builder_options.ph.h"
#include "kalman_local_trajectory_builder_options.pb.h"

#include "kalman_local_trajectory_builder.h"

#include "AddressBook.pb.h"

// #include "customized_time.h"
#include "kalman_local_trajectory_builder.h"
// #include "rigid_transform.h"


// #include "cartographer/common/lua_parameter_dictionary.h"
// #include "cartographer/mapping_3d/proto/kalman_local_trajectory_builder_options.pb.h"

std::deque< pair<double, geometry_msgs::Odometry> > odo_d;
std::deque< pair<double, sensor_msgs::Imu> > imu_d;

kalman_filter::KalmanLocalTrajectoryBuilder kf_builder_;


void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double time = imu_msg->header.stamp.toSec();
    // Eigen::Vector3d angular_velocity = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
    //                                                       imu_msg->angular_velocity.z);
    // Eigen::Vector3d linear_acceleration = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
    //                                                       imu_msg->linear_acceleration.z);                                                          
    // kf_builder_.AddImuData(time, linear_acceleration, angular_velocity);
}

void odometerCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    double time = odom_msg->header.stamp.toSec();
    




    // transform::Rigid3d pose;
    // // transform::Rigid3d pose =  transform::Rigid3d(Eigen::Vector3d(-0.95, -0.05, 0.05),
    // //                      Eigen::AngleAxisd(0.05, Eigen::Vector3d(1., 0., 0.)));

    // Eigen::Vector3d position = Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
    //                                            odom_msg->pose.pose.position.z) ;
                                               
    // Eigen::Quaterniond orientation = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
    //                                                     odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

    // pose = transform::Rigid3d(position, orientation);

    // // std::cout << "[][position.x():] " << position.x() << std::endl;
    // time = odom_msg->header.stamp.toSec();

    
    // kf_builder_.AddOdometryData(time, pose);

    // kf_builder_.AddOdometryData();
    // std::cout << "[odometerCallback][t:] " << t << std::endl;
}

void processSensorData()
{

}


// #include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman_filter");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;

    odom_sub = nh.subscribe("odom", 100, odometerCallback);
    imu_sub = nh.subscribe("imu", 100, imuCallback);

    ros::spin();

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        // ros::spinOnce();
        while(processSensorData())
        {

        }
        loop_rate.sleep();
    }
    return 0;



    // const proto::LocalTrajectoryBuilderOptions options_;
    // const cartographer::mapping_3d::proto::KalmanLocalTrajectoryBuilderOptions options_;
    // kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions options_;
    // options_.set_odometer_rotational_variance(11);
    // std::cout << "[][options_.odometer_rotational_variance():] " <<  options_.odometer_rotational_variance() << std::endl;


    // #include <chrono>

    // std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    // std::cout << "[][now:] " << ros::Time::now() << std::endl;

    

}