#include <iostream>
#include <deque>
#include <algorithm>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "customized_time.h"


// #include "local_trajectory_builder_options.pb.h"
// #include "kalman_local_trajectory_builder_options.ph.h"
#include "kalman_local_trajectory_builder_options.pb.h"

#include "kalman_local_trajectory_builder.h"

#include "AddressBook.pb.h"

// #include "customized_time.h"
#include "kalman_local_trajectory_builder.h"
#include "rigid_transform.h"
#include "unscented_kalman_filter.h"


// #include "cartographer/common/lua_parameter_dictionary.h"
// #include "cartographer/mapping_3d/proto/kalman_local_trajectory_builder_options.pb.h"

std::deque<std::pair<common::Time, nav_msgs::Odometry> > odom_d;
std::deque<std::pair<common::Time, sensor_msgs::Imu> > imu_d;

kalman_filter::KalmanLocalTrajectoryBuilder kf_builder_;

ros::Publisher ukf_pose_pub;

common::Time fromRos(const ros::Time& time)
{
      return common::FromUniversal((time.sec +common::kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll +
      (time.nsec + 50) / 100); 
}

void publishPose(const common::Time time)
{
    transform::Rigid3d pose_estimate = kf_builder_.GetPose(time);
    std::cout << "[main][pose_extimate.transpose():] " << pose_estimate.translation().transpose() << std::endl;
    // kalman_filter::PoseCovariance unused_covariance_estimate;
    // kf_builder_.pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_estimate, &unused_covariance_estimate);
    geometry_msgs::PoseStamped ukf_pose;
    ukf_pose.header.stamp = ros::Time::now();
    ukf_pose.header.frame_id = "/map";
    ukf_pose.pose.position.x = pose_estimate.translation().x();
    ukf_pose.pose.position.y = pose_estimate.translation().y();
    ukf_pose.pose.position.z = pose_estimate.translation().z();
    ukf_pose.pose.orientation.x = pose_estimate.rotation().x();
    ukf_pose.pose.orientation.y = pose_estimate.rotation().y();
    ukf_pose.pose.orientation.z = pose_estimate.rotation().z();
    ukf_pose.pose.orientation.w = pose_estimate.rotation().w();

    ukf_pose_pub.publish(ukf_pose);


}


void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    // uint64_t stamp = imu_msg->header.stamp.toNSec() ;
    // std::cout << "[main][stamp:] " << stamp << std::endl;
    // std::cout << "[main][stamp/ 1e9:] " << static_cast<long double>(stamp) / 1e9 << std::endl;
    // std::cout << "[main][imu_msg->header.stamp:] " << imu_msg->header.stamp << std::endl;


    common::Time time = fromRos(imu_msg->header.stamp);

    // std::cout << "[][std::chrono::milliseconds(milli):] " << std::chrono::milliseconds(stamp) << std::endl;
    // uint64_t     toNSec () 
    // ros::Time time = imu_msg->header.stamp.toSce;

    imu_d.push_back(std::make_pair(time, *imu_msg));
    std::cout << "[main][time:] " << time << std::endl;

    // Eigen::Vector3d angular_velocity = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
    //                                                       imu_msg->angular_velocity.z);
    // Eigen::Vector3d linear_acceleration = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
    //                                                       imu_msg->linear_acceleration.z);                                                          
    // kf_builder_.AddImuData(time, linear_acceleration, angular_velocity);
}

void odometerCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{


    common::Time time = fromRos(odom_msg->header.stamp);

    // transform::Rigid3d pose;

    // // transform::Rigid3d pose =  transform::Rigid3d(Eigen::Vector3d(-0.95, -0.05, 0.05),
    // //                      Eigen::AngleAxisd(0.05, Eigen::Vector3d(1., 0., 0.)));

    // Eigen::Vector3d position = Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
    //                                            odom_msg->pose.pose.position.z) ;
                                               
    // Eigen::Quaterniond orientation = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
    //                                                     odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

    // pose = transform::Rigid3d(position, orientation);

    // kf_builder_.AddOdometryData(time, pose);


    // ros::Time time = odom_msg->header.stamp.toSec;

    // std::cout << "[main][time:] " << time << std::endl;
    // std::cout << "[main][odom_msg->header.secs():] " << odom_msg->header.stamp << std::endl;
    // std::cout << "[][odom_msg->nsecs:] " << odom_msg->stamp.nsecs << std::endl;

    odom_d.push_back(std::make_pair(time, *odom_msg));


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

bool processSensorData()
{
    // std::cout << "[main][imu_d.size():] " << imu_d.size() << std::endl;
    // std::cout << "[main][odom_d.size():] " << odom_d.size() << std::endl;

    

    std::vector<common::Time> times(2, common::Time::max());

    if(odom_d.size() < 2 || imu_d.size() < 2)
        return false;



    

    // if(!odom_d.empty())
    // {
    //     times[0] = odom_d.front().first;
    // }
    // if(!imu_d.empty())
    // {
    //     times[1] = imu_d.front().first;
    // }
    times[0] = odom_d.front().first;
    times[1] = imu_d.front().first;

    int min_id = std::min_element(times.begin(), times.end()) - times.begin();

    // std::cout << "[test_00 main][time[0]:] " <<  odom_d.front().first << std::endl;
    // std::cout << "[test_00 main][time[1]:] " << imu_d.front().first << std::endl;
    // std::cout << "[test_00 main][times[min_id]:] " << min_id << ", " << times[min_id] << std::endl;

    // std::cout << "[main][min_id:] " << min_id << std::endl;
    // std::cout << "[main][odo_d.size():] " << odo_d.size() << std::endl;
    // std::cout << "[main][imu_d.size():] " << imu_d.size() << std::endl;
    // std::cout << "[main][common::Time::max():] " << common::Time::max() << std::endl;

    // common::Time time = common::Time::max();    
    // static common::Time old_time =  time;

    // if(times[min_id] == common::Time::max())
    // {
    //     return false;
    // }

    common::Time time;
    if(min_id == 0) //odometry
    {
        time = odom_d.front().first;
        nav_msgs::Odometry odom_msg = odom_d.front().second;
        Eigen::Vector3d position = Eigen::Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
                                               odom_msg.pose.pose.position.z) ;
                                               
        Eigen::Quaterniond orientation = Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, 
                                                        odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);

        transform::Rigid3d pose = transform::Rigid3d(position, orientation);

        std::cout << "[main]odometry process---------------------------odom" << std::endl;
        kf_builder_.AddOdometryData(time, pose);

        odom_d.pop_front();
    }
    else if(min_id == 1) // imu
    {
        time = imu_d.front().first;
        sensor_msgs::Imu imu_msg = imu_d.front().second;
        Eigen::Vector3d angular_velocity = Eigen::Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
                                                          imu_msg.angular_velocity.z);
        Eigen::Vector3d linear_acceleration = Eigen::Vector3d(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                                                          imu_msg.linear_acceleration.z);   

        std::cout << "[main]imu process---------------------------imu" << std::endl;
        // std::cout << "[main][times[min_id]:] " << times[min_id] << std::endl;

        kf_builder_.AddImuData(time, linear_acceleration, angular_velocity);

        imu_d.pop_front();
    }

    // std::cout << "[test_00 main][times[min_id]:] " << min_id << ", " << time << std::endl;
    publishPose(time);

    // old_time = time;
    return true;

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

    ukf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ukf_pose", 10);

    // ros::spin();

    ros::Rate loop_rate(60);
    while(ros::ok())
    {
        ros::spinOnce();
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