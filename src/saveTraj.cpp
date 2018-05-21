//
// Created by wlh on 18-5-19.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <tf/transform_datatypes.h>

std::ofstream out_rtk, out_odom;

void rtkPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    out_rtk<<std::setprecision(20)<<msg.header.stamp.toSec() <<","
            <<msg.pose.position.x<<","
            <<msg.pose.position.y<<","
            <<msg.pose.position.z<<","
            <<tf::getYaw(msg.pose.orientation)
            <<std::endl;
}

void odomCallback(const nav_msgs::Odometry& msg)
{
    out_odom<<std::setprecision(20)<<msg.header.stamp.toSec() <<","
            <<msg.pose.pose.position.x<<","
            <<msg.pose.pose.position.y<<","
            <<msg.pose.pose.position.z<<","
            <<tf::getYaw(msg.pose.pose.orientation)
            <<std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_baselink");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber sub_points = nh.subscribe("/rtk_pose", 10, rtkPoseCallback);
    ros::Subscriber sub_odom = nh.subscribe("/integrated_to_init",10, odomCallback);

    out_rtk.open("/home/wlh/catkin_ws/src/3d_party/loam_velodyne/calib_file/rtk.txt", std::ios_base::app);
    if(!out_rtk.is_open()){
        std::cerr<<"open rtk file failed";
        return -1;
    }

    out_odom.open("/home/wlh/catkin_ws/src/3d_party/loam_velodyne/calib_file/odom.txt",std::ios_base::app);
    if(!out_odom.is_open()){
        std::cerr<<"open odom file failed";
        return -1;
    }

    ros::spin();
    return 0;
}
