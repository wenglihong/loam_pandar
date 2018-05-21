//
// Created by wlh on 18-4-21.
//

//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>


using namespace std;

ros::Publisher pub_points;
pcl::PointCloud<pcl::PointXYZI> cloud;
//double tx = 0.84;
double tx = 0.82;
double ty = 0.05;
double tz = 2.13119;
double rx=-0.435276, ry=-2.0685, rz=-2.8;
//double tz = 2.16;
//double rx=-0.0, ry=0.0, rz=-4.0;

pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud_ring;


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << tx, ty, tz;
    transform.rotate(Eigen::AngleAxisf(rx*M_PI/180, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(ry*M_PI/180, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(rz*M_PI/180, Eigen::Vector3f::UnitZ()));

    pcl::fromROSMsg(*msg, cloud_ring);
    pcl::transformPointCloud(cloud_ring, cloud_ring, transform);


    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_ring, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "base_link";
    pub_points.publish(cloud_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "toBaseLink");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pub_points = nh.advertise<sensor_msgs::PointCloud2>("ponits_in_baselink",10);
    ros::Subscriber sub_points = nh.subscribe("/pandar_points", 10, pointCloudCallback);


    ros::spin();
    return 0;
}
