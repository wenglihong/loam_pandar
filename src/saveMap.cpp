//
// Created by hl on 18-2-26.
//
#include <cmath>
#include <mrpt/maps/CSimplePointsMap.h>
#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
using namespace std;
using namespace mrpt;

//mrpt simple points map
mrpt::maps::CSimplePointsMap global_points_map;
mrpt::maps::CSimplePointsMap local_points_map;
mrpt::maps::CSimplePointsMap current_points_map;
mrpt::maps::CSimplePointsMap temp_points_map;

//
int frame_id=0;
int id=0;
pcl::PointCloud<PointType>::Ptr Map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloud2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
Eigen::Matrix4f before_node = Eigen::Matrix4f::Identity();
Eigen::Matrix4f change_node = Eigen::Matrix4f::Identity();
std::ofstream node;
int i=1;
inline Eigen::Matrix4f GetMatrixLH(double x,double y,double z,double w)
{
    Eigen::Matrix4f ret;
    double xx = x*x;
    double yy = y*y;
    double zz = z*z;
    double xy = x*y;
    double wz = w*z;
    double wy = w*y;
    double xz = x*z;
    double yz = y*z;
    double wx = w*x;

    ret (0,0) = 1.0f-2*(yy+zz);
    ret(0,1)= 2*(xy-wz);
    ret(0,2) = 2*(wy+xz);
    ret(0,3) = 0.0f;

    ret(1,0)= 2*(xy+wz);
    ret(1,1) = 1.0f-2*(xx+zz);
    ret(1,2) = 2*(yz-wx);
    ret(1,3)= 0.0f;

    ret(2,0) = 2*(xz-wy);
    ret(2,1) = 2*(yz+wx);
    ret(2,2) = 1.0f-2*(xx+yy);
    ret(2,3) = 0.0f;

    ret(3,0) = 0.0f;
    ret(3,1) = 0.0f;
    ret(3,2) = 0.0f;
    ret(3,3)= 1.0f;

    return ret;
};

void odomHandler(const nav_msgs::Odometry::ConstPtr &laserodom)
{

    before_node=GetMatrixLH(
                laserodom->pose.pose.orientation.x,
                laserodom->pose.pose.orientation.y,
                laserodom->pose.pose.orientation.z,
                laserodom->pose.pose.orientation.w);
        before_node(0,3)=laserodom->pose.pose.position.x;
        before_node(1,3)=laserodom->pose.pose.position.y;
        before_node(2,3)=laserodom->pose.pose.position.z;

    change_node=GetMatrixLH(
            laserodom->pose.pose.orientation.x,
            laserodom->pose.pose.orientation.y,
            laserodom->pose.pose.orientation.z,
            laserodom->pose.pose.orientation.w);
        change_node(0,3)= laserodom->pose.pose.position.x;
        change_node(1,3)= laserodom->pose.pose.position.y;
        change_node(2,3)= 0;


}

void mapHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud)
{
    pcl::fromROSMsg(*laserCloud, *laserCloudIn);

    for(int i=0; i<laserCloudIn->points.size(); ++i){
        PointType point;
        point.x = laserCloudIn->points[i].z;
        point.y = laserCloudIn->points[i].x;
        point.z = laserCloudIn->points[i].y;

        if(point.z>0.2)
            laserCloud2->points.push_back(point);
    }

    pcl::transformPointCloud(*laserCloud2,*laserCloud2,change_node);
    before_node = Eigen::Matrix4f::Identity();
    change_node = Eigen::Matrix4f::Identity();
    *Map+=*laserCloud2;
    frame_id++;
    laserCloud2->clear();

    int temp = 80;
    if(frame_id==temp+temp*id)
    {
        //ros::waitForShutdown();
        sensor_msgs::PointCloud2 g_output;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toROSMsg(*Map, g_output);
        pcl_conversions::toPCL(g_output,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ> pcl_pc;
        pcl::fromPCLPointCloud2(pcl_pc2,pcl_pc);
        global_points_map.setFromPCLPointCloud(pcl_pc);
        ROS_INFO("points map size %i", (int)global_points_map.size());
        ROS_INFO("saving ply file");
        stringstream ss;
        ss<<i;

        string s1="/home/wlh/cloud_map/20180520/";
        string s2 = ss.str();
        string s3=".ply";
        std::string filename=s1+s2+s3;
        global_points_map.saveToPlyFile(filename,1);
        i++;
        Map->clear();
        ROS_INFO("ply file saved successfully");
        id++;

    }

}
void fix_odom_first_callback(const nav_msgs::Odometry::ConstPtr &FixOdomFirstIn)
{
    change_node=GetMatrixLH(
            FixOdomFirstIn->pose.pose.orientation.x,
            FixOdomFirstIn->pose.pose.orientation.y,
            FixOdomFirstIn->pose.pose.orientation.z,
            FixOdomFirstIn->pose.pose.orientation.w);
    change_node(0,3)= FixOdomFirstIn->pose.pose.position.x;
    change_node(1,3)= FixOdomFirstIn->pose.pose.position.y;
    change_node(2,3)= 0;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "saveMap");
    ros::NodeHandle nh;

//    node.open ("/home/guolindong/cyberfly_ws/src/loam_velodyne/txt/node.txt");


    ros::Subscriber subLaserMap = nh.subscribe<sensor_msgs::PointCloud2>
            ("/feature_points", 5, mapHandler);

//    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
//            ("/aft_mapped_to_init", 5, odomHandler);

//    ros::Subscriber fix_odom_first_=nh.subscribe("/gps_constraint_first",100,fix_odom_first_callback);
    ros::spin();




    return 0;
}

