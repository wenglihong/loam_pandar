//
// Created by wlh on 18-5-21.
//


//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

//#include <tf2_ros/buffer.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


//std
#include <vector>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


using namespace std;

ros::Publisher pub_points;
pcl::PointCloud<pcl::PointXYZI> cloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

const int GRID_HEIGHT = 300;
const int GRID_WIDTH = 300;

void cut_x_y( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double x_min, double x_max, double y_min, double y_max )
{
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min,x_max);
    pass.filter(*cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min,y_max);
    pass.filter(*cloud);
}

void cut_z( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double z_min, double z_max )
{
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min,z_max);
    pass.filter(*cloud);
}

double get_max_height( pcl::PointCloud<pcl::PointXYZI> cloud )
{
    if( cloud.empty() ) return 0;

    double max_h = 0.0;
    for( int i=0; i<cloud.points.size(); i++ )
        if( max_h < cloud.points[i].z )
            max_h = cloud.points[i].z;

    return max_h;
}

double get_min_height( pcl::PointCloud<pcl::PointXYZI> cloud )
{
    if( cloud.empty() ) return 0;

    double min_h = 10.0;
    for( int i=0; i<cloud.points.size(); i++ )
        if( min_h > cloud.points[i].z )
            min_h = cloud.points[i].z;

    return min_h;
}

double min_max_limit( double val, double min, double max )
{
    val = val < min ? min : val;
    val = val > max ? max : val;
    return val;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, cloud);

    static tf::TransformListener tf_listener1, tf_listener2;
    tf::StampedTransform transform1;
//    tf_listener1.lookupTransform("aft_mapped", "camera_init", msg->header.stamp, transform1);
//    tf_listener1.waitForTransform ("aft_mapped", "camera_init", msg->header.stamp, ros::Duration(1.0));

    try{
        tf_listener1.lookupTransform("/aft_mapped", "/camera_init", ros::Time(0), transform1);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }


    tf::StampedTransform transform2;
//    tf_listener2.lookupTransform("camera_init", "aft_mapped", msg->header.stamp, transform2);
//    tf_listener2.waitForTransform ("camera_init", "aft_mapped", msg->header.stamp, ros::Duration(1.0));
    try{
        tf_listener2.lookupTransform("/camera_init", "/aft_mapped", ros::Time(0), transform2);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//    geometry_msgs::TransformStamped transformStamped;
//    transformStamped = tfBuffer.lookupTransform("aft_mapped", "camera_init",ros::Time(0));
//    pcl_ros::transformPointCloud("aft_mapped", cloud, *lidar_points, tf_listener1);
//    tf2::doTransform(cloud, *lidar_points, transformStamped);
//    std::cout<<"receive point cloud: "<<cloud.points.size()<<std::endl;

    pcl_ros::transformPointCloud(cloud, *lidar_points, transform1);

    for(int i=0; i<lidar_points->points.size(); ++i){
        pcl::PointXYZI point;
        point.x = lidar_points->points[i].z;
        point.y = lidar_points->points[i].x;
        point.z = lidar_points->points[i].y;

        lidar_points->points[i] = point;
    }



    cut_x_y( lidar_points, -30, 30, -30, 30 );

    vector<vector<pcl::PointCloud<pcl::PointXYZI> > > cloud_grid_array;
    cloud_grid_array.resize(300);
    for (int i = 0; i < 300; ++i)
        cloud_grid_array[i].resize(300);

    for (int i = 0; i < lidar_points->points.size(); i++)
    {
        int colum = 0;
        int row = 0;

        colum = int( ( 30 - lidar_points->points[i].y ) / 0.2);
        row = int( ( 30 - lidar_points->points[i].x ) / 0.2);

        if (row >= GRID_HEIGHT) row = GRID_HEIGHT-1;
        if (colum >= GRID_WIDTH) colum = GRID_WIDTH-1;

        cloud_grid_array[row][colum].points.push_back(lidar_points->points[i]);

    }

    cloud_filtered->clear();
    for(int i=0; i<GRID_HEIGHT; i++)
        for(int j=0; j<GRID_WIDTH; j++)
        {
            if( cloud_grid_array[i][j].points.size() > 40 )
            {
                double max_h = get_max_height( cloud_grid_array[i][j] );
                double min_h = get_min_height( cloud_grid_array[i][j] );
                if( max_h > 2.0  && min_h < 1.5 )
                    cloud_grid_array[i][j].header.seq = 1;
            }
        }



    for(int i=1; i<GRID_HEIGHT-1; i++)
        for(int j=1; j<GRID_WIDTH-1; j++)
            if( cloud_grid_array[i][j].header.seq == 1 ){
                *cloud_filtered += cloud_grid_array[i][j];
//                poles.pole.push_back(createPole(cloud_grid_array[i][j],pole_id++));
            }



    for(int i=0; i<cloud_filtered->points.size(); ++i){
        pcl::PointXYZI point;
        point.x = cloud_filtered->points[i].y;
        point.y = cloud_filtered->points[i].z;
        point.z = cloud_filtered->points[i].x;

        cloud_filtered->points[i] = point;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    pcl_ros::transformPointCloud(*cloud_filtered, cloud_out, transform1.inverse());

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_out, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "camera_init";
    pub_points.publish(cloud_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_detection_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber sub_points = nh.subscribe("/velodyne_cloud_registered", 10, pointCloudCallback);
    pub_points = nh.advertise<sensor_msgs::PointCloud2>("/feature_points", 10);

    ros::spin();
    return 0;
}