//
// Created by wlh on 18-5-19.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseArray.h>

#include <dynamic_reconfigure/server.h>
#include <loam_velodyne/calibConfig.h>

std::ifstream rtk_in, odom_in;
ros::Publisher pub_odom;
ros::Publisher pub_rtk;
ros::Publisher pub_calib;
geometry_msgs::PoseArray rtk_pose_v;
geometry_msgs::PoseArray odom_pose_v;

std::vector<double> rtk_stamps, odom_stamps;

double init_rtk_x = 0.0;
double init_rtk_y = 0.0;
double init_rtk_yaw = 0.0;

double calib_tx = 0.0;
double calib_ty = 0.0;
double calib_rz = 0.0;

struct TrajPoint{
    double time;
    double x;
    double y;
    double z;
    double yaw;
};

void splitStr(const std::string& s, const std::string& delim, std::vector< std::string >& ret){
    size_t last = 0;
    ret.clear();
    size_t index = s.find_first_of(delim, last);
    while (index != std::string::npos) {
        ret.push_back(s.substr(last, index - last));
        last = index + 1;
        index = s.find_first_of(delim, last);
    }
    if (index - last > 0){
        ret.push_back(s.substr(last, index - last));
    }
}

void localToMap(const double ref_x, const double ref_y, const double ref_yaw, double& loc_x, double& loc_y, double& loc_yaw)
{
    double x = loc_x * cos(ref_yaw) - loc_y * sin(ref_yaw) + ref_x;
    double y = loc_x * sin(ref_yaw) + loc_y * cos(ref_yaw) + ref_y;
    double yaw = loc_yaw + ref_yaw;
    if(yaw >= M_PI) yaw -= 2*M_PI;
    if(yaw < -M_PI) yaw += 2*M_PI;

    loc_x = x;
    loc_y = y;
    loc_yaw = yaw;
}

void configCallback(loam_velodyne::calibConfig &config, uint32_t level)
{
    calib_tx = config.tx;
    calib_ty = config.ty;
    calib_rz = config.rz/180.0*M_PI;
}

void timerCallback(const ros::TimerEvent& event)
{
    odom_pose_v.header.stamp = ros::Time::now();
    rtk_pose_v.header.stamp = ros::Time::now();


    geometry_msgs::PoseArray calib_pose_v = odom_pose_v;
    double ref_x = init_rtk_x;
    double ref_y = init_rtk_y;
    double ref_yaw = init_rtk_yaw;
    for(int i=0; i<odom_pose_v.poses.size(); ++i){

        double loc_x = calib_pose_v.poses[i].position.x;
        double loc_y = calib_pose_v.poses[i].position.y;
        double loc_z = calib_pose_v.poses[i].position.z;
        double loc_yaw = tf::getYaw(calib_pose_v.poses[i].orientation);

        loc_x += calib_tx;
        loc_y += calib_ty;
//        loc_yaw += calib_rz;

        double x = loc_x;
        double y = loc_y;

        loc_x = x*cos(calib_rz) - y*sin(calib_rz);
        loc_y = x*sin(calib_rz) + y*cos(calib_rz);

        localToMap( ref_x, ref_y, ref_yaw, loc_x, loc_y, loc_yaw );


        calib_pose_v.poses[i].position.x = loc_x;
        calib_pose_v.poses[i].position.y = loc_y;
        calib_pose_v.poses[i].position.z = loc_z;
        calib_pose_v.poses[i].orientation = tf::createQuaternionMsgFromYaw(loc_yaw);
    }



    calib_pose_v.header.stamp = ros::Time::now();

    pub_odom.publish(odom_pose_v);
    pub_rtk.publish(rtk_pose_v);
    pub_calib.publish(calib_pose_v);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_baselink");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");


    rtk_in.open("/home/wlh/catkin_ws/src/3d_party/loam_velodyne/calib_file/rtk.txt");
    if(!rtk_in.is_open()){
        std::cerr<<"open rtk file failed";
        return -1;
    }

    odom_in.open("/home/wlh/catkin_ws/src/3d_party/loam_velodyne/calib_file/odom.txt");
    if(!odom_in.is_open()){
        std::cerr<<"open odom file failed";
        return -1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////

    rtk_pose_v.header.frame_id = "lidar";
    std::string rtk_str;
    std::vector<std::string> rtk_str_v;
    while( getline(rtk_in,rtk_str) ){
        splitStr(rtk_str, ",", rtk_str_v);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp.fromSec( atof(rtk_str_v[0].c_str()) );
        pose.header.frame_id = "lidar";
        pose.pose.position.x = atof(rtk_str_v[1].c_str());
        pose.pose.position.y = atof(rtk_str_v[2].c_str());
        pose.pose.position.z = atof(rtk_str_v[3].c_str());
        pose.pose.orientation = tf::createQuaternionMsgFromYaw( atof(rtk_str_v[4].c_str()) );
        rtk_pose_v.poses.push_back(pose.pose);

        rtk_stamps.push_back(atof(rtk_str_v[0].c_str()));
    }

    std::cout<<std::setprecision(20)<<"rtk t: "<<rtk_stamps[0]<<std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////

    odom_pose_v.header.frame_id = "lidar";

    std::string str;
    std::vector<std::string> str_v;
    while( getline(odom_in,str) ){
        splitStr(str, ",", str_v);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp.fromSec( atof(str_v[0].c_str()) );
        pose.header.frame_id = "lidar";
        pose.pose.position.x = atof(str_v[1].c_str());
        pose.pose.position.y = atof(str_v[2].c_str());
        pose.pose.position.z = atof(str_v[3].c_str());
        pose.pose.orientation = tf::createQuaternionMsgFromYaw( atof(str_v[4].c_str()) );
        odom_pose_v.poses.push_back(pose.pose);

        odom_stamps.push_back( atof(str_v[0].c_str()) );
    }

    std::cout<<std::setprecision(20)<<"odom t: "<<odom_stamps[0]<<std::endl;


    ///////////////////////////////////////////////////////////////////////////
    // looking for init rtk pose /////////////////////////////////////////////

    double idx0, idx1;
    for(int i=0; i<rtk_stamps.size(); ++i){
        if(rtk_stamps[i]>odom_stamps[0]){
            idx1 = i;
            idx0 = i-1;
            break;
        }
    }

    std::cout<<"x0: "<<rtk_pose_v.poses[idx0].position.x
             <<" y0: "<<rtk_pose_v.poses[idx0].position.y
             <<" yaw0: "<<tf::getYaw(rtk_pose_v.poses[idx0].orientation)
             <<" t0: "<<rtk_stamps[idx0]<<std::endl;

    std::cout<<"x1: "<<rtk_pose_v.poses[idx1].position.x
             <<" y1: "<<rtk_pose_v.poses[idx1].position.y
             <<" yaw1: "<<tf::getYaw(rtk_pose_v.poses[idx1].orientation)
             <<" t1: "<<rtk_stamps[idx1]<<std::endl;

    init_rtk_x = rtk_pose_v.poses[idx0].position.x;
    init_rtk_y = rtk_pose_v.poses[idx0].position.y;
    init_rtk_yaw = tf::getYaw(rtk_pose_v.poses[idx0].orientation);

    ///////////////////////////////////////////////////////////////////////////////////

    ros::Rate r(ros::Duration(0.1));
    pub_odom = nh.advertise<geometry_msgs::PoseArray>("slam_traj",10);
    pub_rtk = nh.advertise<geometry_msgs::PoseArray>("rtk_traj",10);
    pub_calib = nh.advertise<geometry_msgs::PoseArray>("calib_traj",10);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    dynamic_reconfigure::Server<loam_velodyne::calibConfig> dr_srv;
    dynamic_reconfigure::Server<loam_velodyne::calibConfig>::CallbackType cb;
    cb = boost::bind(&configCallback, _1, _2);
    dr_srv.setCallback(cb);

    ros::spin();

    return 0;
}
