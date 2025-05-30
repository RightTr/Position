#include "parameters.h"

float lidar2robot_x;
float lidar2robot_y;
float lidar2robot_dis;
float lidar2robot_ang;
float lidar2robot_ang_xx;
std::string odometry_topic;
std::string cluster_topic;
bool odometry_en;
bool cluster_en;

void ReadParameters(ros::NodeHandle &nh)
{
    nh.param<float>("robot/lidar2robot_x", lidar2robot_x, 0);
    nh.param<float>("robot/lidar2robot_y", lidar2robot_y, 0.24215);
    nh.param<float>("robot/lidar2robot_dis", lidar2robot_dis, 0.24215);
    nh.param<float>("robot/lidar2robot_ang", lidar2robot_ang, 90);
    nh.param<float>("robot/lidar2robot_ang_xx", lidar2robot_ang_xx, 0);
    nh.param<std::string>("topic/odometry_topic", odometry_topic, "/aft_mapped_to_init");
    nh.param<std::string>("topic/cluster_topic", cluster_topic, "/depth_clustering/clusters");
    nh.param<bool>("function/odometry_en", odometry_en, true);
    nh.param<bool>("function/cluster_en", cluster_en, false);
}