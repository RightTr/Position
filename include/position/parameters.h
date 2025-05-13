#include <ros/ros.h>
#include <string>


extern float lidar2robot_x;
extern float lidar2robot_y;
extern float lidar2robot_dis;
extern float lidar2robot_ang;
extern float lidar2robot_ang_xx;
extern std::string odometry_topic;
extern std::string cluster_topic;
extern bool odometry_en;
extern bool cluster_en;
extern float delta_dis_max;
extern float delta_dis_min;

void ReadParameters(ros::NodeHandle &nh);