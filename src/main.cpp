#include "uart.h"
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "parameters.h"

#define MID360_IMU2LIDAR_X 0.011
#define MID360_IMU2LIDAR_Y 0.02329
#define MID360_IMU2LIDAR_DIS 0.025757
#define MID360_IMU2LIDAR_ANG atan2f(0.02329, 0.011)

using namespace com;
using namespace std;

UART uart1;

ros::Subscriber odometry_sub;
ros::Subscriber cluster_sub;

extern float lidar2robot_x;
extern float lidar2robot_y;
extern float lidar2robot_dis;
extern float lidar2robot_ang;
extern float lidar2robot_ang_xx;
extern std::string odometry_topic;
extern std::string cluster_topic;
extern bool odometry_en;
extern bool cluster_en;

float euler_last = 0 , euler_total = 0;
float euler_x, euler_z;
int k = 0;
float x_imu2lidar, y_imu2lidar;
float x_lidar2robot, y_lidar2robot;
float x_imu2lidar_vel, y_imu2lidar_vel;
float x_lidar2robot_vel, y_lidar2robot_vel;
float z_angular_vel;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d eulerAngles = q.toRotationMatrix().eulerAngles(2 ,1 ,0);

    euler_z = atan2(2 * (q.y() * q.x() + q.w() * q.z()),
                   1 - 2 * (q.z() * q.z() + q.y() * q.y()));

    x_imu2lidar = float(msg->pose.pose.position.x) + MID360_IMU2LIDAR_X - MID360_IMU2LIDAR_DIS * cosf(MID360_IMU2LIDAR_ANG + euler_z);
    y_imu2lidar = float(msg->pose.pose.position.y) + MID360_IMU2LIDAR_Y - MID360_IMU2LIDAR_DIS * sinf(MID360_IMU2LIDAR_ANG + euler_z); 
    x_imu2lidar_vel = float(msg->twist.twist.linear.x);
    y_imu2lidar_vel = float(msg->twist.twist.linear.y);

    if(lidar2robot_ang_xx == 0)
    {
        x_lidar2robot = x_imu2lidar + lidar2robot_x - lidar2robot_dis * cosf(lidar2robot_ang * (M_PI / 180.0) + euler_z);
        y_lidar2robot = y_imu2lidar + lidar2robot_y - lidar2robot_dis * sinf(lidar2robot_ang * (M_PI / 180.0) + euler_z);
    }
    else if(lidar2robot_ang_xx == 180 || lidar2robot_ang_xx == -180)
    {
        x_lidar2robot = -x_imu2lidar + lidar2robot_x - lidar2robot_dis * cosf(lidar2robot_ang * (M_PI / 180.0) + euler_z);
        y_lidar2robot = -y_imu2lidar + lidar2robot_y - lidar2robot_dis * sinf(lidar2robot_ang * (M_PI / 180.0) + euler_z);
    }
    else
    {
        x_lidar2robot = 0;
        y_lidar2robot = 0;
        cout << "Invalid Angle!" << endl;
    }
    x_lidar2robot_vel = x_imu2lidar_vel * cosf(euler_z) - y_imu2lidar_vel * sinf(euler_z);
    y_lidar2robot_vel = y_imu2lidar_vel * cosf(euler_z) + x_imu2lidar_vel * sinf(euler_z);
    
    z_angular_vel = float(msg->twist.twist.angular.z) * (180.0 / M_PI);

    euler_z = euler_z * (180.0 / M_PI);
    euler_x = eulerAngles[2] * (180.0 / M_PI);

    if(euler_z - euler_last > 180.0)
    {
        k--;
    }
    else if(euler_z - euler_last < -180.0)
    {
        k++;
    }
    euler_total = euler_z + 360.0 * k;
    
    uint8_t senddata[33] = {0};
    senddata[0] = 0xFF;
    senddata[1] = 0xFE;
    senddata[2] = 1;
    memcpy(&senddata[3], &x_lidar2robot, 4);
    memcpy(&senddata[7], &y_lidar2robot, 4);
    memcpy(&senddata[11], &euler_total, 4);
    memcpy(&senddata[15], &euler_x, 4);
    memcpy(&senddata[19], &x_lidar2robot_vel, 4);
    memcpy(&senddata[23], &y_lidar2robot_vel, 4);
    memcpy(&senddata[27], &z_angular_vel, 4);
    senddata[31] = 0xAA;
    senddata[32] = 0xDD;
    uart1.UART_SEND(senddata, 33);
    cout << "x:" << x_lidar2robot << ",y:" << y_lidar2robot << ",yaw:" << euler_z <<",yaw_total:" << euler_total << ",pitch:" << euler_x << endl;
    cout << "vel_x:" << x_lidar2robot_vel << ",vel_y:" << y_lidar2robot_vel << ",angvel_z:" << z_angular_vel << endl;

    euler_last = euler_z;
}

void ClustersCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    int clusters_amount = static_cast<int>(msg->data.size()) / 4;
    if(clusters_amount > 4)
    {
        cout << "Clusters Overflow!" << endl;
        return ;
    }
    if(clusters_amount == 0)
    {
    	return ;
    }
    Eigen::Vector4f clusters = Eigen::Vector4f::Zero();
    uint8_t senddata[12 * 4 + 5] = {0};
    for(int i = 0; i < clusters_amount; i++)
    {
        clusters = Eigen::Vector4f(msg->data[4 * i], msg->data[4 * i + 1], msg->data[4 * i + 2], msg->data[4 * i + 3]);
        clusters.y() -= lidar2robot_y;
        clusters.w() /= 2.0;
        memcpy(&senddata[12 * i + 3], &clusters.x(), 4);
        memcpy(&senddata[12 * i + 7], &clusters.y(), 4);
        memcpy(&senddata[12 * i + 11], &clusters.w(), 4);
        printf("Cluster NO.%d: x:%f, y:%f, z:%f, r:%f\n", i, clusters.x(), clusters.y(), clusters.z(), clusters.w());
    }
    
    senddata[0] = 0xFF;
    senddata[1] = 0xFE;
    senddata[2] = 2;
    senddata[12 * 4 + 3] = 0xAA;
    senddata[12 * 4 + 4] = 0xDD;
    uart1.UART_SEND_CLONE(senddata, 12 * 4 + 5);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "position");
    ros::NodeHandle nh;
    ReadParameters(nh);
    if(odometry_en)
    {
        odometry_sub = nh.subscribe<nav_msgs::Odometry>(odometry_topic, 1, OdomCallback);        
    }
    if(cluster_en)
    {
        cluster_sub = nh.subscribe<std_msgs::Float32MultiArray>(cluster_topic, 1, ClustersCallback);
    }
    ros::spin();
    return 0;
}
