#include "uart.h"
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#define PI 3.1415927
#define LIDAR2ROBOT 0.24215
#define IMU2LIDAR_DISTANCE_X 0.011
#define IMU2LIDAR_DISTANCE_Y 0.02329
#define IMU2LIDAR_DISTANCE 0.025757
#define IMU2LIDAR_ANGLE atan2f(0.02329, 0.011)

using namespace com;
using namespace std;

UART uart1;

float euler_last = 0 , euler_total = 0;
float euler_x, euler_z;
int k = 0;
float x_imu2lidar, y_imu2lidar;
float x_lidar2robot, y_lidar2robot;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d eulerAngles = q.toRotationMatrix().eulerAngles(2 ,1 ,0);

    euler_z = atan2(2 * (q.y() * q.x() + q.w() * q.z()),
                   1 - 2 * (q.z() * q.z() + q.y() * q.y()));

    x_imu2lidar = float(msg->pose.pose.position.x) + IMU2LIDAR_DISTANCE_X - IMU2LIDAR_DISTANCE * cosf(IMU2LIDAR_ANGLE + euler_z);
    y_imu2lidar = float(msg->pose.pose.position.y) + IMU2LIDAR_DISTANCE_Y - IMU2LIDAR_DISTANCE * sinf(IMU2LIDAR_ANGLE + euler_z); 

    x_lidar2robot = x_imu2lidar - LIDAR2ROBOT * cosf(PI / 2 + euler_z);
    y_lidar2robot = y_imu2lidar + LIDAR2ROBOT - LIDAR2ROBOT * sinf(PI / 2 + euler_z);
    
    euler_z = euler_z * (180.0 / M_PI);
    euler_x = eulerAngles[2] * (180.0 / M_PI);

    if(euler_z - euler_last > 180.0 )
    {
        k--;
    }
    else if(euler_z - euler_last < -180.0 )
    {
        k++;
    }
    euler_total = euler_z + 360.0 * k;
    
    uint8_t senddata[21] = {0};
    senddata[0] = 0xFF;
    senddata[1] = 0xFE;
    senddata[2] = 1;
    memcpy(&senddata[3], &x_lidar2robot, 4);
    memcpy(&senddata[7], &y_lidar2robot, 4);
    memcpy(&senddata[11], &euler_total, 4);
    memcpy(&senddata[15], &euler_x, 4);
    senddata[19] = 0xAA;
    senddata[20] = 0xDD;
    uart1.UART_SEND(senddata, 21);
    ROS_INFO("Pose:");
    cout << "x:" << x_lidar2robot << ",y:" << y_lidar2robot << ",yaw:" << euler_z <<",yaw_total:" << euler_total << ",pitch:" << euler_x <<endl;

    euler_last = euler_z;
}

void ClustersCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    int clusters_amount = static_cast<int>(msg->data.size()) / 4;
    if(clusters_amount > 5)
    {
        cout << "Clusters Overflow!" << endl;
        return ;
    }
    Eigen::Vector4f clusters = Eigen::Vector4f::Zero();
    uint8_t senddata[3 * clusters_amount + 5] = {0};
    for(int i = 0; i < clusters_amount; i++)
    {
        clusters = Eigen::Vector4f(msg->data[4 * i], msg->data[4 * i + 1], msg->data[4 * i + 2], msg->data[4 * i + 3]);
        clusters.y() -= LIDAR2ROBOT;
        clusters.w() /= 2.0;
        memcpy(&senddata[3 * i + 3], &clusters.x(), 4);
        memcpy(&senddata[3 * i + 4], &clusters.y(), 4);
        memcpy(&senddata[3 * i + 5], &clusters.w(), 4);
        printf("Cluster NO.%d: x:%f, y:%f, z:%f, r:%f\n", i, clusters.x(), clusters.y(), clusters.z(), clusters.w());
    }
    
    senddata[0] = 0xFF;
    senddata[1] = 0xFE;
    senddata[2] = 2;
    senddata[3 * clusters_amount + 3] = 0xAA;
    senddata[3 * clusters_amount + 4] = 0xDD;
    uart1.UART_SEND(senddata, 3 * clusters_amount + 5);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pos_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("aft_mapped_to_init", 1, OdomCallback);
    ros::Subscriber clusters_sub = nh.subscribe<std_msgs::Float32MultiArray>("/depth_clustering/clusters", 1, ClustersCallback);
    ros::spin();
    return 0;
}