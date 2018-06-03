#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include <cmath>

geometry_msgs::Twist cmd;
int angle;

int quaternion2Angle(tf::Quaternion q);

void amclMsgCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    tf::Quaternion q(msg->pose.pose.orientation.x, 
                     msg->pose.pose.orientation.y, 
                     msg->pose.pose.orientation.z, 
                     msg->pose.pose.orientation.w);
    angle = quaternion2Angle(q);
}
void scanMsgCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    std::vector<float> ranges = msg->ranges;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "turtlebot3_mission");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber amclSub = nh.subscribe("/amcl_pose", 100, amclMsgCallback);
    ros::Subscriber scanSub = nh.subscribe("/scan", 100, scanMsgCallback);
    while(ros::ok()){
        ros::spinOnce();
        ROS_INFO("Angle : %d", angle);
        loop_rate.sleep();
    }
    return 0;
}

int quaternion2Angle(tf::Quaternion q){
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    int a = (int)(yaw/M_PI*180);
    if(a>0){
        a = 360-a;
    }
    else if(a<0){
        a = a * (-1);
    }
    return a;
}