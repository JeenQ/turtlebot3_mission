#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <cmath>

// 콜백 함수 안에서 쓰이는 변수는 전역으로 선언하여 main()안에서도 쓸 수 있도록 한다.
int angle;
double goal_x, goal_y;

// quaternion을 각도로 바꿔주는 함수
int quaternion2Angle(tf::Quaternion q);
// /amcl_pose 토픽 콜백 함수
void amclMsgCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    tf::Quaternion q(msg->pose.pose.orientation.x, 
                     msg->pose.pose.orientation.y, 
                     msg->pose.pose.orientation.z, 
                     msg->pose.pose.orientation.w);
    angle = quaternion2Angle(q);
}
// /scan 토픽 콜백 함수
void scanMsgCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    std::vector<float> ranges = msg->ranges;
    // ranges[90];
}
// 목적지 토픽 콜백 함수
void goalMsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "turtlebot3_mission");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    // /cmd_vel 토픽 publisher 선언
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    // /amcl_pose 토픽 subscriber 선언
    ros::Subscriber amclSub = nh.subscribe("/amcl_pose", 100, amclMsgCallback);
    // /scan 토픽 subscriber 선언
    ros::Subscriber scanSub = nh.subscribe("/scan", 100, scanMsgCallback);
    //  목적지 토픽에 대한 subscriber
    ros::Subscriber goalSub = nh.subscribe("/move_base_simple/goal", 100, goalMsgCallback);

    double a;
    while(ros::ok()){
        ros::spinOnce();
        // 현재 로봇이 보고 있는 방향의 각도 출력
        std::cout<<angle<<std::endl;
        loop_rate.sleep();
    }
    return 0;
}
// quaternion을 각도로 바꿔주는 함수
int quaternion2Angle(tf::Quaternion q){
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    int a = (int)(yaw/M_PI*180);
    if(a>0){
        a = 90-a;
        if(a<0){
            a=360+a;
        }
    }
    else if(a<0){
        a = 90+(-1)*a;
    }
    return a;
}