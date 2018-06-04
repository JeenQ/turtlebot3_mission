# 프로젝트

## 1. Mission

로봇을 현재 위치에서 주어진 목적지까지 자동으로 움직이도록 해보자.
로봇 앞에 장애물이 있을 수도 있다. 라이다 센서를 이용해서 장애물을 피해가도록 한다.

## 2. 프로젝트를 위한 준비

### 2.1. package 설치

    cd ~/catkin_ws/src
    git clone https://github.com/JeenQ/turtlebot3_mission.git
    cd ~/catkin_ws
    catkin_make     
      
### 2.2. 로봇 연결

리모트 PC에서 roscore를 구동한 후 터틀봇3에 원격 접속하여 로봇을 구동시킨다.(turtlebot3_robot.launch를 실행한다.)

### 2.3. Turtlebot3_mission.launch 실행

    roslaunch turtlebot3_mission turtlebot3_mission.launch map_file:=`rospack find turtlebot3_mission`/map/map.yaml
      
### 2.4. rostopic 살펴보기

    rostopic list
    
현재 ROS 시스템 상에서 구동되고 있는 토픽들을 볼 수 있다. 이 중 “/scan”이라는 토픽을 살펴보자. 이 토픽은 라이다 센서의 값을 가지고 있다.

    rostopic echo /scan
    
“/scan” 토픽의 메시지 타입을 살펴보자

    rostopic type /scan
    
“/scan”은 “sensor_msgs/LaserScan”이라는 메시지 타입을 가지고 있다.

http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html

이 링크에서 이 메시지 타입에 대한 자세한 정보를 얻을 수 있다.
위와 같이 필요한 토픽을 사용하려면 타입을 알아내어 그 타입에 해당하는 패키지를 추가해줘야 한다.

### 2.5. mission_node.cpp

본 프로젝트를 수행하기 위해 여러분들이 코드를 넣어야 하는 스켈레톤 소스 파일이다.

    #include <ros/ros.h>
    #include <sensor_msgs/LaserScan.h>
    #include <geometry_msgs/Twist.h>
    #include <geometry_msgs/PoseWithCovarianceStamped.h>
    #include <tf/transform_datatypes.h>
    #include <std_msgs/Int32.h>
    #include <cmath>
    // 콜백 함수 안에서 쓰이는 변수는 전역으로 선언하여 main()안에서도 쓸 수 있도록 한다.
    int angle;
    // quaternion을 각도로 바꿔주는 함수
    int quaternion2Angle(tf::Quaternion q);
    // /amcl_pose 토픽 콜백 함수
    // 현재의 위치를 인식하여 좌표와 바라보는 방향을 알려준다.
    // 이때 방향을 알려주기 위한 방법으로 quaternion이라는 것을 사용하는데
    // 생소할 수 있으므로 각도로 바꿔주었다.
    void amclMsgCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        tf::Quaternion q(msg->pose.pose.orientation.x, 
                     msg->pose.pose.orientation.y, 
                     msg->pose.pose.orientation.z, 
                     msg->pose.pose.orientation.w)
        angle = quaternion2Angle(q);
    }
    // /scan 토픽 콜백 함수
    void scanMsgCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        std::vector<float> ranges = msg->ranges;
        // ranges[index] -> 다음과 같은 방법으로 접근할 수 있다.
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
        while(ros::ok()){
            ros::spinOnce();
            // 현재 로봇이 보고 있는 방향의 각도 출력
            ROS_INFO("Angle : %d", angle);
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
            a = 360-a;
        }
        else if(a<0){
            a = a * (-1);
        }
        return a;
    }
    
다음과 같이 여러 개의 subscriber와 publisher를 가진 하나의 노드를 구성할 수 있다.
각 subscriber는 개별 콜백 함수를 갖는 것에 유의하자.
새로운 터미널에서 미션 노드를 실행한다.

    rosrun turtlebot3_mission turtlebot3_mission_node
