#include <signal.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
 
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>


 
using namespace std;
#define TIME_STEP 32   //时钟
#define NMOTORS 2       //电机数量
#define MAX_SPEED 2.0   //电机最大速度
 
ros::NodeHandle *n;

static int controllerCount;
static vector<string> controllerList; 

static bool callbackCalled = false;

double GPSValues[4];                        // GPS数据
int gps_flag = 0;
double inertialUnitValues[4];                   // IMU数据
double line_speed = 0;
double angular_speed = 0;

ros::ServiceClient timeStepClient;          //时钟通讯客户端
webots_ros::set_int timeStepSrv;            //时钟服务数据

ros::Publisher odompub;

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}
//tf transformation
void broadcastTransform()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(GPSValues[0],GPSValues[1],0));// 设置原点
    tf::Quaternion q(inertialUnitValues[0],inertialUnitValues[1],inertialUnitValues[2],inertialUnitValues[3]);// 四元数 ->欧拉角
    q = q.inverse();// 反转四元数
    transform.setRotation(q); //设置旋转数据
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link"));
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lidar", "/vehicle/lidar"));
}

// void send_odom_data()
// {
//     nav_msgs::Odometry odom;
//     odom.header.frame_id = "odom";
//     odom.child_frame_id = "base_link";
//     odom.header.stamp = ros::Time::now();
//     odom.pose.pose.position.x = GPSValues[0];
//     odom.pose.pose.position.y = GPSValues[1];
//     odom.pose.pose.position.z = 0;

//     odom.pose.pose.orientation.x = 0;
//     odom.pose.pose.orientation.y = 0;
//     odom.pose.pose.orientation.z = inertialUnitValues[2];
//     odom.pose.pose.orientation.w = -inertialUnitValues[3];

//     odom.twist.twist.linear.x = liner_speed;
//     odom.twist.twist.angular.z = angular_speed;

//     odompub.publish(odom);

// }

void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {
  inertialUnitValues[0] = values->orientation.x;
  inertialUnitValues[1] = values->orientation.y;
  inertialUnitValues[2] = values->orientation.z;
  inertialUnitValues[3] = values->orientation.w;

  ROS_INFO("Inertial unit values (quaternions) are x=%f y=%f z=%f w=%f (time: %d:%d).", inertialUnitValues[0],
           inertialUnitValues[1], inertialUnitValues[2], inertialUnitValues[2], values->header.stamp.sec,
           values->header.stamp.nsec);
  callbackCalled = true;
  broadcastTransform(); // tf坐标转换
}

	

void GPSCallback(const geometry_msgs::PointStamped::ConstPtr &value)
{
    GPSValues[0] = value->point.x; // 对应rviz坐标轴x
    GPSValues[1] = -value->point.y; // 对应rviz坐标轴y
    if (gps_flag)
    {
        GPSValues[2] = value->point.x;
        GPSValues[3] = -value->point.y;
        gps_flag=0;
    }
    broadcastTransform();  
}

// void velCallback(const nav_msgs::Odometry::ConstPtr &value)
// {
//     liner_speed = value->twist.twist.linear.x;
//     angular_speed = value->twist.twist.angular.z;
//     send_odom_data();
// }

/**********************************************************/






/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/vehicle' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}






int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 用于显示中文字符
    string controllerName;
    // 在ROS网络中创建一个名为robot_init的节点
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 截取退出信号
    signal(SIGINT, quit);

    ros::Rate r(10000);

    // 订阅webots中所有可用的model_name
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    ros::spinOnce();
    // 服务订阅time_step和webots保持同步
    timeStepClient = n->serviceClient<webots_ros::set_int>("/vehicle/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // 如果在webots中有多个控制器的话，需要让用户选择一个控制器
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        cout << "Choose the # of the controller you want to use:\n";
        cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // 退出主题，因为已经不重要了
    nameSub.shutdown();
    
    
    /*********************************************************************************************************/
    ros::ServiceClient lidar_Client;          
    webots_ros::set_int lidar_Srv;            
    lidar_Client = n->serviceClient<webots_ros::set_int>("/vehicle/lidar/enable"); // 订阅lidar使能服务
    lidar_Srv.request.value = TIME_STEP;
    // 判断是否使能成功
    if (lidar_Client.call(lidar_Srv) && lidar_Srv.response.success) {
        ROS_INFO("lidar enabled.");
    } else {
        if (!lidar_Srv.response.success)
        ROS_ERROR("Failed to enable lidar.");
        return 1;
    }

    /*********************************************************************************************************/
    ros::ServiceClient touch_sensor_Client;          
    webots_ros::set_int touch_sensor_Srv;            
    touch_sensor_Client = n->serviceClient<webots_ros::set_int>("/vehicle/touch_sensor/enable"); // 订阅touch_sensor使能服务
    touch_sensor_Srv.request.value = TIME_STEP;
    // 判断是否使能成功
    if (touch_sensor_Client.call(touch_sensor_Srv) && touch_sensor_Srv.response.success) {
        ROS_INFO("touch sensor enabled.");
    } else {
        if (!touch_sensor_Srv.response.success)
        ROS_ERROR("Failed to enable touch sensor.");
        return 1;
    }

    /*********************************************************************************************************/
    ros::ServiceClient lidar_pointcloud_Client; 
    webots_ros::set_bool lidar_pointcloud_Srv;
    lidar_pointcloud_Client = n->serviceClient<webots_ros::set_bool>("/vehicle/lidar/enable_point_cloud");
    lidar_pointcloud_Srv.request.value = true;
    // 判断是否使能成功
    if (lidar_pointcloud_Client.call(lidar_pointcloud_Srv) && lidar_pointcloud_Srv.response.success) {
        ROS_INFO("lidar point cloud enabled.");
    } else {
        if (!lidar_pointcloud_Srv.response.success)
        ROS_ERROR("Failed to enable lidar point cloud.");
        return 1;
    }

    
    /*********************************************************************************************************/
    //subscribe Imu service
    ros::ServiceClient inertial_unit_Client;          
    webots_ros::set_int inertial_unit_Srv;            
    ros::Subscriber inertial_unit_sub;
    inertial_unit_Client = n->serviceClient<webots_ros::set_int>("/vehicle/inertial_unit/enable"); //订阅IMU使能服务
    inertial_unit_Srv.request.value = TIME_STEP;
    // 判断是否使能成功
    if (inertial_unit_Client.call(inertial_unit_Srv) && inertial_unit_Srv.response.success) {
        ROS_INFO("inertial_unit enabled.");
        // 获取话题数据
        inertial_unit_sub = n->subscribe("/vehicle/inertial_unit/quaternion", 1, inertialUnitCallback);//roll_pitch_yaw

        ROS_INFO("Topic for inertial_unit initialized.");
        while (inertial_unit_sub.getNumPublishers() == 0) {
        }
        ROS_INFO("Topic for inertial_unit connected.");
    } else {
        if (!inertial_unit_Srv.response.success)
        ROS_ERROR("Failed to enable inertial_unit!!!!!");
        return 1;
    }


    /*********************************************************************************************************/

    //subscribe gps service
    ros::ServiceClient gps_Client;          
    webots_ros::set_int gps_Srv;            
    ros::Subscriber gps_sub;
    gps_Client = n->serviceClient<webots_ros::set_int>("/vehicle/gps/enable"); // 使能GPS服务
    gps_Srv.request.value = TIME_STEP;
    // 判断gps使能服务是否成功
    if (gps_Client.call(gps_Srv) && gps_Srv.response.success) {
        ROS_INFO("gps enabled.");
        // 订阅gps话题，获取数据
        gps_sub = n->subscribe("/vehicle/gps/values", 1, GPSCallback);
        ROS_INFO("Topic for gps initialized.");
        while (gps_sub.getNumPublishers() == 0) {
        }
        ROS_INFO("Topic for gps connected.");
    } else {
        if (!gps_Srv.response.success)
        ROS_ERROR("Failed to enable gps.");
        return 1;
    }


    // ros::Subscriber sub_speed;
    // sub_speed = n->subscribe("/vel", 1, velCallback);
    // odompub = n->advertise<nav_msgs::Odometry>("/odom",10);
    
    
    // main loop
    while (ros::ok()) {
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
        ROS_ERROR("Failed to call service time_step for next step.");
        break;
        }
        ros::spinOnce();
        r.sleep();

    }
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown(); 
    return 0;
}

