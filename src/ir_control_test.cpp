#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#ifdef FOR_SIMULATION
    #include <gazebo_msgs/ContactsState.h>
#endif

using namespace std;
//const double threshold = 0.6;
// const double

enum DIRECTION
{
    FORWARD = 1,
    CLOSE,
    RIGHT,
    LEFT,
    STOP
};

class along_the_wall
{
public:
    along_the_wall(double thro_dis,bool is_along_wall);
    void do_action(DIRECTION dir);
    int check_status(const double &fd, const double &ld, const double &rd,const double &threshold,const double &max);
    void set_fx(int status);
    #ifdef FOR_SIMULATION
        void bumper_states_callback(const gazebo_msgs::ContactsState::ConstPtr& bumperptr);
        void ir_callback(const sensor_msgs::LaserScan::ConstPtr& ir);
    #else
        void bumper_states_ir_callback(const std_msgs::Int8MultiArray::ConstPtr& bumperptr);
        void ir_callback(const sensor_msgs::Range::ConstPtr& ir);
    #endif
private:
    ros::NodeHandle nh_;
    ros::Publisher  cmd_vel_pub_;  //控制机器人运动
    ros::Subscriber ir_sub_;   //接收红外传感器信息
    geometry_msgs::Twist cmd_vel;
    double last_distance;
    double current_distance;
    double delta_distance;
    double average;
    double threshold_distance;
    double r_d;
    double f_d;
    double lf_d;
    double l_d;
    double lh_d;
    int current_status;
    int last_status;
    bool first_start;
    int fx;
    bool is_along_wall;
};


along_the_wall::along_the_wall(double thro_dis,bool is_along_wall):threshold_distance(thro_dis),is_along_wall(is_along_wall)
{
    #ifdef FOR_SIMULATION
    ir_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 50, &along_the_wall::ir_callback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);
    first_start =true;
    fx = 0;
    #endif
    ros::spin();
}

int along_the_wall::check_status(const double &fd, const double &ld, const double &rd,const double &threshold,const double &max)
{
    if(fd>max && ld>max && rd>max)
        return 1;
    else if(ld<threshold && fd>max && rd>max)
        return 2;
    else if(fd<threshold && ld>max && rd>max)
        return 3;
    else if(ld>max && rd<threshold && fd>max)
        return 4;
    else if(ld<threshold && rd>max && fd<threshold)
        return 5;
    else if(ld>max && rd<threshold && fd<threshold)
        return 6;
    else if(ld<threshold && rd<threshold && fd>max)
        return 7;
    else if(ld<threshold && rd<threshold && fd<threshold)
        return 8;
    else
        return 0;
}


void along_the_wall::ir_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
     lh_d = msg->ranges[390];
     l_d = msg->ranges[360];
     lf_d = msg->ranges[330];
     f_d = msg->ranges[180];
     r_d = msg->ranges[0];

     if(first_start)
     {
        current_status = check_status(f_d,l_d,r_d,0.6,8.0);
        first_start = false;
        last_status = current_status;
        return;
     }
     current_status = check_status(f_d,l_d,r_d,0.6,8.0);
     if(current_status == 1 && last_status ==1)
         do_action(FORWARD);
     else if(current_status == 1 && last_status ==2)
         do_action(LEFT);
     else if(current_status == 1 && last_status ==3)
     {
            do_action(RIGHT);
     }
     else if(current_status == 1 && last_status ==4)
         do_action(RIGHT);
     else if(current_status == 1 && last_status ==5)
         do_action(LEFT);
     else if(current_status == 1 && last_status ==6)
         do_action(RIGHT);
     else if(current_status == 1 && last_status ==7)
     {
            do_action(RIGHT);
     }
     else if(current_status == 1 && last_status ==8)
     {
            do_action(RIGHT);
     }
     else if(current_status == 2 && last_status ==1)
         do_action(FORWARD);
     else if(current_status == 2 && last_status ==2)
         do_action(FORWARD);
     else if(current_status == 2 && last_status ==3)
         do_action(FORWARD);
     else if(current_status == 2 && last_status ==4)
         do_action(RIGHT);
     else if(current_status == 2 && last_status ==5)
         do_action(FORWARD);
     else if(current_status == 2 && last_status ==6)
         do_action(RIGHT);
     else if(current_status == 2 && last_status ==7)
     {
            do_action(RIGHT);
     }
     else if(current_status == 2 && last_status ==8)
     {
            do_action(RIGHT);
     }
     else if(current_status == 3 && last_status ==1)
     {
            do_action(LEFT);
     }
     else if(current_status == 3 && last_status ==2)
         do_action(LEFT);
     else if(current_status == 3 && last_status ==3)
     {
            do_action(LEFT);
     }
     else if(current_status == 3 && last_status ==4)
         do_action(RIGHT);
     else if(current_status == 3 && last_status ==5)
         do_action(LEFT);
     else if(current_status == 3 && last_status ==6)
         do_action(RIGHT);
     else if(current_status == 3 && last_status ==7)
     {
            do_action(RIGHT);
     }
     else if(current_status == 3 && last_status ==8)
     {
            do_action(RIGHT);
     }
     else if(current_status == 4 && last_status ==1)
         do_action(FORWARD);
     else if(current_status == 4 && last_status ==2)
         do_action(LEFT);
     else if(current_status == 4 && last_status ==3)
         do_action(FORWARD);
     else if(current_status == 4 && last_status ==4)
         do_action(FORWARD);
     else if(current_status == 4 && last_status ==5)
         do_action(LEFT);
     else if(current_status == 4 && last_status ==6)
         do_action(FORWARD);
     else if(current_status == 4 && last_status ==7)
     {
            do_action(FORWARD);
     }
     else if(current_status == 4 && last_status ==8)
     {
            do_action(FORWARD);
     }
     else if(current_status == 5 && last_status ==1)
         do_action(RIGHT);
     else if(current_status == 5 && last_status ==2)
         do_action(RIGHT);
     else if(current_status == 5 && last_status ==3)
         do_action(RIGHT);
     else if(current_status == 5 && last_status ==4)
         do_action(RIGHT);
     else if(current_status == 5 && last_status ==5)
         do_action(RIGHT);
     else if(current_status == 5 && last_status ==6)
         do_action(RIGHT);
     else if(current_status == 5 && last_status ==7)
         do_action(RIGHT);
     else if(current_status == 5 && last_status ==8)
         do_action(RIGHT);
     else if(current_status == 6 && last_status ==1)
         do_action(RIGHT);
     else if(current_status == 6 && last_status ==2)
         do_action(LEFT);
     else if(current_status == 6 && last_status ==3)
         do_action(LEFT);
     else if(current_status == 6 && last_status ==4)
         do_action(LEFT);
     else if(current_status == 6 && last_status ==5)
         do_action(LEFT);
     else if(current_status == 6 && last_status ==6)
         do_action(LEFT);
     else if(current_status == 6 && last_status ==7)
         do_action(LEFT);
     else if(current_status == 6 && last_status ==8)
         do_action(LEFT);
     else if(current_status == 7 && last_status ==1)
         do_action(FORWARD);
     else if(current_status == 7 && last_status ==2)
         do_action(FORWARD);
     else if(current_status == 7 && last_status ==3)
         do_action(FORWARD);
     else if(current_status == 7 && last_status ==4)
         do_action(FORWARD);
     else if(current_status == 7 && last_status ==5)
         do_action(FORWARD);
     else if(current_status == 7 && last_status ==6)
         do_action(FORWARD);
     else if(current_status == 7 && last_status ==7)
         do_action(FORWARD);
     else if(current_status == 7 && last_status ==8)
         do_action(FORWARD);
     else if(current_status == 8 && last_status ==1)
     {
            do_action(LEFT);
     }
     else if(current_status == 8 && last_status ==2)
         do_action(RIGHT);
     else if(current_status == 8 && last_status ==3)
     {
            do_action(LEFT);
     }
     else if(current_status == 8 && last_status ==4)
         do_action(LEFT);
     else if(current_status == 8 && last_status ==5)
         do_action(RIGHT);
     else if(current_status == 8 && last_status ==6)
         do_action(LEFT);
     else if(current_status == 8 && last_status ==7)
     {
            do_action(LEFT);
     }
     else if(current_status == 8 && last_status ==8)
     {
            do_action(LEFT);
     }
     last_status = current_status;
}


void along_the_wall::do_action(DIRECTION dir)
{
    geometry_msgs::Twist cmd_vel ;
    geometry_msgs::Twist cmd_vel_zero ;
    cmd_vel_zero.linear.x = 0.0 ;
    cmd_vel_zero.angular.z = 0.0 ;
    switch(dir)
    {
        case FORWARD:
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel);
            break;
        case CLOSE:
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = -0.1;
            cmd_vel_pub_.publish(cmd_vel);
            break;
        case LEFT:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z =  0.1;
            cmd_vel_pub_.publish(cmd_vel);
            break;
        case RIGHT:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -0.1;
            cmd_vel_pub_.publish(cmd_vel);
            break;
        case STOP:
            cmd_vel_pub_.publish(cmd_vel_zero);
            break;
    }
}


int main(int argc, char* argv[])
{
   ros::init(argc, argv, "along_the_wall");
   along_the_wall alw(0.01,true);
   return 0;
}

