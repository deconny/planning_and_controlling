#ifndef ALONG_THE_WALL_H
#define ALONG_THE_WALL_H

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

#include <planning_and_controlling/pid.h>

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

using namespace std;

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
    along_the_wall();
    void do_action(DIRECTION dir);
    int check_status(const double &fd, const double &ld, const double &rd,const double &threshold);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param_;
    ros::Publisher  cmd_vel_pub_;
    ros::Subscriber scan_sub_;
    geometry_msgs::Twist cmd_vel;
    double threshold_distance;
    double r_d;
    double f_d;
    double rf_d;
    double l_d;
    double rh_d;
    double target;
    double pid_increment;
    double Kp;
    double Ti;
    double Td;
    double line_v;
    double angular_z;
    double angular;
    double turn_v;
    int num_f;
    int num_l;
    int num_r;
    int num_rf;
    int num_rh;
    bool turn;
    PID_incremental pid;
};

#endif
