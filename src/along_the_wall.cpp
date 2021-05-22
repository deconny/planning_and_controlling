#include "planning_and_controlling/along_the_wall.h"
#include "planning_and_controlling/pid.h"

void along_the_wall::do_action(DIRECTION dir)
{
    geometry_msgs::Twist cmd_vel ;
    geometry_msgs::Twist cmd_vel_zero ;
    cmd_vel_zero.linear.x = 0.0 ;
    cmd_vel_zero.angular.z = 0.0 ;
    switch(dir)
    {
        case FORWARD:
            cmd_vel.linear.x = line_v;
            cmd_vel.angular.z = 0.0 ;  
            cmd_vel_pub_.publish(cmd_vel);
            break;
        case CLOSE:
            cmd_vel.linear.x = line_v;
            cmd_vel.angular.z = -angular_z;
            cmd_vel_pub_.publish(cmd_vel);         
            break;
        case LEFT:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = angular_z;  
            cmd_vel_pub_.publish(cmd_vel);
            break;            
        case RIGHT:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -angular_z;        
            cmd_vel_pub_.publish(cmd_vel);
            break;
        case STOP:
            cmd_vel_pub_.publish(cmd_vel_zero);
            break;
    }
}

along_the_wall::along_the_wall():nh_param_("~")
{
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 5, &along_the_wall::scan_callback, this);    
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5, true);
    turn = false;     
    pid_increment=0.0;
    //pid = PID_incremental(0.35,0.65,0.005);
    pid = PID_incremental(Kp,Ti,Td);
    ros::spin();
}

int along_the_wall::check_status(const double &fd, const double &ld, const double &rd,const double &threshold)
{
    if(fd>threshold && ld>threshold && rd>threshold)
        return 1;
    else if(ld<threshold && fd>threshold && rd>threshold)
        return 2;
    else if(fd<threshold && ld>threshold && rd>threshold)
        return 3;
    else if(ld>threshold && rd<threshold && fd>threshold)
        return 4;
    else if(ld<threshold && rd>threshold && fd<threshold)
        return 5;
    else if(ld>threshold && rd<threshold && fd<threshold)
        return 6;
    else if(ld<threshold && rd<threshold && fd>threshold)
        return 7;
    else if(ld<threshold && rd<threshold && fd<threshold)
        return 8;    
    else
        return 0;
}


void along_the_wall::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{ 
    if(nh_param_.getParam("target_distance",target))
        nh_param_.setParam("target_distance",target);
    else
        nh_param_.param("target_distance",target,0.25);
    if(nh_param_.getParam("threshold_distance",threshold_distance))
        nh_param_.setParam("threshold_distance",threshold_distance); 
    else
        nh_param_.param("threshold_distance",threshold_distance,0.4);      
    if(nh_param_.getParam("Kp",Kp))
        nh_param_.setParam("Kp",Kp);
    else
        nh_param_.param("Kp",Kp,0.35);
    if(nh_param_.getParam("Ti",Ti))
        nh_param_.setParam("Ti",Ti);
    else
        nh_param_.param("Ti",Ti,0.65);
    if(nh_param_.getParam("Td",Td))
        nh_param_.setParam("Td",Td); 
    else
        nh_param_.param("Td",Td,0.005);
    if(nh_param_.getParam("line_v",line_v))
        nh_param_.setParam("line_v",line_v); 
    else
        nh_param_.param("line_v",line_v,0.1); 
    if(nh_param_.getParam("angular_z",angular_z))
        nh_param_.setParam("angular_z",angular_z); 
    else
        nh_param_.param("angular_z",angular_z,0.2); 
    if(nh_param_.getParam("angular",angular))
        nh_param_.setParam("angular",angular); 
    else
        nh_param_.param("angular",angular,0.001);   
    if(nh_param_.getParam("num_f",num_f))
        nh_param_.setParam("num_f",num_f); 
    else
        nh_param_.param("num_f",num_f,245);     
    if(nh_param_.getParam("num_l",num_l))
        nh_param_.setParam("num_l",num_l); 
    else
        nh_param_.param("num_l",num_l,367); 
    if(nh_param_.getParam("num_r",num_r))
        nh_param_.setParam("num_r",num_r); 
    else
        nh_param_.param("num_r",num_r,124); 
    if(nh_param_.getParam("num_rf",num_rf))
        nh_param_.setParam("num_rf",num_rf); 
    else
        nh_param_.param("num_rf",num_rf,143); 
    if(nh_param_.getParam("num_rh",num_rh))
        nh_param_.setParam("num_rh",num_rh); 
    else
        nh_param_.param("num_rh",num_rh,102);
    if(nh_param_.getParam("turn_v",turn_v))
        nh_param_.setParam("turn_v",turn_v); 
    else
        nh_param_.param("turn_v",turn_v,0.03);         
     cout<<"msg->ranges = "<<msg->ranges.size()<<endl;
     rh_d = msg->ranges[num_rh];    //实际场景490束 
     l_d = msg->ranges[num_l];
     rf_d = msg->ranges[num_rf];
     f_d = msg->ranges[num_f];
     r_d = msg->ranges[num_r];
     ROS_INFO("\033[1;32m l_d = %f \033[0m",l_d);
     ROS_INFO("\033[1;32m f_d = %f \033[0m",f_d);
     ROS_INFO("\033[1;32m r_d = %f \033[0m",r_d);
     ROS_INFO("\033[1;32m rh_d-rf_d = %f \033[0m",abs(rh_d-rf_d));
//      ROS_INFO(RED"f_d = %f ",f_d);
     int status = check_status(f_d,l_d,r_d,threshold_distance); //0.32
     if(turn ==false)
     {
        switch(status)
        {
         case 1:
            ROS_INFO("\033[1;33m  in case 1 \033[0m");
            cmd_vel.linear.x = line_v;
            cmd_vel.angular.z = 0;        
         break;
         case 2:
            ROS_INFO("\033[1;33m  in case 2 \033[0m");
            cmd_vel.linear.x = line_v;
            pid_increment=pid.pid_control(target,l_d);
            l_d+=pid_increment;    
            cout<<"actual= "<<l_d<<endl;
            cmd_vel.angular.z = l_d - target;
//             cout<<"cmd_vel.angular.z "<<cmd_vel.angular.z<<endl;         
         break;   
         case 3:
             ROS_INFO("\033[1;33m  in case 3 \033[0m");
             pid_increment = 0;
             turn = true;
         break;       
         case 4:
             ROS_INFO("\033[1;33m  in case 4 \033[0m");
             cmd_vel.linear.x = line_v;
             pid_increment=pid.pid_control(target,r_d);
             r_d+=pid_increment;
             cout<<"actual= "<<r_d<<endl;
             cmd_vel.angular.z = target - r_d;
             break;
         case 5:
             pid_increment = 0;
             cmd_vel.linear.x = turn_v;
             ROS_INFO("\033[1;33m  in case 5 \033[0m");
             turn = true;    
             break;   
         case 6:
             pid_increment = 0;
             cmd_vel.linear.x = turn_v;
             cmd_vel.angular.z = angular_z;        
             ROS_INFO("\033[1;33m  in case 5 \033[0m");   
             turn = true;  
             break;                
        }
        
     }  
     else
     {
         if(abs(rf_d-rh_d)<angular)
         {
             turn = false;
             ROS_INFO("\033[1;31m abs = %f \033[0m",abs(rf_d-rh_d));
         }
         else
         {
             ROS_INFO("\033[1;34m LEFT \033[0m");
             ROS_INFO("\033[1;34m abs = %f \033[0m",abs(rf_d-rh_d));
             cmd_vel.angular.z = angular_z;        
         }
     }
    cmd_vel_pub_.publish(cmd_vel);
}
