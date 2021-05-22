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

#ifdef FOR_SIMULATION
    #include <gazebo_msgs/ContactsState.h>
#endif




struct action
{
    bool is_direction_forward ;
    double radius ;
    double distance ;
};

struct bumper
{
    bool is_bumper_contacted ;
    int point_bumper_contacted ;  //0:no contacted,1:front_right,2:front_left,3:right,4:left;
};


class planning_and_controlling
{
public:
    planning_and_controlling();
    ~planning_and_controlling();


private:

    ros::NodeHandle nh;
    ros::Publisher  cmd_vel_pub_;
    ros::Publisher  bumper_pose_pub_;
    ros::Subscriber bumper_contact_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ir_sub_;


    #ifdef FOR_SIMULATION
        void bumper_states_cb(const gazebo_msgs::ContactsState::ConstPtr &msg_ptr);
        void ir_cb(const sensor_msgs::LaserScan::ConstPtr &msg_ptr);
    #else
        void bumper_states_cb(const std_msgs::Int8MultiArray::ConstPtr &msg_ptr);
        void ir_cb(const sensor_msgs::Range::ConstPtr &msg_ptr);
    #endif


    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg_ptr);

    bool do_action(const action &action_data);

    bool is_start_odom ;
    bool is_enter_wall_mode ;

    tf::TransformListener listener;

    nav_msgs::Odometry odom_rec;
    nav_msgs::Odometry odom_last ;

    double x_gap ;
    double y_gap ;
    double distance ;
    double range_to_wall;


    bumper bumper_state;


};


planning_and_controlling::planning_and_controlling()
{
    ros::Rate loop_rate(10);
    bumper_state.is_bumper_contacted = false;

    is_start_odom = false;
    is_enter_wall_mode = false;


    x_gap = 0.0 ;
    y_gap = 0.0 ;
    distance = 0.0 ;
    range_to_wall = 0.0;

    action data_move_forward,data_move_backward,data_move_forward_right,data_move_forward_left,data_move_backward_right,data_move_backward_left;
    data_move_forward.distance = data_move_backward.distance = data_move_forward_right.distance = data_move_forward_left.distance
    = data_move_backward_right.distance = data_move_backward_left.distance = 0.20;

    data_move_forward.is_direction_forward = data_move_forward_right.is_direction_forward = data_move_forward_left.is_direction_forward = true;
    data_move_backward.is_direction_forward = data_move_backward_right.is_direction_forward  = data_move_backward_left.is_direction_forward  = false;

    data_move_forward.radius = DBL_MAX;
    data_move_backward.radius = -DBL_MAX;

    data_move_forward_right.radius = data_move_forward_left.radius = data_move_backward_right.radius = data_move_backward_left.radius = 0.3;

    #ifdef FOR_SIMULATION
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);

    bumper_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bumper_point", 100, true);

    bumper_contact_sub_ = nh.subscribe<gazebo_msgs::ContactsState>("/robot/bumper_states", 50, &planning_and_controlling::bumper_states_cb, this);

    ir_sub_  = nh.subscribe<sensor_msgs::LaserScan>("ir_right", 50, &planning_and_controlling::ir_cb, this);
    #endif

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 50, &planning_and_controlling::odom_cb, this);



    odom_last = odom_rec ;

    while(ros::ok())
    {

        bumper bumper_state_now = bumper_state ;

        if( bumper_state_now.is_bumper_contacted )
        {
            if( is_start_odom )
            {
                odom_last = odom_rec ;
                is_start_odom = false ;
            }

            if( bumper_state_now.point_bumper_contacted = 1)
            {

                do_action( data_move_backward_right );

            }
            else if( bumper_state_now.point_bumper_contacted = 2)
            {

                do_action( data_move_backward_right );

            }

        }

        if( is_enter_wall_mode )
        {
            ROS_ERROR(" is_enter_wall_mode ");
            geometry_msgs::Twist cmd_vel ;

            cmd_vel.linear.x = 0.2 ;

            cmd_vel.angular.z = 2 * ( ( 0.10 - range_to_wall ) / 0.3 ) * cmd_vel.linear.x ;

            cmd_vel_pub_.publish( cmd_vel );

        }
        else
        {
            geometry_msgs::Twist cmd_vel ;
            cmd_vel.linear.x = 0.2 ;

            cmd_vel_pub_.publish( cmd_vel );
        }

        ros::spinOnce();
        loop_rate.sleep();

    }


}

planning_and_controlling::~planning_and_controlling()
{

}


#ifdef FOR_SIMULATION
void planning_and_controlling::bumper_states_cb(const gazebo_msgs::ContactsState::ConstPtr &msg_ptr)
{


    for( int i = 0; i < msg_ptr->states.size(); i++)
    {
        for( int j = 0;j < msg_ptr->states[i].contact_positions.size(); j++)
        {

            geometry_msgs::PoseStamped pose_odom,pose_bumper;

            pose_odom.header.frame_id = "/odom";
            //pose_odom.header.stamp  = msg_ptr->header.stamp;

            pose_odom.pose.position.x = msg_ptr->states[i].contact_positions[j].x;
            pose_odom.pose.position.y = msg_ptr->states[i].contact_positions[j].y;
            pose_odom.pose.position.z = msg_ptr->states[i].contact_positions[j].z;

            pose_odom.pose.orientation.x = 0.0;
            pose_odom.pose.orientation.y = 0.0;
            pose_odom.pose.orientation.z = 0.0;
            pose_odom.pose.orientation.w = 1.0;

            try{
                listener.transformPose("bumper_link", pose_odom, pose_bumper);
            }
            catch( tf::TransformException ex)
            {
                ROS_WARN("transfrom exception : %s",ex.what());
                return;
            }

            is_start_odom = true;
            bumper_pose_pub_.publish(pose_bumper);

            if( pose_bumper.pose.position.y >= 0)
            {
                bumper_state.point_bumper_contacted = 2;
                //ROS_ERROR( "bumper_state.point_bumper_contacted = 2;");
            }
            else
            {
                bumper_state.point_bumper_contacted = 1;
                //ROS_ERROR( "bumper_state.point_bumper_contacted = 1;");
            }

            //ROS_ERROR(" bumper_states_cb ");
            bumper_state.is_bumper_contacted = true;


        }
    }

}


void planning_and_controlling::ir_cb(const sensor_msgs::LaserScan::ConstPtr &msg_ptr)
{
    int num = msg_ptr->ranges.size();
    double ranges = 0.0 ;
    for (int i = 0 ; i < num; i++)
    {
        ranges += msg_ptr->ranges[i];
    }

    range_to_wall = ranges / num ;

    ROS_ERROR(" range_to_wall is :%f\n",range_to_wall);

    if( range_to_wall <= 0.50)
    {
        is_enter_wall_mode = true;
    }
    else
    {
        is_enter_wall_mode = false;
    }
}


#else
void planning_and_controlling::bumper_states_cb(const std_msgs::Int8MultiArray::ConstPtr &msg_ptr)
{




}

void planning_and_controlling::ir_cb(const sensor_msgs::Range::ConstPtr &msg_ptr)
{



}

#endif






void planning_and_controlling::odom_cb(const nav_msgs::Odometry::ConstPtr &msg_ptr)
{
    odom_rec = *msg_ptr;

}



bool planning_and_controlling::do_action(const action &action_data)
{
    geometry_msgs::Twist cmd_vel ;

    cmd_vel.linear.x = 0.1 ;

    cmd_vel.angular.z = cmd_vel.linear.x / action_data.radius ;

    geometry_msgs::Twist cmd_vel_zero ;

    cmd_vel_zero.linear.x = 0.0 ;
    cmd_vel_zero.angular.z = 0.0 ;


    if( action_data.is_direction_forward )
    {

        if( distance <= action_data.distance )
        {
            cmd_vel_pub_.publish( cmd_vel );

            x_gap = odom_last.pose.pose.position.x - odom_rec.pose.pose.position.x ;
            y_gap = odom_last.pose.pose.position.y - odom_rec.pose.pose.position.y ;
            distance += sqrt( x_gap * x_gap + y_gap * y_gap);
            odom_last = odom_rec ;
            ros::Duration(0.1).sleep();

        }
        else
        {
            cmd_vel_pub_.publish( cmd_vel_zero );
            bumper_state.is_bumper_contacted = false;
            bumper_state.point_bumper_contacted = 0;
            distance = 0.0;
        }

    }
    else
    {
        if( distance <= action_data.distance )
        {
            cmd_vel.linear.x = - cmd_vel.linear.x ;
            cmd_vel_pub_.publish( cmd_vel );


            x_gap = odom_last.pose.pose.position.x - odom_rec.pose.pose.position.x ;
            y_gap = odom_last.pose.pose.position.y - odom_rec.pose.pose.position.y ;
            distance += sqrt( x_gap * x_gap + y_gap * y_gap);
            odom_last = odom_rec ;
            ros::Duration(0.1).sleep();
        }
        else
        {
            cmd_vel_pub_.publish( cmd_vel_zero );
            bumper_state.is_bumper_contacted = false;
            bumper_state.point_bumper_contacted = 0;
            distance = 0.0;
        }

    }
}


int main(int argc, char* argv[])
{
   ros::init(argc, argv, "planning_and_controlling");
   planning_and_controlling planning_and_controlling;
   ros::spin();
}
