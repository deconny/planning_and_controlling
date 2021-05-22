#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class scan_receive
{
public:
    scan_receive(ros::NodeHandle nh);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    sensor_msgs::LaserScan newscan_;
    int num;
};

scan_receive::scan_receive(ros::NodeHandle nh):nh_(nh),nh_param_("~")
{
    sub_ = nh_.subscribe("scan", 50, &scan_receive::scan_callback,this);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("filter_scan", 50);
}


void scan_receive::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    cout<<"msg->ranges.size = "<<msg->ranges.size()<<endl;
    newscan_ = *msg;
    if(nh_param_.getParam("num",num))
        nh_param_.setParam("num",num);
    else
        nh_param_.param("num",num,0);
    for(unsigned int i = 0; i<newscan_.ranges.size(); i++)   //前：180 731 左：270 1096 右： 90 365
    {                                                                                //左前：250 1015 左后：290 1177
        if(i==num)
        newscan_.ranges[i] =msg->ranges[i];
        else
        newscan_.ranges[i] = 0;   
    }    
    pub_.publish(newscan_);
    cout<<"msg->ranges"<<"["<<num<<"]"<<" = "<<msg->ranges[num]<<endl;
}

int main(int argc, char * argv[])
{
   ros::init(argc, argv,"scan_receive");
   ros::NodeHandle nh;
   scan_receive sc(nh);
   ros::spin();
   return 0;
}
