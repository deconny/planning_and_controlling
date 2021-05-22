#include "planning_and_controlling/along_the_wall.h"

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "scan_control_node");
   along_the_wall alw;
   ros::spin();
   return 0;
}
