#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <sstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "samplepublish" );
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);



    ros::Rate loop_rate(10);
    int count =0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count++;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        loop_rate.sleep() ;
    }
}
