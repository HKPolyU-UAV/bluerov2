#include <ros/ros.h>

#include "bluerov2_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_dobmpc_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    BLUEROV2_INTERFACE br(nh);

    while(ros::ok()){
        br.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}