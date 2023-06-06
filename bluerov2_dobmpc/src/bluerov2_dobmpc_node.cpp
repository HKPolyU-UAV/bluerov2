#include <ros/ros.h>

#include "bluerov2_dobmpc/bluerov2_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_dobmpc_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    BLUEROV2_INTERFACE br(nh);
    std::cout<< "br constructed"<<std::endl;
    while(ros::ok()){
        std::cout<< "ros::ok, go to run"<<std::endl;
        br.run();
        std::cout<< "run once"<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}