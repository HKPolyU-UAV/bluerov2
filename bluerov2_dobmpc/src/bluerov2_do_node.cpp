#include <ros/ros.h>

#include "bluerov2_dobmpc/bluerov2_do.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_do_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    // Run EKF algorithm
    BLUEROV2_DO brdo(nh);

    while(ros::ok()){
        if(brdo.is_start==true){
            brdo.EKF(); // run EKF algorithm
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}