#include <ros/ros.h>

#include "bluerov2_dobmpc/bluerov2_ampc.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_ampc_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Time start_time = ros::Time::now();
    ros::Duration duration(40.0); // Set the desired duration to 25 seconds

    BLUEROV2_AMPC br(nh);
    // ros::Duration(20.0).sleep();
    while(ros::ok()){
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - start_time;

        if (elapsed_time >= duration)
        {
            ROS_INFO("Reached 40 seconds. Stopping the program.");
            break;
        }

        if(br.is_start==true)
        {
            // br.applyBodyWrench();
            br.EKF();
            br.RLSFF();
            br.solve();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}