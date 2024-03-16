#include "bluerov2_dobmpc/bluerov2_ctrl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_ctrl_node");
    ROS_INFO("WELCOME TO CTRLLER 4 BLUEROV2 DEVELOPED BY AIRO-LAB!");
    
    ros::NodeHandle nh;
    BLUEROV2_CTRL br(nh);

    ros::spin();

    return 0;
}