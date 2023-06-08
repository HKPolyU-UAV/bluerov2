#include <ros/ros.h>

#include "bluerov2_dobmpc/bluerov2_dob.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_dob_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    BLUEROV2_DOB br(nh);
    //std::cout<< "br constructed"<<std::endl;
    while(ros::ok()){
        //std::cout<< "ros::ok, go to run"<<std::endl;
        if(br.is_start==true)
        {
            br.solve();
        }
        //std::cout<< "run once"<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}