#include "bluerov2_path/bluerov2_path.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_dob_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Time start_time = ros::Time::now();
    ros::Duration duration(50.0); // Set the desired duration to 25 seconds

    BLUEROV2_PATH br(nh);
    while(ros::ok()){
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - start_time;

        
        if(br.is_start==true)
        {
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}