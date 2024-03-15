#include "bluerov2_path/bluerov2_path.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluerov2_path_node");
    ros::NodeHandle nh("~");

    BLUEROV2_PATH br(nh);

    ros::spin();

    return 0;
}