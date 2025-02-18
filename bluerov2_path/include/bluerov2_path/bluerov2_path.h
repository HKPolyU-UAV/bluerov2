#ifndef BLUEROV2_PATH_H
#define BLUEROV2_PATH_H

#include "ros_utilities/ros_utilities.h"
#include "airo_message/BlueRefPreview.h"
#include "uuv_control_msgs/TrajectoryPoint.h"

#define BLUEROV2_NY     16
#define BLUEROV2_N      80

class BLUEROV2_PATH
{
    private:
        int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);     // read trajectory
        void read_N_pub(int line_to_read); 

        std::string REF_TRAJ;
        std::vector<std::vector<double>> trajectory;
        int line_number = 0;
        int number_of_steps = 0;

        airo_message::BlueRefPreview ref_traj;

        ros::Publisher ref_traj_pub, path_start_pub;
        ros::Timer mainspin_timer;

        void mainspin_cb(const ros::TimerEvent& e);
        airo_message::BlueRef extract_ref_pt(const int no_of_line);

    public:
        BLUEROV2_PATH(ros::NodeHandle&);
        void traj_go(){return;};
        bool is_start;

};

#endif