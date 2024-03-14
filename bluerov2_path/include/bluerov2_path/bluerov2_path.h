#ifndef BLUEROV2_PATH_H
#define BLUEROV2_PATH_H

#include "ros_utilities/ros_utilities.h"

class BLUEROV2_PATH
{
    private:
        
        int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);     // read trajectory
        void ref_cb(int line_to_read); 
        

    public:
        BLUEROV2_PATH(ros::NodeHandle&);
        bool is_start;

};

#endif