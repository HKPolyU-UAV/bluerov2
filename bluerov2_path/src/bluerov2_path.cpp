#include "bluerov2_path/bluerov2_path.h"

BLUEROV2_PATH::BLUEROV2_PATH(ros::NodeHandle& nh)
{
    nh.getParam("/bluerov2_path_node/ref_traj", REF_TRAJ);
    std::cout<<REF_TRAJ<<std::endl;
    const char * c = REF_TRAJ.c_str();

    number_of_steps = readDataFromFile(c, trajectory);

    if (number_of_steps == 0)
		ROS_WARN("Cannot load CasADi optimal trajectory!");
	else
		ROS_INFO_STREAM(
            "Number of steps of selected trajectory: " 
            << number_of_steps 
            << std::endl
        );

    ref_traj_pub = nh.advertise<airo_message::BlueRefPreview>
                ("/ref_traj", 1);
    ref_pt_pub = nh.advertise<geometry_msgs::PoseStamped>
                ("/bluerov2/cmd_pose",1);

    mainspin_timer = nh.createTimer(
        ros::Duration(1.0/20.0),
        &BLUEROV2_PATH::mainspin_cb,
        this
    );

    std::cout<<"gan"<<std::endl;

    is_start = true;
}

void BLUEROV2_PATH::mainspin_cb(const ros::TimerEvent& e)
{
    if(line_number > number_of_steps)
        return;

    read_N_pub(line_number);
    line_number++;

    ref_point.header.frame_id = "world";
    ref_point.header.stamp = ros::Time::now();
    
    ref_point.pose.position.x = -5;
    ref_point.pose.position.y = -5;
    ref_point.pose.position.z = -20;

    ref_pt_pub.publish(ref_point);
}

int BLUEROV2_PATH::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
{
	std::ifstream file(fileName);
	std::string line;
	int number_of_lines = 0;

	if (file.is_open())
	{
        std::cout<<"file is open"<<std::endl;
		while(getline(file, line)){
			number_of_lines++;
			std::istringstream linestream( line );
			std::vector<double> linedata;
			double number;

			while( linestream >> number ){
				linedata.push_back( number );
			}
			data.push_back( linedata );
		}

		file.close();
	}
	else
	{
        std::cout<<"file not open"<<std::endl;
		return 0;
	}

	return number_of_lines;
}

void BLUEROV2_PATH::read_N_pub(int line_to_read)
{
    ref_traj.header.stamp = ros::Time();
    ref_traj.preview.clear();

    if (BLUEROV2_N + line_to_read+1 <= number_of_steps)  
    // All ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  
            ref_traj.preview.emplace_back(extract_ref_pt(i + line_to_read));
            // Fill all horizon with file data
    }
    else if(line_to_read < number_of_steps)    
    // Part of ref points within the file
    {
        for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)
            ref_traj.preview.emplace_back(extract_ref_pt(i + line_to_read));    
            // Fill part of horizon with file data
        
        for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2_N; i++)  
            ref_traj.preview.emplace_back(extract_ref_pt(number_of_steps-1));    
            // Fill the rest horizon with the last point
    }
    else    
    // none of ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  
            ref_traj.preview.emplace_back(extract_ref_pt(number_of_steps-1)); 
            // Fill all horizon with the last point
    }

    std::cout<<"TRAJ_SIZE: "<<ref_traj.preview.size()<<std::endl;
    std::cout<<"WHICH LINE: "<<line_to_read<<std::endl<<std::endl;;

    
}

airo_message::BlueRef BLUEROV2_PATH::extract_ref_pt(const int no_of_line)
{
    airo_message::BlueRef ref_pt;

    ref_pt.ref_pos.x = trajectory[no_of_line][0];
    ref_pt.ref_pos.y = trajectory[no_of_line][1];
    ref_pt.ref_pos.z = trajectory[no_of_line][2];

    ref_pt.ref_ang.x = trajectory[no_of_line][3];
    ref_pt.ref_ang.y = trajectory[no_of_line][4];
    ref_pt.ref_ang.z = trajectory[no_of_line][5];

    ref_pt.ref_twist.linear.x = trajectory[no_of_line][6];
    ref_pt.ref_twist.linear.y = trajectory[no_of_line][7];
    ref_pt.ref_twist.linear.z = trajectory[no_of_line][8];

    ref_pt.ref_twist.angular.x = trajectory[no_of_line][9];
    ref_pt.ref_twist.angular.y = trajectory[no_of_line][10];
    ref_pt.ref_twist.angular.z = trajectory[no_of_line][11];

    return ref_pt;
}
