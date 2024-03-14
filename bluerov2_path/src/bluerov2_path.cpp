#include "bluerov2_path/bluerov2_path.h"


BLUEROV2_PATH::BLUEROV2_PATH(ros::NodeHandle&)
{

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

void BLUEROV2_PATH::ref_cb(int line_to_read)
{
    // if (BLUEROV2_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    // {
    //     for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with file data
    //     {
    //         for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
    //         {
    //             acados_in.yref[i][j] = trajectory[i+line_to_read][j];
    //         }
    //     }
    // }
    // else if(line_to_read < number_of_steps)    // Part of ref points within the file
    // {
    //     for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
    //     {
            
    //         for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
    //         {
    //             acados_in.yref[i][j] = trajectory[i+line_to_read][j];
    //         }
            
    //     }

    //     for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2_N; i++)  // Fill the rest horizon with the last point
    //     {
            
    //         for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
    //         {
    //             acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
    //         }
            
    //     }
    // }
    // else    // none of ref points within the file
    // {
    //     for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with the last point
    //     {
            
    //         for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
    //         {
    //             acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
    //         }
            
    //     }
    // }
    
}