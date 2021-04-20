#include "robot_map.hpp"

#include <string>
#include <iostream>
#include <fstream>

int main() {
    std::string input_map_path("map.txt");
    RobotMap<LidarSensor<5> > robot_map(input_map_path);
    std::ofstream output_file;
    output_file.open("map_vis.txt");
    std::cout<<"Writing to File "<<"map_vis.txt"<<std::endl;
    output_file << robot_map.convertToString(); 
    std::cout<<"File Writen"<<std::endl;
    output_file.close();
    std::cout<<"Done"<<std::endl;
    return 0;
}