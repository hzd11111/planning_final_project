#include "robot_map.hpp"

#include <ctime>
#include <string>
#include <iostream>
#include <fstream>

int main() {
    std::string input_map_path("map3.txt");
    std::clock_t start = std::clock();
    RobotMap<LidarSensor<100> > robot_map(input_map_path);
    std::cout<<"Map Loading Time: "<<(std::clock() - start) / (double) CLOCKS_PER_SEC<<std::endl;
    // manipulations
    start = std::clock();
    //robot_map.updateExploration(Position(10,10));
    //robot_map.updateExploration(Position(100,100));
    //robot_map.updateExploration(Position(1000,1000));
    //robot_map.updateExploration(Position(500,700));
    robot_map.updateExploration(Position(250,300));
    robot_map.updateExploration(Position(300,250));
    std::cout<<"Total Exploration Time: "<<(std::clock() - start) / (double) CLOCKS_PER_SEC<<std::endl;

    // frontiers
    start = std::clock();
    auto frontier_groups = robot_map.getAllFrontierGroups();
    std::cout<<"Number of Frontier Groups: "<<frontier_groups.size()<<std::endl;
    for (const auto & frontier_group : frontier_groups) {
        std::cout<<"Number of Frontiers: "<<frontier_group.frontiers.size()<<std::endl;
        std::cout<<"Group Size: "<<frontier_group.group_size<<std::endl;
    }

    std::cout<<"Frontier Grouping Time: "<<(std::clock() - start) / (double) CLOCKS_PER_SEC<<std::endl;
    std::ofstream output_file;
    output_file.open("map_vis.txt");
    std::cout<<"Writing to File "<<"map_vis.txt"<<std::endl;
    output_file << robot_map.convertToString(); 
    std::cout<<"File Writen"<<std::endl;
    output_file.close();
    std::cout<<"Done"<<std::endl;
    return 0;
}