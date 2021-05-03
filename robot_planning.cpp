#include "robot_map.hpp"
#include "robot.hpp"

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
int main(int argc, char* argv[]) {
    // load arguments
    int robot_num = 1;
    string map_file = "map3.txt";
    string start_position_file_path = "pos3.txt";

    if (argc > 1) {
        robot_num = atoi(argv[1]); 
    }

    if (argc > 2) {
        map_file = std::string(argv[2]);
    }

    if (argc > 3) {
        start_position_file_path = std::string(argv[3]);
    }

    // initialize system
    std::cout<<"Initializing Sytem"<<std::endl;
    RobotMap<LidarSensor<LIDAR_RANGE> > robot_map(map_file);

    // load positions
    std::cout<<"Loading Positions"<<std::endl;
    std::ifstream start_position_file(start_position_file_path);
    std::string line;
    std::vector<Position> starting_postions;
    while (getline(start_position_file, line)) {
        if (line != "\n") {
            std::cout<<"Loading Postion "<< line<<std::endl;
            std::istringstream token_stream(line);
            std::string token;
            std::vector<int> numbers;
            while (getline(token_stream, token, ',')) {
                int element_value = std::stoi(token);
                numbers.push_back(element_value);
            }
            starting_postions.emplace_back(numbers.at(0), numbers.at(1));
        }
    }
    start_position_file.close();
    vector<Robot> robot_list;
    for (int i = 0; i < robot_num; ++i) {
        std::cout<<"Initializing Robot "<<i<<std::endl;
        robot_list.emplace_back(std::to_string(i), starting_postions.at(i));
    }
    CentralPlanner central_planner(1,10,10, robot_num);

    // main loop
    std::cout<<"Starting Main Loop"<<std::endl;
    bool map_explored = false;
    int loop_counter = 0;
    while(!map_explored) {
        loop_counter++;
        std::cout<<"Loop Number "<<loop_counter<<std::endl;
        // detect all frontier groups
        std::cout<<"Detect all frontier groups"<<std::endl;
        vector<FrontierGroup> all_f_group = robot_map.getAllFrontierGroups();

        // assign the frontiers to the robots
        std::cout<<"Assign the frontiers to the robots"<<std::endl;
        vector<Position> robot_positions;
        for (const auto& robot : robot_list) {
            robot_positions.push_back(robot.get_position());
        }
        auto frontiers_assigned = central_planner.assignFrontierGroup(all_f_group, robot_positions, robot_map.max_frontier_group_size);
        for (int i = 0; i < robot_num; i++) {
            robot_list[i].assignFrontierGroup(all_f_group[frontiers_assigned[i]]);
        }

        // make plans for all robots
        std::cout<<"Make plans for all robots"<<std::endl;
        vector<bool> path_executed(robot_num, false);
        bool all_robots_reached = false;
        while(!all_robots_reached) {
            all_robots_reached = true;
            for (int i = 0; i < robot_num; i++) {
                path_executed[i] = robot_list[i].executePlan(robot_map, all_f_group[frontiers_assigned[i]]);
                all_robots_reached &= path_executed[i];
            }
        }
    }


    return 0;
}