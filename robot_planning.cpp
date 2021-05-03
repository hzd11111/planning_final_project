#include "robot_map.hpp"
#include "robot.hpp"

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

#define FLOW_DEBUG false

void writeRobotPoses(string filename, vector<Robot> rob) {
    std::ofstream output_file_intermediate;
    output_file_intermediate.open(filename);
    for (const Robot& r:rob) {
        output_file_intermediate << r.get_position().toString() << "\n";
    }
    output_file_intermediate.close();
}

int main(int argc, char* argv[]) {
    // load arguments
    int robot_num = 1;
    string map_file = "map3.txt";
    string start_position_file_path = "pos3.txt";
    string run_name = "default_run";

    if (argc > 1) {
        robot_num = atoi(argv[1]); 
    }

    if (argc > 2) {
        map_file = std::string(argv[2]);
    }

    if (argc > 3) {
        start_position_file_path = std::string(argv[3]);
    }

    if (argc > 4) {
        run_name = std::string(argv[4]);
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
        robot_map.updateExploration(starting_postions.at(i));
    }
    CentralPlanner central_planner(1,7,20, robot_num);

    // initial map vis
    std::ofstream output_file;
    output_file.open("map_vis.txt");
    if (FLOW_DEBUG) std::cout<<"Writing to File "<<"map_vis.txt"<<std::endl;
    output_file << robot_map.convertToString(); 
    if (FLOW_DEBUG) std::cout<<"File Writen"<<std::endl;
    output_file.close();


    // main loop
    if (FLOW_DEBUG) std::cout<<"Starting Main Loop"<<std::endl;
    int loop_counter = 0;
    while(true) {
        loop_counter++;
        std::cout<<"Loop Number "<<loop_counter<<std::endl;
        // detect all frontier groups
        if (FLOW_DEBUG) std::cout<<"Detect all frontier groups"<<std::endl;
        vector<FrontierGroup> all_f_group = robot_map.getAllFrontierGroups();
        if (all_f_group.empty()) break;

        // assign the frontiers to the robots
        if (FLOW_DEBUG)  std::cout<<"Assign the frontiers to the robots: "<<all_f_group.size()<<std::endl;
        vector<Position> robot_positions;
        for (const auto& robot : robot_list) {
            robot_positions.push_back(robot.get_position());
        }
        if (FLOW_DEBUG)  std::cout<<"Calling Central Planner"<<std::endl;
        auto frontiers_assigned = central_planner.assignFrontierGroup(all_f_group, robot_positions, robot_map.max_frontier_group_size);
        //decltype(central_planner.assignFrontierGroup(all_f_group, robot_positions, robot_map.max_frontier_group_size)) frontiers_assigned;
        //frontiers_assigned[0] = 0;
        // std::cout<<"Length of Frontiers"<<all_f_group.size()<<std::endl;
        if (FLOW_DEBUG) std::cout<<"Assigning Frontier Groups to robots"<<std::endl;
        for (int i = 0; i < robot_num; i++) {
            robot_list[i].assignFrontierGroup(all_f_group[frontiers_assigned[i]]);
        }

        // make plans for all robots
        if (FLOW_DEBUG) std::cout<<"Make plans for all robots"<<std::endl;
        vector<bool> path_executed(robot_num, false);
        bool all_robots_reached = false;
        bool robot_idle = false;
        bool last_robot_idle = false;
        int idle_time = 0;
        // std::atomic<bool> robot_idle; 
        while(!all_robots_reached) {
            all_robots_reached = true;
            robot_idle = false;
            for (int i = 0; i < robot_num; i++) {
                bool isidle = false;
                path_executed[i] = robot_list[i].executePlan(robot_map, all_f_group[frontiers_assigned[i]], isidle);
                robot_idle |= isidle;
                if (FLOW_DEBUG) std::cout<<"Path Executed"<<std::endl;
                all_robots_reached &= path_executed[i];
            }
            robot_map.timestep++;
            if (robot_map.timestep % 10 == 1) {
                // std::cout<<"\nNAME" <<run_name+"/robot_poses"+std::to_string(robot_map.timestep);
                writeRobotPoses(run_name+"/robot_poses"+std::to_string(robot_map.timestep), robot_list);
                std::ofstream output_file_intermediate;
                output_file_intermediate.open(run_name+"/map_vis_intermediate"+std::to_string(robot_map.timestep));
                output_file_intermediate << robot_map.convertToString(); 
                output_file_intermediate.close();
            }
            if(robot_idle && robot_idle != last_robot_idle) {
                idle_time = robot_map.timestep;
            }
            else if(robot_idle &&  robot_map.timestep - idle_time > 7) {
                break;
            }
            else {
                /*Do Nothing*/
            }

            last_robot_idle = robot_idle;
        }


    }


    return 0;
}