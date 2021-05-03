#pragma once
#include <vector>
#include <set>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_map>
#include<algorithm>
#include "robot_map.hpp"

#define LIDAR_RANGE 5

using namespace std;

// template<typename Sensor>
// struct PlannerMap {

//     vector<vector<set<int>>> robot_occupancy_map; //keys are timestamps
//     RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map;
//     int timestamp;

//     PlannerMap(RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map_inc): robot_map(robot_map_inc) {
//         // this->robot_map = robot_map;
//         // initialize occupancy map
//         size_t num_rows = robot_map.current_map.size();
//         size_t num_cols = robot_map.current_map[0].size();

//         this->robot_occupancy_map.resize(num_rows, vector<set<int>>(num_cols));
//         // TODO: where do we initialize the intial robot state
//         this->timestamp = 0;
//     }

//     int size(){
//         return robot_map.current_map.size() * robot_map.current_map[0].size();
//     }

// };

struct Node {
    int index;
    Position pos = Position(-1,-1);
    int g;
    int h = 0; 
    Position parent_pos = Position(-1,-1);
    int parent_node_int;
    std::vector<Position> neighbours;

    // Constructor 
    Node(int index, Position pos, float g, Position parent_pos) {
        this->index = index; //this->pos = pos; 
        this->g=g; 
        this->parent_pos = parent_pos;
    }
    Node(){}

    bool operator==(const Node& other)
    {
        return (other.pos.x == this->pos.x) && (other.pos.y == this->pos.y);
    }

};

struct NodeComparator	{   
    bool operator()(Node const& a, Node const& b) const
    {
        return (a.g + a.h) > (b.g + b.h);
    }
};

struct NodeHasher {
    size_t operator() (const Node& node) const {
        return std::hash<std::string>()(node.pos.toString());
    }
};

class Robot {
    private:
        vector<Position> traj_history;
        deque<Position> planned_traj; 
        bool reached_any_frontier = false;

        // FrontierGroup& assigned_frontier_group;
        unordered_set<Position, PositionHash> frontiers_map;
        unordered_map<Position, int, PositionHash> frontiers_weights_map; 

        double calcDistance(Position a, Position b) {
            return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
        }

    public:
        string id;

        Robot(string id, const Position& start_position) {
            this->id = id;
            this->traj_history.push_back(start_position);
        }

        void assignFrontierGroup(FrontierGroup& assigned_frontier_group) {
            // this->assigned_frontier_groups.push_back(frontier_group);
            // this->assigned_frontier_group = frontier_group;
            planned_traj.clear(); frontiers_map.clear(); frontiers_weights_map.clear();
            reached_any_frontier = false;

            int index = 0;
            for (Position frontier : assigned_frontier_group.frontiers) {
                frontiers_map.insert(frontier);
                frontiers_weights_map[frontier] = assigned_frontier_group.frontier_weights[index];
                index++;
            }
        }
       
        bool isTraversable(const Position& pos, const RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map) {
            // traversable positions are explored, obstacle free and in the map range
            // RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map = planner_map.robot_map;
            return robot_map.inMapRange(pos) && robot_map.isExplored(pos) && !(robot_map.isObstacle(pos));
        }

        void planToClosestFrontier(RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map, FrontierGroup& assigned_frontier_group) {

            // reset planned traj                
            this->planned_traj = deque<Position>();  //Alvin check
            
            // create start node
            Position start_position = traj_history.back();
            Node start_node = Node(-1, start_position, 0, start_position); // parent pos same as node pos

            // find the closest frontier out of all frontier groups and create goal node with it
            Node goal_node = Node();

            // initialize A*
            std::priority_queue<Node, vector<Node>, NodeComparator> open_list; 
            open_list.push(start_node);
            // Node curr_node;
            unordered_map<Node, Node, NodeHasher, NodeComparator> visited;
            visited.insert({start_node, start_node});
            bool goal_found = false;

            // start planning outwards till you hit the closest frontier
            while(!open_list.empty())
            {
                auto curr_node = open_list.top();
                open_list.pop();
                
                // check if curr_node is a frontier, if so exit. 
                if (frontiers_map.find(curr_node.pos) != frontiers_map.end()) {
                    goal_found = true; goal_node = curr_node; 
                    // auto &frontiers = assigned_frontier_group.frontiers;
                    // pop the frontier from the list of frontiers
                    // assigned_frontier_group.frontier_weights.erase(std::find(assigned_frontier_group.frontiers.begin(), assigned_frontier_group.frontiers.end(), curr_node.pos));
                    
                    // TODO: Delete weights from the 
                    // assigned_frontier_group.frontiers.erase(std::find(assigned_frontier_group.frontiers.begin(), assigned_frontier_group.frontiers.end(), curr_node.pos));
                    assigned_frontier_group.frontiers.erase(assigned_frontier_group.frontiers.begin() + assigned_frontier_group.position_to_id[curr_node.pos.getPositionIndex()]);
                    assigned_frontier_group.frontier_weights.erase(assigned_frontier_group.frontier_weights.begin() + assigned_frontier_group.position_to_id[curr_node.pos.getPositionIndex()]);
                    assigned_frontier_group.mapPositiontoIndex();
                    frontiers_map.erase(curr_node.pos);
                    break;
                }

                // check if node is already visited
                bool node_not_yet_expanded = (visited.find(curr_node) == visited.end());
                if(node_not_yet_expanded)
                {
                    // insert curr node to visited
                    visited.insert({curr_node, curr_node});

                    // get all the neighbors
                    vector<Position> neighbours_positions = robot_map.getNeighbours(curr_node.pos);

                    // loop over all the neighbors
                    while(!neighbours_positions.empty())
                    {
                        Position neighbour_pos = neighbours_positions.back();
                        // skip node if it isn't traversable 
                        if (!isTraversable(neighbour_pos, robot_map)) {
                            neighbours_positions.pop_back(); 
                            continue; 
                        }

                        int g_value = curr_node.g + 1;
                        Node neighbour = Node(0, neighbour_pos, g_value, curr_node.pos);
                        if (frontiers_map.find(neighbour_pos) != frontiers_map.end()) {
                            neighbour.h = frontiers_weights_map[neighbour_pos];
                        }
                        else 
                        {
                            neighbour.h = 0;
                        }
                        neighbours_positions.pop_back();
                        bool node_not_yet_expanded = (visited.find(neighbour) == visited.end());
                        
                        // a lower cost path cannot be found since distances per move are constant
                        if(node_not_yet_expanded)
                        {
                            open_list.push(neighbour);
                        }
                    }
                }
            }

            // backtrack
            if (goal_found) {
                // reset the planned trajectory
                planned_traj = deque<Position>();
                assigned_frontier_group.inflateCost(goal_node.pos); 

                auto curr_node = goal_node;
                // backtrack till the start node, dont push start node to planned traj
                while (!(curr_node.pos == curr_node.parent_pos)){
                    planned_traj.push_front(curr_node.pos);     
                    // auto parent_iterator = visited.find(Node(-1, curr_node.parent_pos, -1.0, curr_node.parent_pos));
                    curr_node = visited.at(Node(-1, curr_node.parent_pos, -1.0, curr_node.parent_pos));
                }
                
                // update the frontiers map
                frontiers_weights_map.clear();
                int index = 0;
                for (Position frontier : assigned_frontier_group.frontiers) {
                    frontiers_weights_map.insert({frontier, assigned_frontier_group.frontier_weights[index]});
                    index++;
                }
            }
            else {
                cout << "GOAL NOT FOUND!" << endl;
            }

        }

        Position getPosition(int index) const { 
            return traj_history[index];
        }

        Position get_position() const { 
            return traj_history.back();
        }

        bool executePlan(RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map, FrontierGroup& assigned_frontier_group) { 
            // take planned traj and keep updating traj_history till goal is found
            // return false if no frontier is reached
            
            // check if planned_traj has steps remaining, otherwise replan if frontiers remaining
            if (planned_traj.size() == 0 && assigned_frontier_group.frontiers.size() == 0) {
                return true; // no steps left to execute
            }
            else if (planned_traj.size() == 0 && assigned_frontier_group.frontiers.size() > 0) {
                planToClosestFrontier(robot_map, assigned_frontier_group);
            }
            
            // check if the step is collision free, otherwise skip moving for this timestep
            set<int> &occupancy_set = robot_map.robot_occupancy_map[planned_traj[0].x][planned_traj[0].y];
            bool position_occupied = occupancy_set.find(robot_map.timestep + 1) != occupancy_set.end();
            if (position_occupied) {return reached_any_frontier;}


            // once planned traj has steps to execute and the position is obstacle free
            traj_history.push_back(planned_traj[0]);
            robot_map.updateExploration(planned_traj[0]);
            occupancy_set.insert(robot_map.timestep + 1);
            planned_traj.pop_front();
            if (planned_traj.size() == 0) {reached_any_frontier = true;}
            
            return reached_any_frontier;
        } 

};







/* Discarded Code
// pair<bool, int> isAssignedFrontier(PlannerMap& planner_map,Position pos, bool fromRemainingFrontiersOnly=true) { 
    
//     if (!(planner_map.robot_map.isFrontier(pos))) {return false}

//     vector<Position> frontier_list;

//     if (fromRemainingFrontiersOnly)
//     {   
//         frontier_list = remaining_assigned_frontiers;
//     }
//     else {
//         frontier_list = assigned_frontiers;
//     }

//     int index = 0;
//     for (const auto& frontier : frontier_list) {
//         index++;
//         if ((frontier.x == pos.x) && (frontier.y == pos.y)) {
//             return make_pair(true, index);
//         }
//     }
//     else make_pair(false, -1);
// }
*/

             