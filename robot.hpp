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
#include "robot_map.hpp"

#define LIDAR_RANGE 5

using namespace std;

// template<typename Sensor>
struct PlannerMap {

    vector<vector<set<int>>> robot_occupancy_map; //keys are timestamps
    RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map;
    int timestamp;

    PlannerMap(RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map_inc): robot_map(robot_map_inc) {
        // this->robot_map = robot_map;
        // initialize occupancy map
        size_t num_rows = robot_map.current_map.size();
        size_t num_cols = robot_map.current_map[0].size();

        this->robot_occupancy_map.resize(num_rows, vector<set<int>>(num_cols));
        // TODO: where do we initialize the intial robot state
        this->timestamp = 0;
    }

    int size(){
        return robot_map.current_map.size() * robot_map.current_map[0].size();
    }

};

struct Node {
    int index;
    Position pos = Position(-1,-1);
    float dist; 
    Position parent_pos = Position(-1,-1);
    int parent_node_int;
    std::vector<Position> neighbours;

    // Constructor 
    Node(int index, Position pos, float dist, Position parent_pos) {
        this->index = index; this->pos = pos; this->dist=dist; 
        this->parent_pos = parent_pos;
    }

    bool operator==(const Node& other)
    {
        return (other.pos.x == this->pos.x) && (other.pos.y == this->pos.y);
    }

};

struct NodeComparator	{   
    bool operator()(Node const& a, Node const& b) const
    {
        return a.dist > b.dist;
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
        deque<Position> planned_traj; //ALVIN, why deque vs a vector?
        // vector<Position> assigned_frontiers;
        vector<FrontierGroup> assigned_frontier_groups;
        // vector<Position> remaining_assigned_frontiers;
        // Position robot_position; //same as last elem of traj_hist

        double calcDistance(Position a, Position b) {
            return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
        }

    public:
        string id;

        Robot(string id, Position start_position) {
            this->id = id;
            this->traj_history.push_back(start_position);
        }
        
        void resetFrontierGroups() { 
            // resets the frontiers and the plannet traj
            this->assigned_frontier_groups = vector<FrontierGroup>();
        }

        void assignFrontierGroup(FrontierGroup frontier_group) {
            this->assigned_frontier_groups.push_back(frontier_group);
        }
       
        bool isTraversable(const Position& pos, const PlannerMap& planner_map) {
            // traversable positions are explored, obstacle free and in the map range
            RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map = planner_map.robot_map;
            return robot_map.isExplored(pos) && !(robot_map.isObstacle(pos)) && robot_map.inMapRange(pos);
        }

        pair<Position, int> getClosestFrontier(Position pos) { 
            // assumes 8-connected grid, returns distance in float and parent frontier group index
            // calculate the Euclidean distance to the nearest frontier of all the frontier groups

            pair<Position, int> closest_frontier = make_pair(pos, -1); // pair<distance, parent group index>
            int parent_frontier_ind = 0;
            double closest_frontier_distance = __DBL_MAX__;

            for (auto frontier_group : assigned_frontier_groups) {
                for (auto frontier : frontier_group.frontiers) {
                    
                    double distance = calcDistance(frontier, pos);
                    
                    if (distance < closest_frontier_distance) {
                        closest_frontier = make_pair(frontier, parent_frontier_ind);
                        closest_frontier_distance = distance;
                    }
                }
                parent_frontier_ind++;
            }
            
            return closest_frontier;
        }

        void planToClosestFrontier(PlannerMap& planner_map) {

            // reset planned traj                
            this->planned_traj = deque<Position>();  //Alvin check
            
            // create start node
            Position start_position = traj_history.back();
            Node start_node = Node(-1, start_position, 0, start_position); // parent pos same as node pos

            // find the closest frontier out of all frontier groups and create goal node with it
            pair<Position, int> closest_frontier = getClosestFrontier(start_position);
            double goal_distance = calcDistance(closest_frontier.first, start_position);
            Node goal_node = Node(1, closest_frontier.first, goal_distance, closest_frontier.first);

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
                
                // check for goal
                if (curr_node == goal_node) {goal_found = true; goal_node = curr_node; break;}

                // check if node is already visited
                bool node_not_yet_expanded = (visited.find(curr_node) == visited.end());
                if(node_not_yet_expanded)
                {
                    // insert curr node to visited
                    visited.insert({curr_node, curr_node});

                    // get all the neighbors
                    vector<Position> neighbours_positions = planner_map.robot_map.getNeighbours(curr_node.pos);

                    // loop over all the neighbors
                    while(!neighbours_positions.empty())
                    {
                        // skip node if it isn't traversable 
                        if (!isTraversable(neighbours_positions.back(), planner_map)) {
                            neighbours_positions.pop_back(); 
                            continue; 
                        }

                        double distance = calcDistance(neighbours_positions.back(), start_position);
                        Node neighbour = Node(0, neighbours_positions.back(), distance, curr_node.pos);
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

                auto curr_node = goal_node;
                // backtrack till the start node, dont push start node to planned traj
                while (!(curr_node.pos == curr_node.parent_pos)){
                    planned_traj.push_front(curr_node.pos);     
                    // auto parent_iterator = visited.find(Node(-1, curr_node.parent_pos, -1.0, curr_node.parent_pos));
                    curr_node = visited.at(Node(-1, curr_node.parent_pos, -1.0, curr_node.parent_pos));
                }
            }
            else {
                cout << "GOAL NOT FOUND!" << endl;
            }

        }

        Position getPosition(int index) { 
            return traj_history[index];
        }

        Position get_position() { 
            return traj_history.back();
        }

        bool executePlan(RobotMap<LidarSensor<LIDAR_RANGE>>& robot_map) { 
            // take planned traj and keep updating traj_history till goal is found
            // return false if goal is already reached
            if (planned_traj.size() > 0) {
                traj_history.push_back(planned_traj[0]);
                robot_map.updateExploration(planned_traj[0]);
                planned_traj.pop_front();
                return true;
            }
            else {
                return false;
            }
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

             