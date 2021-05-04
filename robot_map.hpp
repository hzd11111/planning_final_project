#pragma once
#include <vector>
#include <string>
#include <climits>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <set>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include<algorithm>
#include<random>
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
#define INFLATION_RADIUS 3

enum{
    PRIOTIZED_PLANNING_FIXED = 0,
    PRIOTIZED_PLANNING_RANDOM,
    PRIOTIZED_PLANNING_CLOSEST,
};
#define PRIOTIZED_PLANNING 0

struct Position {
    int x, y;
    Position(int x_, int y_) :
        x(x_), y(y_) {}
    static Position fromRowCol(int row, int col) {
        return Position(col, row);
    }

    int getRow() const {
        return y;
    }

    int getCol() const {
        return x;
    }

    int getPositionIndex() const {
        return (y)*10000 + (x);
    }

    std::string toString() const {
        return std::to_string(x) + "," + std::to_string(y);
    }
    
    bool operator==(const Position& other) const
    {
        return (other.x == this->x) && (other.y == this->y);
    }
};

struct PositionHash {
    size_t operator() (const Position& pos) const {
        return std::hash<std::string>()(pos.toString());
    }
};

struct MapElement {
    bool obstacle, explored;
    MapElement(bool obstacle_, bool explored_):
        obstacle(obstacle_), explored(explored_) {}

    inline int intRepresentation() const {
        return explored ? (obstacle ? 1:0) : 2;
    }
};

template<int Range>
class LidarSensor {
public:
    const double ANGLE_DISC = 1.0/Range;
    inline bool updateMap(std::vector<std::vector< MapElement> >& robot_map,
                   const Position& pos) const {

        int row = pos.getRow(), col = pos.getCol();

        // if obstacle
        if (robot_map[row][col].obstacle) {
            if (robot_map[row][col].explored) {
                return false;
            } else {
                robot_map[row][col].explored = true;
                return true;
            }
        }

        // expand the sensors
        bool map_changed = false;
        double current_ray_angle = 0.0;
        while (current_ray_angle <= 2 *M_PI) {
            for (double range = 0.0; range <= Range; range += 0.5) {
                
                int delta_row = range * cos(current_ray_angle);
                int delta_col = range * sin(current_ray_angle);
                int global_row = row + delta_row;
                int global_col = col + delta_col;
                if (global_row < 0 || global_row >= robot_map.size()) break;
                if (global_col < 0 || global_col >= robot_map[0].size()) break;
                if (!robot_map[global_row][global_col].explored) {
                    robot_map[global_row][global_col].explored = true;
                    map_changed = true;
                }
                if (robot_map[global_row][global_col].obstacle) break;
            }
            current_ray_angle += ANGLE_DISC;
        }

        return map_changed;
    }
};

struct FrontierGroup {
    std::vector<Position> frontiers;
    std::vector<int> frontier_weights;
    std::unordered_map<int, int> position_to_id;

    int group_size;
    int smallest_x;
    int smallest_y;
    int largest_x;
    int largest_y;

    FrontierGroup() : group_size(-1), smallest_x(-1), smallest_y(-1), largest_x(-1), largest_y(-1) {}

    void updateGroupSize() {
        int y_diff = largest_y - smallest_y;
        int x_diff = largest_x - smallest_x;
        group_size = (y_diff > x_diff) ? y_diff : x_diff;
    }

    // void addFrontier(const Position& pos) {
    //     frontiers.push_back(pos);
    //     int new_x = pos.x, new_y = pos.y;
    //     smallest_x = (new_x < smallest_x || smallest_x < 0) ? new_x : smallest_x;
    //     largest_x = (new_x > largest_x || largest_x < 0) ? new_x : largest_x;
    //     smallest_y = (new_y < smallest_y || smallest_y < 0) ? new_y : smallest_y;
    //     largest_y = (new_y > largest_y || largest_y < 0) ? new_y : largest_y;
    //     updateGroupSize();
    // }
    void addFrontier(const Position& pos, const int score) {
        frontiers.push_back(pos);
        frontier_weights.push_back(score);

        int new_x = pos.x, new_y = pos.y;
        smallest_x = (new_x < smallest_x || smallest_x < 0) ? new_x : smallest_x;
        largest_x = (new_x > largest_x || largest_x < 0) ? new_x : largest_x;
        smallest_y = (new_y < smallest_y || smallest_y < 0) ? new_y : smallest_y;
        largest_y = (new_y > largest_y || largest_y < 0) ? new_y : largest_y;
        updateGroupSize();
    }

    void mapPositiontoIndex() {
        
        position_to_id.clear();
        for(int i=0; i<frontiers.size(); i++ ) {
            position_to_id[frontiers[i].getPositionIndex()] = i;
        }

    }
    void inflateCost(const Position& pos) {
        if (position_to_id.size() == 0) {
            mapPositiontoIndex();
        }

        // n^2 complexity
        for(int i=0; i< INFLATION_RADIUS; i++ ) {
            for(int j=0; i< INFLATION_RADIUS; i++ ) {
                int id = (pos.x + i)*10000 + (pos.y + j);
                if(position_to_id.find(id) != position_to_id.end()) {
                    frontier_weights[position_to_id[id]] += INFLATION_RADIUS - std::max(i,j);
                }
                
            }
        }

        // // Do this to be faster if array is smaller (better than n^2 complexity)
        // int id = position_to_id[pos.getPositionIndex()];
        // if(INFLATION_RADIUS*INFLATION_RADIUS < frontier_weights.size()) {
        //     for(int i=0; i< frontiers.size(); i++ ) {
        //          if((std::abs(frontiers[i].x - frontiers[id].x ) < INFLATION_RADIUS) && 
        //             (std::abs(frontiers[i].y - frontiers[id].y ) < INFLATION_RADIUS) ) {
        //                 frontier_weights[id] += INFLATION_RADIUS - std::max(std::abs(frontiers[i].x - frontiers[id].x ), std::abs(frontiers[i].y - frontiers[id].y ));
        //             }
        //     }
                
        // }

    }

    void fillScores() {
        for(int i=0; i< frontiers.size(); i++ ) {
            for(int j=i+1; j< frontiers.size(); j++ ) {
                if((std::abs(frontiers[i].x - frontiers[j].x ) < INFLATION_RADIUS) && 
                    (std::abs(frontiers[i].y - frontiers[j].y ) < INFLATION_RADIUS) ) {
                        frontier_weights[i]++;
                        frontier_weights[j]++;
                    }
            }
            frontier_weights[i] = std::max(2*INFLATION_RADIUS - frontier_weights[i], 1);
        }
    }

};

template<typename Sensor>
class RobotMap {
private:
    void readFromFile(std::string filename) {

        // open the file
        std::ifstream map_file(filename);

        if (map_file.is_open()) {
            std::string line;
            while (getline(map_file, line)) {
                if (line != "\n") {
                    std::istringstream token_stream(line);
                    std::string token;
                    current_map.emplace_back();
                    while (getline(token_stream, token, ',')) {
                        double element_value = std::stod(token);
                        current_map.back().emplace_back(element_value>2.0, false);
                    }
                }
            }
            map_file.close();
        } else {
            std::cout<<"File "<<filename<<"not exists"<<std::endl;
        }

    }
public:
    std::vector<std::vector< MapElement> > current_map;
    Sensor sensor;
    int max_frontier_group_size = 0;
    std::vector<Position> robot_poses;
    std::vector<std::vector<std::set<int>>> robot_occupancy_map;
    int timestep = 0;

    RobotMap(std::string filename) {
        readFromFile(filename);
        robot_occupancy_map.resize(current_map.size(), std::vector<std::set<int>> (current_map[0].size()));
    }

    //Sachit: Can just be replaced by number of explored 
    std::string convertToString() const {
        std::string result = "";
        bool first_row = true;
        for (const auto& row : current_map) {
            if (!first_row) {
                result.append("\n");
            } else {
                first_row = false;
            }
            bool first_element = true;
            for (const auto& element : row) {
                if (!first_element) {
                    result.append(",");
                } else {
                    first_element = false;
                }
                result.append(std::to_string(element.intRepresentation()));
            }
        }
        return result;
    }

    bool updateExploration(const Position& pos) {
        return sensor.updateMap(current_map, pos);
    }

    inline bool inMapRange(const Position& pos) const {
        size_t num_rows = current_map.size();
        //std::cout<<"Num Rows: "<<num_rows<<std::endl;
        if (pos.getRow() >= num_rows || pos.getRow() < 0) {
            return false;
        }
        size_t num_cols = current_map[0].size();
        //std::cout<<"Num Col: "<<num_cols<<std::endl;
        if (pos.getCol() >= num_cols || pos.getCol() < 0) {
            return false;
        }
        //std::cout<<"True"<<std::endl;

        return true;
    }

    inline bool isObstacle(const Position& pos) const {
        if(!inMapRange(pos)) return false;
        return current_map[pos.getRow()][pos.getCol()].obstacle;
    }

    inline bool isExplored(const Position& pos) const {
        if(!inMapRange(pos)) return false;
        return current_map[pos.getRow()][pos.getCol()].explored;
    }

    inline std::vector<Position> getNeighbours(const Position& pos) const {
        static constexpr int NEIGHBOUR_DELTA[24][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
        std::vector<Position> neighbours;
        for (const auto& delta : NEIGHBOUR_DELTA) {
            Position new_neighbour(pos.x + delta[0], pos.y + delta[1]);
            if (inMapRange(new_neighbour)) {
                neighbours.push_back(std::move(new_neighbour));
            }
        }
        return neighbours;
    }

    inline std::vector<Position> getAdjacentCells(const Position& pos) const {
        static constexpr int NEIGHBOUR_DELTA[24][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1},
            {-2,-2}, {-2,-1}, {-2, 0}, {-2, 1}, {-2, 2}, {-1, -2}, {-1, 2}, {0, -2}, {0, 2} ,{1, -2} ,{1, 2} ,
            {2, -2}, {2, -1} ,{2, 0} ,{2, 1} ,{2, 2}};
        std::vector<Position> neighbours;
        for (const auto& delta : NEIGHBOUR_DELTA) {
            Position new_neighbour(pos.x + delta[0], pos.y + delta[1]);
            if (inMapRange(new_neighbour)) {
                neighbours.push_back(std::move(new_neighbour));
            }
        }
        return neighbours;
    }

    bool isFrontier(const Position& pos) const {
        if(!isExplored(pos)) return false;
        if(isObstacle(pos)) return false;
        std::vector<Position> all_neighbours = getNeighbours(pos);
        //std::cout<<"Position: "<<pos.toString()<<std::endl;
        for (const auto& neighbour : all_neighbours) {
            //std::cout<<"Neighbour: "<<neighbour.toString()<<" Is Explored: "<<isExplored(neighbour)<<std::endl;
            if (!isExplored(neighbour)) {
                return true;
            }
        }
        return false;
    }

    bool isFrontier(int row, int col) const {
        return isFrontier(Position::fromRowCol(row,col));
    }

    std::optional<Position> findFrontier(bool** frontiers_to_exclude,
                                         size_t& prev_row, size_t& prev_col) const {
        for (int i = prev_row; i < current_map.size(); ++i) {
            size_t j = (i == prev_row) ? prev_col : 0;
            
            for (; j < current_map[i].size(); ++j) {
                prev_row = i;
                prev_col = j;
                if (frontiers_to_exclude[i][j]) {
                    continue;
                }

                Position cur_pos = Position::fromRowCol(i, j);
                if (isFrontier(cur_pos)) {
                    return std::optional<Position>(cur_pos); 
                }
            }
        }
        return std::nullopt;
    }

    std::pair<std::vector<Position>, std::vector<int>> findAllConnectingFrontiers(bool** frontiers_to_exclude,
                                    const Position& pos) const {
        std::pair<std::vector<Position>, std::vector<int>> frontiers;
        int row = pos.getRow(), col = pos.getCol();
        if (frontiers_to_exclude[row][col]) return frontiers;
        if (!isFrontier(row, col)) return frontiers;
        frontiers_to_exclude[row][col] = true;
        frontiers.first.push_back(pos);
        frontiers.second.push_back(-1);
        int index_curr_neigh = frontiers.second.size()-1;
        auto neighbours = getAdjacentCells(pos);
        decltype(neighbours) more_neighbours;

        /*for (int i = 0; i < 1; i++) {
            more_neighbours = decltype(neighbours)();
            for (const auto& n : neighbours) {
                auto neighbours_of_neighbours = getAdjacentCells(n);
                more_neighbours.insert(more_neighbours.end(), neighbours_of_neighbours.begin(), neighbours_of_neighbours.end());
            }
            neighbours = std::move(more_neighbours);
        }*/

        int score = 0;
        for (const auto& n : neighbours) {
            auto new_frontiers = findAllConnectingFrontiers(frontiers_to_exclude, n);
            // if(isObstacle(n)) {
            //     score--; //Can be further reduced to make robots stay away from obstacles TOFO:Tune this
            // }
            frontiers.first.insert(frontiers.first.end(), new_frontiers.first.begin(), new_frontiers.first.end());
            frontiers.second.insert(frontiers.second.end(), new_frontiers.second.begin(), new_frontiers.second.end());
        }
        frontiers.second[index_curr_neigh] = score;

        return frontiers;
    }

    std::vector<FrontierGroup> getAllFrontierGroups() {
        int num_row = current_map.size();
        int num_col = current_map[0].size();
        bool** frontiers_explored = new bool *[num_row];
        for (int i = 0; i < num_row; ++i) {
            frontiers_explored[i] = new bool[num_col]();
            memset(frontiers_explored[i], false, num_col*sizeof(bool));
        }
        size_t row = 0, col = 0;
        std::optional<Position> new_frontier = findFrontier(frontiers_explored, row, col);
        std::vector<FrontierGroup> result;
        while (new_frontier != std::nullopt) {
            // find all connecting frontiers
            auto list_of_frontiers = findAllConnectingFrontiers(frontiers_explored, new_frontier.value());

            // make frontier group
            FrontierGroup new_group;
            int index = 0;
            for (const auto& frontier : list_of_frontiers.first) {
                new_group.addFrontier(frontier, list_of_frontiers.second[index++]);
            }
            // new_group.mapPositiontoIndex();
            // new_group.fillScores(); Done Later When Chosen

            max_frontier_group_size = std::max(max_frontier_group_size, new_group.group_size );
            result.push_back(std::move(new_group));

            new_frontier = findFrontier(frontiers_explored, row, col);
        }
        for (int i = 0; i < num_row; ++i) {
            delete[] frontiers_explored[i];
        }
        delete[] frontiers_explored;
        return result;
    }

};

// typedef std::vector<std::vector<int>> weight_map;

class CentralPlanner {
public:
    
    int num_robots;
    double alpha;
    double beta;
    double gamma;
    double lamda = 0; //For a future probabilistic model
    //per robot Heuristic
    // std::vector<weight_map> g_map;
    

    // 1, 10, 10
    CentralPlanner(double alpha, double beta, double gamma, int num_robot):
        num_robots(num_robot) {
        this->alpha = alpha;
        this->beta = beta;
        this->gamma = gamma;
    }
    /* Distance of robot to each FG (based on closest Frontier) */

    int getRobotFrontierDistance(Position robot_loc, Position frontier_loc) {
        return std::max( std::abs(robot_loc.x - frontier_loc.x), std::abs(robot_loc.y - frontier_loc.y)  ); 
    }

    std::unordered_map<int, std::vector<std::pair<int, int>>> getRobotFrontierGroupDistance(std::vector<FrontierGroup>& frontier_groups, std::vector<Position>& robot_positions, std::vector<std::pair<int,int>>& closest_to_robot ) {
        std::unordered_map<int, std::vector<std::pair<int, int>>> frontier_group_distance;
        for(int i=0 ; i< num_robots; i++) {  
            for(int j = 0; j< frontier_groups.size() ; j++ )  { // Number Groups
                std::pair<int, int> temp_id_dist = {0,INT_MAX}; 
                for(int k =0 ; k< frontier_groups[j].frontiers.size(); k++){ // Number Frontier in Group
                    int distance_to_robot = getRobotFrontierDistance(robot_positions[i], frontier_groups[j].frontiers[k]);
                    if(temp_id_dist.second > distance_to_robot) {
                        temp_id_dist.first = j;
                        temp_id_dist.second = distance_to_robot;
                    }
                }
                frontier_group_distance[i].push_back(temp_id_dist); 
                if(closest_to_robot.size() >  i) {
                    closest_to_robot[i].first = std::min(closest_to_robot[i].first, temp_id_dist.second);
                }  
                else {
                    closest_to_robot.push_back({temp_id_dist.second, i});
                }   
            }
            
        }

        return frontier_group_distance;

    }

    std::unordered_map<int, int> assignFrontierGroup(std::vector<FrontierGroup>& frontier_groups, std::vector<Position>& robot_positions, int max_frontier_group_size) {
        std::unordered_map<int, int> robot_frontier_group;
        

        #if PRIOTIZED_PLANNING == 1
            /*randomized starting */
            std::vector<int> shuffled;
            for(int i=0 ; i< num_robots; i++) {
                shuffled.push_back(i);
            }
            auto rd = std::random_device {};
            auto rng = std::default_random_engine {rd()};
            std::shuffle(std::begin(shuffled), std::end(shuffled), rng);
        #endif

        //{Closeness to frontier, RobotID}
        std::vector<std::pair<int,int>> closest_to_robot; // /*prioritized starting */
        auto frontier_group_distance = getRobotFrontierGroupDistance(frontier_groups, robot_positions, closest_to_robot);

        #if PRIOTIZED_PLANNING == 2
            /*prioritized starting */
            std::sort(closest_to_robot.begin(), closest_to_robot.end());

            if(frontier_groups.size() <= 1) {
                for(int i=0 ; i< num_robots; i++) {
                    robot_frontier_group[i] = 0;
                }
                return robot_frontier_group;
            }
        #endif


        std::unordered_map<int, int> num_robots_assigned_to_frontier;
        for(int i=0 ; i< num_robots; i++) {
            double score = INT_MAX; 
            int chosen_frontier_group_id = 0;
            int rid = i;
            #if PRIOTIZED_PLANNING == 1
                rid = shuffled[i];
            #elif PRIOTIZED_PLANNING == 2
                rid = closest_to_robot[i].second;
            #endif

            for(int j = 0; j< frontier_groups.size() ; j++ ) { //Frontier Groups
            // std::cout<<"\nRobot :"<<i<<" Frontier :"<<j<<", Closest :"<<frontier_group_distance[rid][j].second<<" P: "<< frontier_groups[j].frontiers[frontier_group_distance[rid][j].first].toString();
                // alpha * COST + beta * (IMPORTANCE) + gamme * (CURIOSITY) + lamda*REPETITION [last is zero]
                double frontier_group_score = alpha * (frontier_group_distance[rid][j].second) - beta * (frontier_groups[j].group_size/max_frontier_group_size) + gamma * (num_robots_assigned_to_frontier[j]);
                if (score > frontier_group_score) {
                    score = frontier_group_score;
                    chosen_frontier_group_id = j;
                }
            }
            // std::cout<<"\nChosen: "<< chosen_frontier_group_id<<std::endl;
            robot_frontier_group[rid] = chosen_frontier_group_id;
            if(!num_robots_assigned_to_frontier[chosen_frontier_group_id]) {
                frontier_groups[chosen_frontier_group_id].mapPositiontoIndex();
                frontier_groups[chosen_frontier_group_id].fillScores();
            }
            num_robots_assigned_to_frontier[chosen_frontier_group_id]++;

        }

        return robot_frontier_group;
    }    
};

//TESTS 
//1,2,3
// 3 Our, Greedy, ours+switch

//MAP update -> Frontiere Update
/* After robot runs lets keep same frontier group and just do a isfrontier check and then Update when 
    -> all n robots reach froniter
    -> No member left in frontier group

Better way
If map is updated at each timestep
Map frontier from list to fronier group and then keep on culling all visible sections [Too time complex]

//FUTURE EXPANSION
 -> Incremental search (LEC8)
 -> Non Real Time ARA to use other robots searches
 -> If num robots > 2 do backward then forward [Do Time Comparision]
*/