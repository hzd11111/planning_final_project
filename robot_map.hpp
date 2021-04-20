#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <optional>

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

    std::string toString() const {
        return std::to_string(x) + "," + std::to_string(y);
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
        while (current_ray_angle <= M_PI) {
            for (double range = 0.0; range <= Range; range += 0.5) {
                int delta_row = range * cos(current_ray_angle);
                int delta_col = range * sin(current_ray_angle);
                int global_row = row + delta_row;
                int global_col = col + delta_col;
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

    void addFrontier(const Position& pos) {
        frontiers.push_back(pos);
        int new_x = pos.x, new_y = pos.y;
        smallest_x = (new_x < smallest_x) ? new_x : smallest_x;
        largest_x = (new_x > largest_x) ? new_x : largest_x;
        smallest_y = (new_y < smallest_y) ? new_y : smallest_y;
        largest_y = (new_y > largest_y) ? new_y : largest_y;
        updateGroupSize();
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
                        current_map.back().emplace_back(element_value>0.5, false);
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

    RobotMap(std::string filename) {
        readFromFile(filename);
    }

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
        if (pos.getRow() >= num_rows || pos.getRow() < 0) {
            return false;
        }
        size_t num_cols = current_map[0].size();
        if (pos.getCol() >= num_cols || pos.getCol() < 0) {
            return false;
        }

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
        static constexpr int NEIGHBOUR_DELTA[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
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
        for (const auto& neighbour : all_neighbours) {
            if (!isExplored(neighbour)) {
                return true;
            }
        }
        return false;
    }

    bool isFrontier(int row, int col) const {
        isFrontier(Position::fromRowCol(row,col));
    }

    std::optional<Position> findFrontier(bool** frontiers_to_exclude,
                                         size_t& prev_row, size_t& prev_col) const {
        for (size_t i = prev_row; i < current_map.size(); ++i) {
            size_t j = (i == prev_row) ? prev_col : 0;
            for (; j < current_map[i].size(); ++j) {
                prev_row = i;
                prev_col = j;
                if (frontiers_to_exclude[i][j]) continue;

                Position cur_pos = Position::fromRowCol(i, j);
                if (isFrontier(cur_pos)) {
                    return std::optional<Position>(cur_pos); 
                }
            }
        }
        return std::nullopt;
    }

    std::vector<Position> findAllConnectingFrontiers(bool** frontiers_to_exclude,
                                    const Position& pos) {
        std::vector<Position> frontiers;
        int row = pos.getRow(), col = pos.getCol();
        if (frontiers_to_exclude[row][col]) return frontiers;
        if (!isFrontier(row, col)) return frontiers;
        frontiers_to_exclude[row][col] = true;
        auto neighbours = getNeighbours(pos);
        for (const auto& n : neighbours) {
            auto new_frontiers = findAllConnectingFrontiers(frontiers_to_exclude, n);
            frontiers.insert(frontiers.end(), new_frontiers.begin(), new_frontiers.end());
        }
        return frontiers;
    }

    std::vector<FrontierGroup> getAllFrontierGroups() const {
        bool frontiers_explored[current_map.size()][current_map[0].size()] = {false};
        size_t row = 0, col = 0;
        std::optional<Position> new_frontier = findFrontier(frontiers_explored, row, col);
        std::vector<FrontierGroup> result;
        while (new_frontier != std::nullopt) {
            // find all connecting frontiers
            auto list_of_frontiers = findAllConnectingFrontiers(frontiers_explored, new_frontier.value());

            // make frontier group
            FrontierGroup new_group;
            for (const auto& frontier : list_of_frontiers) {
                new_group.addFrontier(frontier);
            }

            result.push_back(std::move(new_group));

            new_frontier = findFrontier(frontiers_explored, row, col);
        }
        return result;
    }

};