#ifndef __DSTAR_LITE_PLANNER__
#define __DSTAR_LITE_PLANNER__

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>
#include "path_planner/default_planner.hpp"
#include "path_planner/map_handler.hpp"
#include "path_planner/utils.hpp"



class DStarLitePlanner : public DefaultPlanner
{
    struct Cell : Tile 
    {
        public:
            double g_{1e10};
            double rhs_{1e10};
            Key key_;
            ID parent_id_;
            double cell_size_;
            double sl_cost_{0};  
            double height_{0};
            double distance_cost_{0}; 
            double turn_cost_{0};
            double roughness_{0};
            bool is_obstacle_{false};
            bool hidden_obstacle_{false};
            double masked_height_{0};   

    };

public:
    DStarLitePlanner(); 
    ~DStarLitePlanner();

    Cell iter_cell_;
    double cell_size_;

    std::map<ID,Cell> planning_map_;
    std::map<ID,Tile> universal_map_;
    MapHandler map_hd_;
    Key iter_key_;
    int k_m_; //key modifier
    ID start_node_;
    ID goal_node_;
    ID last_node_;
    double omega_{0};          //weight for slopes
    double mu_{0};             // steps threshold
    double gamma_{0};         //weight for roughness
    double beta_{0};  //weight for turns 

    int max_num_cel_x,max_num_cel_y;  //for results

    std::vector<ID> unordered_path_;
    std::vector<ID> path_traced_id_;
    bool forward_path_found_{false};

    int max_compute_path_iter{80000};
    int max_trace_path_iter{80000};

    std::vector<Key> queue_;
    std::vector<std::pair<int,int>> obstacle_list_;
    bool path_changed_{false};

    bool planForward(ID start, ID goal);
    bool planForward(ID start, ID goal, std::vector<std::vector<int>> obstacle_array, double omega);
    bool planForward(ID start, ID goal, double omega, double beta, double gamma, double mu); 
    void printMap();
    void printQueue();
    void printQueue(std::vector<Key> &queue);
    
    void addToQueue(std::vector<Key> &queue, Key key);
    int calculateHeuristic(ID current_id);
    Key calculateKey(ID &current_id, int k_m);
    double calTurnAngle(ID &current_id,  ID &target_id);

    void initialize(std::vector<Key> &queue, ID s_start, ID s_goal, int k_m);
    std::vector<ID> getNeighbor(ID u);
    bool compareCoordinate(ID u, ID v);
    double cost(ID target_id, ID current_id);
    void updateVertex( std::vector<Key> &queue, ID current_id, int k_m);

    bool compareKey(Key key1, Key key2);           //return true if key1 < key2
    bool matchID(ID id1, ID id2);                  //return true if same id 
    Key getTopKey(std::vector<Key> &queue);       //return min key in queue
    Key popTopKey(std::vector<Key> &queue);       //basically work the same as getTopKey but also pop the top key out
    void computeShortestPath(std::vector<Key> &queue,ID s_start, ID s_goal, int k_m );
    bool isNeighbor(ID id1, ID id2);
    void traceBackPath();
    std::vector<ID> scanMap(ID current_id, int range);   // return list of id has changed
    bool moveAndScan();                             // return true while still be able to move, false if goal reached

    void addObstacle(std::vector<std::vector<int>> obstacle_array);
    void addHiddenObstacle(std::vector<std::vector<int>> hidden_obstacle_array);

    void importMap(MapHandler& map_hd);
    void transferMap(std::map<ID,Tile> base_planning_map);
    
    void createTestMap();
    void clearMap();
    // void clearCells();
    void runTestFunc();
    
    std::map<ID,Tile> convertToUniversalMap();
protected:
    rclcpp::Node::SharedPtr node_;

};
#endif