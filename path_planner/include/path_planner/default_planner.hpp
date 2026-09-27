#ifndef __DEFAULT_PLANNER__
#define __DEFAULT_PLANNER__

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include "path_planner/map_handler.hpp"
#include "path_planner/msg/all_info.hpp"
#include "path_planner/utils.hpp"



class DefaultPlanner
{
    struct Cell : Tile
    {
        public:
            double g_cost_{1e10};          // g_cost is distant from the start node to current  
            double h_cost_{1e10};          // h_cost is distant to the goal node from current  
            double f_cost_{1e10};          // g_cost + h_cost  or m_cost_ + h_cost_
            double m_cost_{1e10};          // center mean cost , at this point mean is height
            double height_{0};              // height of cell
            double cell_size_{0};              // side length of cell
            double sl_cost_{0};             // slope cost   
            double distance_cost_{0}; 
            double turn_cost_{0};
            double confidence_score_{0};
            double roughness_{0};
            int id_x_, id_y_;
            ID parent_id_;
            int parent_id_x_,parent_id_y_;
            bool is_goal_{false};
            bool is_start_{false};
            bool is_explored_{false};
            bool is_exploring_{false};
            bool is_obstacle_{false};
            bool hidden_obstacle_{false};       // is this cell a hidden obstacle or not, this will be discovered only at scanning stage
            double masked_height_{0};                 // the real height of this cell, this will be discovered only at scanning stage
            bool is_unknown_{false};
            void setCellNewCost(double new_m_cost,double new_g_cost,double new_h_cost,double new_f_cost,int new_parent_id_x, int new_parent_id_y,  double slope_cost);
            void setCellNewCost(double new_m_cost,double new_g_cost,double new_h_cost,double new_f_cost,ID parent_id,  double slope_cost, double distance_cost);
    };
    struct PlanningMap
    {
        public:

            std::vector<std::vector<Cell>> cell_map;
            std::vector<std::vector<int>> obstacle_cell_list;
            std::vector<std::vector<int>> map_id_list;
            std::vector<std::vector<int>> explored_cell_list;
            std::vector<std::vector<int>> exploring_cell_list;
            std::vector<std::vector<int>> exploring_cell_list_tmp;
            double cell_size_;
            void resetExploreLists();
            void resetCellMapCost();
            int test_var = {23};
    };
    struct Heuristic
    {
        public:
            std::vector<std::vector<int>> rook{{0,1},{0,-1},{1,0},{-1,0}}; 
            std::vector<std::vector<int>> bishop{{1,1},{1,-1},{-1,1},{-1,-1}};
            std::vector<std::vector<int>> queen{{0,1},{0,-1},{1,0},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}};

    };
    
public:
    DefaultPlanner() ; 
    ~DefaultPlanner();



    void transferMap(std::map<ID,Tile> base_planning_map);
    bool planForward(ID start, ID goal, double omega, double beta, double gamma, double mu); 
    std::vector<ID> path_traced_id_;
    void updateMap(std::vector<ID> changed_id);

    std::map<ID,Cell> planning_map_constructing_;

    //cutting from here, from this line variables and functions will not be used
    bool received_new_msgs_ = false;

    // map handler 
    MapHandler map_hd_;

    // Cell iter_cell;
    ID start_node_;
    ID goal_node_;
    ID current_node_;
    ID target_node_;
    double cell_size_;
    int max_num_cel_x,max_num_cel_y;

    Heuristic default_h;
    std::vector<std::vector<int>> d_heuristic_ = default_h.queen;

    // astar functions and variables  
    std::pair<int, int> start_cell;
    double current_min_fcost{0};
    double max_slope_threshold_;
    double gamma_weight_;           //weight for roughness
    double beta_weight_;           //weight for turn

    //for obstacle
    std::vector<std::vector<int>> static_obstacle_;

    //for iteration
    double distance_sqr_ ;
    double distance_h_sqr_ ;
    double m_current_ , g_current_, h_current_, f_current_;
    double slope_cost_current_;
    double distance_cost_current_;
    double turn_cost_current_;
    double m_goal_, m_finalgoal_;

    //modification for cost 
    double w_weight_heigh_{1};
    double w_weight_hcost_{1};
    double w_weight_gcost_{1};
    double omega_{0};
    
    //id of current iterating cell , 
    int cur_id_x_ , cur_id_y_, goal_id_x_, goal_id_y_;
    //iterate mincost value
    double min_cost_h_, min_cost_f_;
    //id of the final goal
    int start_id_x_{-1}, start_id_y_{-1},finalgoal_id_x_{-1}, finalgoal_id_y_{-1};
    bool found_path_{false};
    std::vector<std::vector<int>> path_traced_;
    // std::vector<ID> path_traced_id_;
    //for result
    double final_f_cost_;
    double final_distance_cost_;
    int path_color_{6} ;

    int max_compute_path_iter{50000};

    void setParam(double max_slope_threshold, double gamma, double beta);
    void importMap(MapHandler& map_hd);
    void importMap();
    // void transferMap(std::map<ID,Tile> base_planning_map);
    void addObstacle(std::vector<std::vector<int>> obstacle_array);
    void addHiddenObstacle(std::vector<std::vector<int>> hidden_obstacle_array);
    void addMaskedMap(std::vector<std::vector<int>> masked_map_array);
    void setStartGoalID(ID start, ID goal );    //id based newly added 
    bool checkOKStartGoal();
    double cellToCellCost(double x_cur,double y_cur,double m_cur,double x_goal,double y_goal,double m_goal);
    double cellToCellCost(double distance_sqr,double m_cur,double m_goal);
    void exploreCell(PlanningMap& planning_map,int cur_id_x, int cur_id_y);

    void exploreCell();                         //id based newly added 

    double euclideanDistance(double first_x , double first_y, double second_x, double second_y);
    double euclideanDistance(double first_x , double first_y, double first_z, double second_x, double second_y, double second_z);
    double calTurnAngle(double first_x , double first_y, double first_z, double second_x, double second_y, double second_z);
    void removeIDFromExploringList(PlanningMap& planning_map,int id_x, int id_y);
    void findMinCostCellID(PlanningMap& planning_map);
    void findMinCostCellID(); //newly added 
    bool isInExplored(PlanningMap& planning_map);
    bool isObstacle(PlanningMap& planning_map);
    bool isOutMap(PlanningMap& planning_map);

    bool isOutMap();                            // newly added 

    bool isInExploring(PlanningMap& planning_map);

    void exploringTargetCell(PlanningMap& planning_map);
    void chooseNextTargetCell(PlanningMap& planning_map);
    
    // bool planForward(ID start, ID goal, std::vector<std::vector<int>> obstacle_array, double omega); 
    // bool planForward(ID start, ID goal, double omega, double beta, double gamma, double mu); // newly added 
    bool moveAndScan(int scan_range);                             // newly added  , return true while still be able to move, false if goal reached
    // 1 moving, 0 stuck, 2 goal reach, 3 updates in scan 
    std::vector<ID> scanMap(ID current_id, int range);
    

    ID last_node_;
    bool path_changed_{false};

    void traceBackPathID();                     // newly added   
    std::vector<ID> reversePath(std::vector<ID> original_path);

    void clearOldPath();
    void resetCellMapCost(); //newly added   
    void resetExploreVariables(); //newly added   
    bool matchID(ID id1, ID id2);      

    std::map<ID,Tile> convertToUniversalMap();
 
};
#endif