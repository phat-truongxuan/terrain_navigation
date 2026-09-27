#ifndef __NAV_NODE__
#define __NAV_NODE__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <fstream>
#include <sstream>
#include <iostream> 
#include <filesystem>
#include <vector>
#include <map>
#include "path_planner/map_handler.hpp"
#include "path_planner/default_planner.hpp"
#include "path_planner/dstar_lite_planner.hpp"
#include "path_planner/astar_planner.hpp"
#include "path_planner/utils.hpp"
#include "path_planner/msg/all_info.hpp"
#include <yaml-cpp/yaml.h> 


class NavNode
{
      
public:
    NavNode(rclcpp::Node::SharedPtr node) ; 
    ~NavNode();

    rclcpp::Subscription<path_planner::msg::AllInfo>::SharedPtr all_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

    void allInfoCallBack(const path_planner::msg::AllInfo::SharedPtr allinfo_data);
    void goalPoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr goalpose_data);
    void initialPoseCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose_data);

    std::map<std::string, DefaultPlanner> planner;
    DStarLitePlanner dstarlite_planner;
    AStarPlanner astar_planner;

    ID start_id_;
    ID goal_id_;
    ID current_id_;

    bool goal_reached_{false};

    bool received_new_msgs_{false};

    double cell_size_;
    double omega_{0};
    double beta_{0};
    double gamma_{0};
    double mu_{0};
    MapHandler map_hd_;
    std::map<ID,Tile> universal_map_;
    std::vector<ID> path_traced_;
    std::map<ID,Tile> base_planning_map_;
    double height_diff_threshold_{0.05};
    bool follow_path_{false};


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_ ;
	rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    geometry_msgs::msg::Pose viz_pose{geometry_msgs::msg::Pose()};	 
    geometry_msgs::msg::Vector3 size_vec; 
    Eigen::Vector3d vec_viz_;
    std_msgs::msg::ColorRGBA color_viz_;
    visualization_msgs::msg::Marker marker_;
    visualization_msgs::msg::Marker marker_list_;
    
    //resolution, max_z, cell size, numcell x, numcell y, upper range roughness, lower range roughness
    std::vector<double> transfer_params_{0,0,0,0,0,0,0};

    void setStartGoalID(ID start, ID goal);
    void setStartID(ID start);
    void setGoalID(ID goal);
    void setParam(double omega, double beta, double gamma, double mu);

    void importMap(MapHandler& map_hd);
    void importMaskedMap(MapHandler& map_hd);   //for exploration 
    void maskMap(std::vector<std::vector<double>> map_mask);
    void addObstacles(std::vector<std::vector<int>> obstacle_array, std::vector<std::vector<int>> hidden_obstacle_array);
    bool matchID(ID id1, ID id2);
    std::vector<ID> changed_id_;
    std::vector<ID> global_changed_id_;
    std::vector<ID> scanMap(ID current_id, int scan_range);
    void move(std::vector<ID>& path);       
    bool moveAndScan(int scan_range);       

    void initMarker();
    void getMapParams(std::vector<double> transfer_param);
    void pubMapRviz(std::map<ID,Tile> universal_map);
    void pubMap(std::map<ID,Tile> universal_map);
    void pubStartGoal(ID start, ID goal);
    void pubStart(ID start);
    void markPath(ID id);
    void pubGoal(ID goal);
    void pubPath(std::vector<ID> path_traced, bool moving);
    void pubUpdatedMap(std::vector<ID> changed_id);
    void pubHiddenObstacle(std::vector<std::vector<int>> hid_obs);
    void cleanPath(std::vector<ID> path_traced);
    void cleanPath();

    // int marker_start_id_ = 1100000;
    int marker_start_id_ = 9000000;
    // int marker_goal_id_ = 1100001;
    int marker_goal_id_ = 8000000;
    int path_domain_id_ = 1000000;
    int hidden_obstacle_id_ = 2000000;
    int scanned_cells_id_ = 3000000;

    rclcpp::Time start_planning_t_;
    rclcpp::Time finish_planning_t_;

    enum Step{
        INIT_NODE,
        INIT_PARAM,
        INIT_MAP,
        RECEIVE_START_GOAL,
        VISUALIZE_OBSTACLE,
        PLAN_FORWARD,
        

    };
    

private:
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
protected:
    rclcpp::Node::SharedPtr node_;
};
#endif