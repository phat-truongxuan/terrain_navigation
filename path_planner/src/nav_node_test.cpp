#include "path_planner/map_handler.hpp"
#include "path_planner/dstar_lite_planner.hpp"
#include "path_planner/astar_planner.hpp"
#include "path_planner/navigation_node.hpp"


int main(int argc, char** argv)
{
    std::string project_dir = get_current_dir_name(); 

    std::ifstream fin(project_dir + "/src/path_planner/config/config.yaml"); 
    YAML::Node config = YAML::Load(fin);

    double cell_size = config["cell_size"].as<double>(); 
    double point_per_cell = config["point_per_cell"].as<double>(); 
    double size_adjust = config["size_adjust"].as<double>(); 
    double max_slope_threshold = config["max_slope_threshold"].as<double>(); 
    double gamma = config["gamma"].as<double>(); 
    double beta = config["beta"].as<double>(); 
    auto obstacles = config["obstacles"]; 
    std::vector<std::vector<int>> obs ;
    // for(int i=0; i< (int)(obstacles.size());i++){
    //     obs.push_back({obstacles[i][0].as<int>(),obstacles[i][1].as<int>()});
    // }
    rclcpp::init(argc, argv); 

    auto node = std::make_shared<rclcpp::Node>("nav_node_test");
    auto nav_node = std::make_shared<NavNode>(node);
    rclcpp::Rate loop_rate(10);

    int scan_range = 5;

    // std::vector<std::vector<int>> hid_obs ;

    std::string filename = "/src/path_planner/pcl_map/terrain4.xyz";
    // std::string filename_masked = "/src/path_planner/pcl_map/terrain3_mask.xyz";
    std::string file_dir = project_dir + filename;
    // std::string file_dir_masked = project_dir + filename_masked;

    nav_node->map_hd_.setParam(cell_size, point_per_cell, size_adjust);
    nav_node->map_hd_.getMap(file_dir);
    nav_node->importMap(nav_node->map_hd_);

    nav_node->dstarlite_planner.transferMap(nav_node->base_planning_map_);
    nav_node->getMapParams(nav_node->map_hd_.transfer_params_);

    nav_node->pubMapRviz(nav_node->base_planning_map_);
    // nav_node->pubHiddenObstacle(hid_obs); 

    while(rclcpp::ok()) { 
        rclcpp::spin_some(node);
        if(nav_node->received_new_msgs_ == true){
            nav_node->goal_reached_ = false;
            nav_node->cleanPath();
            nav_node->received_new_msgs_ = false;
            // nav_node->pubHiddenObstacle(hid_obs);

            rclcpp::spin_some(node);
            if(nav_node->dstarlite_planner.planForward(nav_node->start_id_, nav_node->goal_id_, nav_node->omega_, nav_node->beta_, nav_node->gamma_, nav_node->mu_)){
                nav_node->pubPath(nav_node->dstarlite_planner.path_traced_id_, false);
                std::cout << " params " <<  
                 " omega " << nav_node->omega_ << " beta " << nav_node->beta_ <<
                 " gamma " << nav_node->gamma_ << " mu " << nav_node->mu_
                << std::endl;

    //             std::cout << " found initial path " << std::endl;
                rclcpp::spin_some(node);
    //             usleep(25000000);

                while(nav_node->goal_reached_ == false) {
                    if(nav_node->received_new_msgs_ == true){
                        nav_node->goal_reached_ = false;
                        break;
                    }
    //                 // nav_node->pubStart(nav_node->start_id_);
    //                 nav_node->changed_id_.clear();
    //                 nav_node->changed_id_ = nav_node->scanMap(nav_node->start_id_,scan_range);
    //                 if((int)(nav_node->changed_id_.size()) > 0){ //magic number 
    //                     std::cout << "detect changes " << std::endl;
    //                     // nav_node->pubUpdatedMap(nav_node->changed_id_);
    //                     rclcpp::spin_some(node);
    //                     nav_node->astar_planner.updateMap(nav_node->changed_id_);
    //                     nav_node->pubMapRviz(nav_node->base_planning_map_);
    //                     // if((int)(nav_node->changed_id_.size()) > 0){ 

    //                         nav_node->astar_planner.planForward(nav_node->start_id_, nav_node->goal_id_, nav_node->omega_, nav_node->beta_, nav_node->gamma_, nav_node->mu_);
    //                         nav_node->pubPath(nav_node->astar_planner.path_traced_id_, false);
    //                     // }

    //                 }
    //                 nav_node->move(nav_node->astar_planner.path_traced_id_);
    //                 nav_node->pubStart(nav_node->start_id_);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                }
                
            }
        }
    }

}