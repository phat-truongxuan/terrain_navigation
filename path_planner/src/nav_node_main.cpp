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
    double omega = config["omega"].as<double>(); 
    double gamma = config["gamma"].as<double>(); 
    double beta = config["beta"].as<double>(); 
    double mu = config["mu"].as<double>(); 
    double scan_range = config["scan_range"].as<double>(); 
    auto obstacles = config["obstacles"]; 
    std::string planner_name = "astar";
    std::vector<std::vector<int>> obs ;
    // for(int i=0; i< (int)(obstacles.size());i++){
    //     obs.push_back({obstacles[i][0].as<int>(),obstacles[i][1].as<int>()});
    // }
    rclcpp::init(argc, argv); 

    auto node = std::make_shared<rclcpp::Node>("nav_node_main");
    auto nav_node = std::make_shared<NavNode>(node);
    rclcpp::Rate loop_rate(10);

    // std::vector<std::vector<int>> hid_obs ;

    std::string filename = "/src/path_planner/pcl_map/terrain4.xyz";
    // std::string filename_masked = "/src/path_planner/pcl_map/terrain3_mask.xyz";
    std::string file_dir = project_dir + filename;
    // std::string file_dir_masked = project_dir + filename_masked;

    nav_node->map_hd_.setParam(cell_size, point_per_cell, size_adjust);
    nav_node->map_hd_.getMap(file_dir);
    nav_node->importMap(nav_node->map_hd_);
    // nav_node->addObstacles(obs,hid_obs);

    // nav_node->maskMap(map_mask);

    nav_node->setParam(omega, beta, gamma, mu);
    nav_node->planner[planner_name].transferMap(nav_node->base_planning_map_);
    // nav_node->astar_planner.transferMap(nav_node->base_planning_map_);
    nav_node->getMapParams(nav_node->map_hd_.transfer_params_);

    nav_node->pubMapRviz(nav_node->base_planning_map_);
    // nav_node->pubMap(nav_node->base_planning_map_);
    // nav_node->pubHiddenObstacle(hid_obs); 

    while(rclcpp::ok()) { 
        rclcpp::spin_some(node);
        if(nav_node->received_new_msgs_ == true){
            nav_node->goal_reached_ = false;
            nav_node->cleanPath();
            // nav_node->pubStartGoal(nav_node->start_id_, nav_node->goal_id_);
            nav_node->received_new_msgs_ = false;

            // nav_node->pubHiddenObstacle(hid_obs);
            // nav_node->pubUpdatedMap(nav_node->global_changed_id_);

            rclcpp::spin_some(node);
            std::cout << "   " << std::endl;

            if(nav_node->planner[planner_name].planForward(nav_node->start_id_, nav_node->goal_id_, nav_node->omega_, nav_node->beta_, nav_node->gamma_, nav_node->mu_)){
                nav_node->pubPath(nav_node->planner[planner_name].path_traced_id_, false);
                std::cout << " start  " << nav_node->start_id_.x *  nav_node->map_hd_.transfer_params_[2] <<
                " " << nav_node->start_id_.y *  nav_node->map_hd_.transfer_params_[2] <<  std::endl;
                std::cout << " goal  " << nav_node->goal_id_.x *  nav_node->map_hd_.transfer_params_[2] <<
                " " << nav_node->goal_id_.y *  nav_node->map_hd_.transfer_params_[2] <<  std::endl;

                std::cout << " found initial path " << std::endl;
                rclcpp::spin_some(node);
                usleep(2000000);
                if(nav_node->follow_path_ == true){
                    while(nav_node->goal_reached_ == false) {
                        if(nav_node->received_new_msgs_ == true){
                            nav_node->goal_reached_ = false;
                            break;
                        }
                        // nav_node->pubStart(nav_node->start_id_);
                        nav_node->changed_id_.clear();
                        nav_node->changed_id_ = nav_node->scanMap(nav_node->start_id_,scan_range);
                        if((int)(nav_node->changed_id_.size()) > 0){ //magic number 
                            std::cout << "detect changes " << std::endl;
                            // nav_node->pubUpdatedMap(nav_node->changed_id_);
                            rclcpp::spin_some(node);
                            nav_node->planner[planner_name].updateMap(nav_node->changed_id_);
                            nav_node->pubMapRviz(nav_node->base_planning_map_);
                            // if((int)(nav_node->changed_id_.size()) > 0){ 

                                nav_node->planner[planner_name].planForward(nav_node->start_id_, nav_node->goal_id_, nav_node->omega_, nav_node->beta_, nav_node->gamma_, nav_node->mu_);
                                nav_node->pubPath(nav_node->planner[planner_name].path_traced_id_, false);
                            // }

                        }
                        nav_node->move(nav_node->planner[planner_name].path_traced_id_);
                        nav_node->pubStart(nav_node->start_id_);
                        rclcpp::spin_some(node);
                        loop_rate.sleep();
                    }
                }
                
                
            }
        }
    }

}