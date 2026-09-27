// #include <rclcpp/rclcpp.hpp>
#include "path_planner/navigation_node.hpp"
using std::placeholders::_1;

NavNode::NavNode(rclcpp::Node::SharedPtr node) : node_(node)
{
    all_sub_ = node->create_subscription<path_planner::msg::AllInfo>(
      "/all_in_one", rclcpp::SystemDefaultsQoS(), std::bind(&NavNode::allInfoCallBack, this, _1));

    goalpose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&NavNode::goalPoseCallBack, this, _1));

    initialpose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::SystemDefaultsQoS(), std::bind(&NavNode::initialPoseCallBack, this, _1));

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("visual_marker",10);
      
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "/rviz_visual_tools",node_));
	visual_tools_->loadMarkerPub();
    visual_tools_->deleteAllMarkers();
    cleanPath();
    planner.insert(std::pair<std::string, DefaultPlanner>("astar", astar_planner));
    planner.insert(std::pair<std::string, DefaultPlanner>("dstarlite", dstarlite_planner));

}

NavNode::~NavNode()
{
    //nothing to do
}

void NavNode::allInfoCallBack(const path_planner::msg::AllInfo::SharedPtr allinfo_data) {


    // found_path = false;
    // std::cout << " recieve new message " << std::endl;
    received_new_msgs_ = true;
    ID tmp_start;
    ID tmp_goal;
    omega_ = allinfo_data->omega;
    beta_ = allinfo_data->beta;
    gamma_ = allinfo_data->gamma;
    mu_ = allinfo_data->mu;
    tmp_start.x = allinfo_data->start_id_x;
    tmp_start.y = allinfo_data->start_id_y;
    tmp_goal.x = allinfo_data->goal_id_x;
    tmp_goal.y = allinfo_data->goal_id_y;
    follow_path_ = allinfo_data->follow_path;
    setStartGoalID(tmp_start, tmp_goal);
    
}

void NavNode::goalPoseCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr goalpose_data){
    // std::cout <<" recieve new goal " << "X " <<goalpose_data->pose.position.x << "Y " <<goalpose_data->pose.position.y << std::endl;;
    RCLCPP_INFO_STREAM(node_->get_logger(),"recieve new goal: " << "X " <<goalpose_data->pose.position.x << "Y " <<goalpose_data->pose.position.y );
    received_new_msgs_ = true;
    ID tmp_goal;
    tmp_goal.x = int(std::ceil(goalpose_data->pose.position.x/transfer_params_[2])); // position in meter divided by cell size
    tmp_goal.y = int(std::ceil(goalpose_data->pose.position.y/transfer_params_[2])); 
    setGoalID(tmp_goal);

}

void NavNode::initialPoseCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose_data){

    RCLCPP_INFO_STREAM(node_->get_logger(),"initial pose set to : " << "X " <<initialpose_data->pose.pose.position.x << "Y " <<initialpose_data->pose.pose.position.y );
    ID tmp_start;
    tmp_start.x = int(std::ceil(initialpose_data->pose.pose.position.x/transfer_params_[2])); // position in meter divided by cell size
    tmp_start.y = int(std::ceil(initialpose_data->pose.pose.position.y/transfer_params_[2])); 
    setStartID(tmp_start);

}

void NavNode::setStartGoalID(ID start, ID goal){
    start_id_ = start;
    goal_id_ = goal;
}

void NavNode::setStartID(ID start){
    start_id_ = start;
}

void NavNode::setGoalID(ID goal){
    goal_id_ = goal;
}

void NavNode::setParam(double omega, double beta, double gamma, double mu){
    omega_ = omega;
    beta_  = beta;
    gamma_ = gamma;
    mu_    = mu;
}


void NavNode::importMap(MapHandler& map_hd){
    ID iter_id;
    Tile iter_cell;
    cell_size_ =  map_hd.grid_map.cell_size;
    for(int i = 0; i < map_hd.grid_map.num_cel_x; i++){
        for(int j = 0; j < map_hd.grid_map.num_cel_y; j++){
            iter_cell.tile_size_ = cell_size_;
            if(map_hd.grid_map.grid_matrix[i][j].getOverThree()){
                iter_cell.height_ = map_hd.grid_map.grid_matrix[i][j].getMeanCenterCoordinate()[2];
                iter_cell.is_obstacle_ = false;
                iter_cell.roughness_ = map_hd.grid_map.grid_matrix[i][j].getRMS() ; 
                // if(i == 20 && j == 20){
                //     std::cout << " rough ness id 20 20 " << iter_cell.roughness_;
                // }
            }
            else if(map_hd.grid_map.grid_matrix[i][j].getOverThree() == false){
                iter_cell.height_ =  1e10;
                iter_cell.is_obstacle_ = true;
            }
            if( i==map_hd.grid_map.num_cel_x-1 && j==map_hd.grid_map.num_cel_y-1){
                iter_cell.max_id_ = true;
                // std::cout << " set max id " << std::endl;
            }
            iter_cell.masked_height_ = iter_cell.height_;
            iter_id.setID(i, j);
            base_planning_map_.insert(std::pair<ID,Tile>(iter_id, iter_cell)); 
        }
    }       
}

void NavNode::importMaskedMap(MapHandler& map_hd){
    ID iter_id;
    double height;
    // Tile iter_cell;
    cell_size_ =  map_hd.grid_map.cell_size;
    for(int i = 0; i < map_hd.grid_map.num_cel_x; i++){
        for(int j = 0; j < map_hd.grid_map.num_cel_y; j++){
            // iter_cell.tile_size_ = cell_size_;
            if(map_hd.grid_map.grid_matrix[i][j].getOverThree()){
                height = map_hd.grid_map.grid_matrix[i][j].getMeanCenterCoordinate()[2];
            //     iter_cell.is_obstacle_ = false;
            }
            else if(map_hd.grid_map.grid_matrix[i][j].getOverThree() == false){
                height =  1e10;
            //     iter_cell.is_obstacle_ = true;
            }
            // if( i==map_hd.grid_map.num_cel_x-1 && j==map_hd.grid_map.num_cel_y-1){
            //     iter_cell.max_id_ = true;
            //     // std::cout << " set max id " << std::endl;
            // }
            // iter_cell.masked_height_ = iter_cell.height_;
            iter_id.setID(i, j);
            base_planning_map_[iter_id].masked_height_ = height; 
        }
    }       
}

void NavNode::maskMap(std::vector<std::vector<double>> map_mask){
    ID iter_id;
    for(int s=0; s < (int)(map_mask.size()); s++){
        for(int i=(int)(map_mask[s][0]); i<=(int)(map_mask[s][2]); i++ ){
            for(int j=(int)(map_mask[s][1]); j<=(int)(map_mask[s][3]); j++ ){
                iter_id.setID(i,j);
                // base_planning_map_[iter_id].masked_height_*= map_mask[s][4];
                base_planning_map_[iter_id].masked_height_= map_mask[s][4];
                global_changed_id_.push_back(iter_id);
            }
        }
    }
}


void NavNode::addObstacles(std::vector<std::vector<int>> obstacle_array, std::vector<std::vector<int>> hidden_obstacle_array){
    ID iter_id;
    // static_obstacle_ = obstacle_array;
    for(int i=0; i< (int)(obstacle_array.size()); i++ ){
        iter_id.setID(obstacle_array[i][0], obstacle_array[i][1]);
        base_planning_map_[iter_id].is_obstacle_ = true;
    } 
    for(int i=0; i< (int)(hidden_obstacle_array.size()); i++ ){
        iter_id.setID(hidden_obstacle_array[i][0], hidden_obstacle_array[i][1]);
        base_planning_map_[iter_id].hidden_obstacle_ = true;
    } 
}

bool NavNode::matchID(ID id1, ID id2){
    if(id1.x == id2.x && id1.y == id2.y){
        return true;
    }
    else{
        return false;
    }
}


std::vector<ID> NavNode::scanMap(ID current_id, int scan_range){
    std::vector<ID> changed_ids;
    ID iter_id;
    for(int i = -1*scan_range; i <= scan_range; i++){
        for(int j = -1*scan_range; j <= scan_range; j++){
            // if( current_id.x + i >=0 && current_id.y + j >=0 && (i*j + i+j !=0)){
            if( current_id.x + i >=0 && current_id.y + j >=0 ){
                iter_id.x = current_id.x + i;
                iter_id.y = current_id.y + j;
                // std::cout << " abs height diff " <<  std::abs(base_planning_map_[iter_id].masked_height_ - base_planning_map_[iter_id].height_)  << std::endl;
                if(base_planning_map_[iter_id].hidden_obstacle_ == true && base_planning_map_[iter_id].is_obstacle_ == false){
                    std::cout << " detect hidden obstacle " << std::endl;
                    base_planning_map_[iter_id].is_obstacle_ =true;
                    changed_ids.push_back(iter_id);
                    // continue;
                }
                if( std::abs(base_planning_map_[iter_id].masked_height_ - base_planning_map_[iter_id].height_) > height_diff_threshold_ ){
                    // std::cout << "height changed " << std::endl;
                    base_planning_map_[iter_id].height_ = base_planning_map_[iter_id].masked_height_;
                    changed_ids.push_back(iter_id);
                    // continue;
                }
            }
            else{
                continue;
            }
        }
    }
    return changed_ids;
}

void NavNode::move(std::vector<ID>& path){
    path.erase(path.begin());
    path.shrink_to_fit();
    start_id_ = path[0];
    if(matchID(start_id_, goal_id_)){
        goal_reached_ = true;
        std::cout << "goal_reached_ " << std::endl;

    }
}     


bool NavNode::moveAndScan(int scan_range){
    
}

void NavNode::initMarker(){
    marker_.header.frame_id = "/map";

    marker_.type = visualization_msgs::msg::Marker::CUBE;
    marker_.action = visualization_msgs::msg::Marker::ADD;

    // marker_.pose.position.x = 0;
    // marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    marker_.scale.x = cell_size_;
    marker_.scale.y = cell_size_;
    marker_.scale.z = cell_size_;

    marker_.color.r = 1.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0;
}

void NavNode::getMapParams(std::vector<double> transfer_param){
    //resolution, max_z, cell size, numcell x, numcell y, upper range roughness, lower range roughness
    transfer_params_ = transfer_param;
}



void NavNode::pubMap(std::map<ID,Tile> universal_map){
    universal_map_ = universal_map;
    
    initMarker();
    geometry_msgs::msg::Point temp;
    marker_list_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_list_.action = visualization_msgs::msg::Marker::ADD;

    for (auto iter : universal_map)   {
        cell_size_ = iter.second.tile_size_;
        marker_list_.scale.x = cell_size_ - 0.01;
        marker_list_.scale.y = cell_size_ - 0.01;
        temp.x = iter.first.x * cell_size_ + cell_size_/2;
        temp.y = iter.first.y * cell_size_ + cell_size_/2; 
        temp.z = iter.second.height_;

        // marker_.scale.x = cell_size_ - 0.01;
        // marker_.scale.y = cell_size_ - 0.01;
        // marker_.pose.position.x = iter.first.x * cell_size_ + cell_size_/2;
        // marker_.pose.position.y = iter.first.y * cell_size_ + cell_size_/2; 
        // marker_.pose.position.z = iter.second.height_;

        // if(iter.second.is_obstacle_ == true){
        //     marker_.color.r = 0.4f;
        //     marker_.color.g = 0.4f;
        //     marker_.color.b = 0.4f;
        //     marker_.color.a = 1.0;
        // }

        // marker_.id =  iter.first.x*1000 + iter.first.y;   /// limit number of blocks to 999x999

        // geometry_msgs::msg::Point temp;
        // temp.x = 0.5;
        // temp.y = 0.5;
        // temp.z = 0.5;
        marker_list_.points.push_back(temp);
        // marker_pub_->publish(marker_);

        marker_.color.r = 1.0f;
        marker_.color.g = 1.0f;
        marker_.color.b = 1.0f;
        marker_.color.a = 1.0;
        rclcpp::spin_some(node_);
        // usleep(5000);
    }
    marker_pub_->publish(marker_list_);
    rclcpp::spin_some(node_);
    
}


void NavNode::pubMapRviz(std::map<ID,Tile> universal_map){
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "/rviz_visual_tools",node_));
	visual_tools_->loadMarkerPub();
    visual_tools_->deleteAllMarkers();
    
    universal_map_ = universal_map;
    size_vec.z = 0.2;
    viz_pose.orientation.x =  0;
    viz_pose.orientation.y =  0;
    viz_pose.orientation.z =  0;
    viz_pose.orientation.w =  1;
    double gap = 0.01;
    Eigen::Vector3d vec_viz;
    
    
    for (auto iter : universal_map){  // iter first = ID, iter second = Tile
            // cell_size_ = iter.second.tile_size_; //test fixbug

            if(iter.second.is_obstacle_ == false){
                // size_vec.z = 0.2;
                // color_viz_.r = iter.second.height_/ 3.8822;   // height graph 
                // color_viz_.g = 0;
                // color_viz_.b = 1 - iter.second.height_/ 3.8822;  // height on max z scale 
                // color_viz_.a = 0.8; // translucent 
                
                color_viz_.r =  iter.second.height_/ transfer_params_[1];   
                color_viz_.g =  iter.second.height_/ transfer_params_[1];
                color_viz_.b =  iter.second.height_/ transfer_params_[1] ;  
                // color_viz_.r =  iter.second.roughness_/  transfer_params_[5] ;   
                // color_viz_.g =  iter.second.roughness_/  transfer_params_[5] ;
                // color_viz_.b =  iter.second.roughness_/  transfer_params_[5] ;
                
                color_viz_.a = 1; // translucent
                viz_pose.position.x =  iter.first.x * cell_size_ + cell_size_/2;
                viz_pose.position.y =  iter.first.y * cell_size_ + cell_size_/2;
                viz_pose.position.z =  iter.second.height_;
                visual_tools_->publishCuboid(viz_pose,cell_size_ - gap, cell_size_ - gap, cell_size_ - gap, color_viz_);
                // color_viz_.r = 0;
                // color_viz_.g = 0;
                // color_viz_.b = 0.8;   // translucent blue
                // viz_pose.position.z =  iter.second.masked_height_ ;
                // visual_tools_->publishCuboid(viz_pose,cell_size_ - gap, cell_size_ - gap, cell_size_ - gap, color_viz_);
                

            }
            else if(iter.second.is_obstacle_ == true){
                
                // size_vec.z = 0.2;
                color_viz_.r = 1;
                color_viz_.g = 0.1;
                color_viz_.b = 0.1;
                color_viz_.a = 0.8; // translucent red
                viz_pose.position.x =  iter.first.x * cell_size_ + cell_size_/2;
                viz_pose.position.y =  iter.first.y * cell_size_ + cell_size_/2;
                viz_pose.position.z =  iter.second.height_+0.1;
                visual_tools_->publishCuboid(viz_pose,cell_size_ - gap, cell_size_ - gap, cell_size_ - gap, color_viz_);
            }
    }
    // std::cout << " pub map rviz cell size " <<  cell_size_ << std::endl;

    visual_tools_->trigger();
}

void NavNode::pubStartGoal(ID start, ID goal){
    
    initMarker();
    current_id_= start_id_;
    // std::cout << " pub start goal cell size " << cell_size_<< std::endl;
    marker_.pose.position.z = 0.1;
    marker_.scale.x = cell_size_ - 0.01;
    marker_.scale.y = cell_size_ - 0.01;
    marker_.color.r = 0.0f;
    marker_.color.g = 0.0f;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0;
    
    marker_.pose.position.x = start.x * cell_size_ + cell_size_/2;
    marker_.pose.position.y = start.y * cell_size_ + cell_size_/2; 
    // marker_.pose.position.z =     universal_map_[start].height_ + 0.1; 
    marker_.pose.position.z =     base_planning_map_[start].height_ + 0.1; 
    marker_.id =  marker_start_id_; //specific ID for start
    marker_pub_->publish(marker_);
  
    marker_.color.r = 1.0f;
    marker_.color.g = 0.1f;
    marker_.color.b = 0.1f;
    marker_.pose.position.x = goal.x * cell_size_ + cell_size_/2;
    marker_.pose.position.y = goal.y * cell_size_ + cell_size_/2; 
    marker_.pose.position.z =     universal_map_[goal].height_ + 0.1; 
    marker_.id =  marker_goal_id_; //specific ID for goal
    marker_pub_->publish(marker_);
    rclcpp::spin_some(node_);
    
}

void NavNode::pubStart(ID start){
    // initMarker();
    current_id_= start_id_;
    marker_.pose.position.z = 0.1;
    marker_.scale.x = cell_size_ - 0.01;
    marker_.scale.y = cell_size_ - 0.01;
    marker_.scale.x = cell_size_ *1.5;
    marker_.scale.y = cell_size_ *1.5;
    marker_.scale.z = cell_size_ - 0.01;
    marker_.color.r = 0.0f;
    marker_.color.g = 0.0f;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0;
    
    marker_.pose.position.x = start.x * cell_size_ + cell_size_/2;
    marker_.pose.position.y = start.y * cell_size_ + cell_size_/2; 
    // marker_.pose.position.z =     universal_map_[start].height_ + 0.1; 
    marker_.pose.position.z =     base_planning_map_[start].height_ + 0.1; 
    marker_.id =  marker_start_id_; 
    marker_pub_->publish(marker_);
    markPath(start);
    
}

void NavNode::markPath(ID id){
    initMarker();
    marker_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_.pose.position.z = 0.1;
    marker_.scale.x = cell_size_/2;
    marker_.scale.y = cell_size_/2;
    marker_.scale.z = cell_size_/6;
    marker_.color.r = 0.0f;
    marker_.color.g = 0.5f;
    marker_.color.b = 0.8f;
    marker_.color.a = 0.8;
    
    marker_.pose.position.x = id.x * cell_size_ + cell_size_/2;
    marker_.pose.position.y = id.y * cell_size_ + cell_size_/2; 
    // marker_.pose.position.z =     universal_map_[id].height_ + cell_size_/2 ; 
    marker_.pose.position.z =     base_planning_map_[id].height_ + cell_size_/2 ; 
    marker_.id =   id.x*1000 + id.y; 
    marker_pub_->publish(marker_);
    
}

void NavNode::pubGoal(ID goal){
    initMarker();
    marker_.color.r = 1.0f;
    marker_.color.g = 0.1f;
    marker_.color.b = 0.1f;
    marker_.scale.x = cell_size_ *1.5;
    marker_.scale.y = cell_size_ *1.5;
    marker_.pose.position.x = goal.x * cell_size_ + cell_size_/2;
    marker_.pose.position.y = goal.y * cell_size_ + cell_size_/2; 
    // marker_.pose.position.z =     universal_map_[goal].height_ + 0.1; 
    marker_.pose.position.z =     base_planning_map_[goal].height_ + 0.1; 
    marker_.id =  marker_goal_id_; //specific ID for goal
    marker_pub_->publish(marker_);
    rclcpp::spin_some(node_);
}

void NavNode::pubPath(std::vector<ID> path_traced, bool moving){

    initMarker();
    if(moving == true){
        // cleanPath();
        marker_.action = visualization_msgs::msg::Marker::DELETE;
        // marker_.pose.position.z = 0;
        marker_.id =   current_id_.x*1000 + current_id_.y; 
        marker_pub_->publish(marker_);
        marker_.action = visualization_msgs::msg::Marker::MODIFY;
    }
    marker_.scale.x = cell_size_/2;
    marker_.scale.y = cell_size_/2;
    marker_.scale.z = cell_size_/6;
    // std::cout << " cell_size_ " << cell_size_ << std::endl;

    if(cell_size_ == 0){
        std::cout << " error " << std::endl;
    }
    start_id_ = path_traced[0];
    pubStart(start_id_);
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.2f;
    marker_.color.a = 1.0;
    for(int i=1;i<(int)(path_traced.size())-1; i++){
        marker_.pose.position.x = path_traced[i].x * cell_size_ + cell_size_/2;
        marker_.pose.position.y = path_traced[i].y * cell_size_ + cell_size_/2; 
        // marker_.pose.position.z = universal_map_[path_traced[i]].height_ + cell_size_/2; 
        marker_.pose.position.z = base_planning_map_[path_traced[i]].height_ + cell_size_/2; 
        marker_.id =   path_traced[i].x*1000 + path_traced[i].y; 
        marker_pub_->publish(marker_);
        // rclcpp::spin_some(node_);
        // usleep(5000);
    }
    pubGoal(path_traced.back());
}

void NavNode::pubUpdatedMap(std::vector<ID> changed_id){
    initMarker();
    
    marker_.scale.x = cell_size_;
    marker_.scale.y = cell_size_;
    marker_.scale.z = cell_size_;
    marker_.color.r = 1.0f;
    marker_.color.g = 0.2f;
    marker_.color.b = 1.0f;
    marker_.color.a = 0.8f;
    for(int i=0;i<(int)(changed_id.size()); i++){
        marker_.pose.position.x = changed_id[i].x * cell_size_ + cell_size_/2;
        marker_.pose.position.y = changed_id[i].y * cell_size_ + cell_size_/2; 
        // marker_.pose.position.z = base_planning_map_[changed_id[i]].height_ ; 
        marker_.pose.position.z = base_planning_map_[changed_id[i]].masked_height_ ; 
        marker_.id =   changed_id[i].x*1000 + changed_id[i].y + scanned_cells_id_; 
        marker_pub_->publish(marker_);
        // rclcpp::spin_some(node_);
        // usleep(5000);
    }

}


void NavNode::pubHiddenObstacle(std::vector<std::vector<int>> hid_obs){
    initMarker();
    ID iter_id;
    marker_.scale.x = cell_size_ - 0.01;
    marker_.scale.y = cell_size_ - 0.01;
    marker_.scale.z = cell_size_;
    marker_.color.r = 1.0f;
    marker_.color.g = 0.45f;
    marker_.color.b = 0.0f;
    marker_.color.a = 0.8;
    for(int i=0;i<(int)(hid_obs.size()); i++){
        marker_.pose.position.x = hid_obs[i][0] * cell_size_ + cell_size_/2;
        marker_.pose.position.y = hid_obs[i][1] * cell_size_ + cell_size_/2; 
        iter_id.x = hid_obs[i][0] ;
        iter_id.y = hid_obs[i][1] ;
        marker_.pose.position.z = universal_map_[iter_id].height_ + 0.13; 
        marker_.id =   hid_obs[i][0]*1000 + hid_obs[i][1] + hidden_obstacle_id_; 
        marker_pub_->publish(marker_); 
    }
}

void NavNode::cleanPath(std::vector<ID> path_traced){
    // cell_size_ = 0.6;   // hardcode
    initMarker();
    marker_.scale.x = cell_size_ - 0.01;
    marker_.scale.y = cell_size_ - 0.01;

    for(int i=0;i<(int)(path_traced.size()); i++){
        marker_.pose.position.x = path_traced[i].x * cell_size_ + cell_size_/2;
        marker_.pose.position.y = path_traced[i].y * cell_size_ + cell_size_/2; 
        marker_.id =   path_traced[i].x*1000 + path_traced[i].y; //test change id counting 
        marker_pub_->publish(marker_);
        rclcpp::spin_some(node_);
        usleep(10000);
    }
}
void NavNode::cleanPath(){
    
    initMarker();
    marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(marker_);
    rclcpp::spin_some(node_);
    initMarker();

}