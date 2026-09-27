
#include "path_planner/default_planner.hpp"
using std::placeholders::_1;

DefaultPlanner::DefaultPlanner()
{
    max_slope_threshold_ = 1;
    gamma_weight_ = 1;
    beta_weight_ = 1;
    
}

DefaultPlanner::~DefaultPlanner()
{
}

void DefaultPlanner::setParam(double max_slope_threshold, double gamma, double beta){
    max_slope_threshold_ = max_slope_threshold;
    gamma_weight_ = gamma;
    beta_weight_ = beta;
}

void DefaultPlanner::addObstacle(std::vector<std::vector<int>> obstacle_array){
    ID iter_id;
    static_obstacle_ = obstacle_array;
    for(int i=0; i< (int)(obstacle_array.size()); i++ ){
        
        iter_id.x = obstacle_array[i][0];
        iter_id.y = obstacle_array[i][1];
        
        if(matchID(iter_id, start_node_) == true || matchID(iter_id, goal_node_) == true){
            continue;
        }
        planning_map_constructing_[iter_id].is_obstacle_ = true;
        // std::cout << " id " << iter_id.x << "-" << iter_id.y << " obstacle " << std::endl;
    } 
}

void DefaultPlanner::addHiddenObstacle(std::vector<std::vector<int>> hidden_obstacle_array){
    ID iter_id;
    for(int i=0; i< (int)(hidden_obstacle_array.size()); i++ ){
        
        iter_id.x = hidden_obstacle_array[i][0];
        iter_id.y = hidden_obstacle_array[i][1];
        
        if(matchID(iter_id, start_node_) == true || matchID(iter_id, goal_node_) == true){
            continue;
        }
        planning_map_constructing_[iter_id].hidden_obstacle_ = true;

    } 
}

void DefaultPlanner::addMaskedMap(std::vector<std::vector<int>> masked_map_array){
    ID iter_id;
    for(int i=0; i< (int)(masked_map_array.size()); i++ ){
        
        iter_id.x = masked_map_array[i][0];
        iter_id.y = masked_map_array[i][1];
        
        if(matchID(iter_id, start_node_) == true || matchID(iter_id, goal_node_) == true){
            continue;
        }
        planning_map_constructing_[iter_id].masked_height_ = masked_map_array[i][2];
    } 
}


void DefaultPlanner::Cell::setCellNewCost(double new_m_cost,double new_g_cost,double new_h_cost,double new_f_cost,int new_parent_id_x, int new_parent_id_y, double slope_cost){

    if(f_cost_ > new_f_cost){ // not in used 
        g_cost_ = new_g_cost;
        h_cost_ = new_h_cost; 
        f_cost_ = new_f_cost;
        parent_id_x_ = new_parent_id_x;
        parent_id_y_ = new_parent_id_y;
        sl_cost_ = slope_cost;
        new_m_cost = new_m_cost;
    }
}

void DefaultPlanner::Cell::setCellNewCost(double new_m_cost,double new_g_cost,double new_h_cost,double new_f_cost,ID parent_id, double slope_cost, double distance_cost){

    if(f_cost_ > new_f_cost){
        g_cost_ = new_g_cost;
        h_cost_ = new_h_cost; 
        f_cost_ = new_f_cost;
        parent_id_ = parent_id;
        sl_cost_ = slope_cost;
        distance_cost_ = distance_cost;
        new_m_cost = new_m_cost;
    }
}


void DefaultPlanner::PlanningMap::resetExploreLists(){
    explored_cell_list.clear();
    explored_cell_list.shrink_to_fit();
    exploring_cell_list.clear();
    exploring_cell_list.shrink_to_fit();
    exploring_cell_list_tmp.clear();
    exploring_cell_list_tmp.shrink_to_fit();
}

void DefaultPlanner::PlanningMap::resetCellMapCost(){

    for(int i = 0; i < (int)(cell_map.size()) ; i++){
        for(int j = 0; j < (int)(cell_map[0].size()); j++){
            cell_map[i][j].f_cost_ = 1e10;
            cell_map[i][j].h_cost_ = 1e10;
            cell_map[i][j].g_cost_ = 1e10;
            cell_map[i][j].parent_id_x_ = 0;
            cell_map[i][j].parent_id_y_ = 0;
        }
    }
    

}

void DefaultPlanner::resetCellMapCost(){
    // Cell reset_cell;
    for (auto iter : planning_map_constructing_)   {
        // planning_map_constructing_[iter.first] = reset_cell
        planning_map_constructing_[iter.first].f_cost_ = 1e10;
        planning_map_constructing_[iter.first].h_cost_ = 1e10;
        planning_map_constructing_[iter.first].g_cost_ = 1e10;
        planning_map_constructing_[iter.first].sl_cost_ = 1e10;
        planning_map_constructing_[iter.first].parent_id_x_ = 0;
        planning_map_constructing_[iter.first].parent_id_y_ = 0;
        planning_map_constructing_[iter.first].parent_id_.setID(0,0);
    }
}

void DefaultPlanner::resetExploreVariables(){
    for (auto iter : planning_map_constructing_)   {
        planning_map_constructing_[iter.first].is_explored_ = false;
        planning_map_constructing_[iter.first].is_exploring_ = false;
        planning_map_constructing_[iter.first].is_start_ = false;
        planning_map_constructing_[iter.first].is_goal_ = false;

    }
}

bool DefaultPlanner::matchID(ID id1, ID id2){
    if(id1.x == id2.x && id1.y == id2.y){
        return true;
    }
    else{
        return false;
    }
}

void DefaultPlanner::importMap(MapHandler& map_hd){

    ID iter_id;
    Cell iter_cell;
    cell_size_ =  map_hd_.grid_map.cell_size;
    std::cout << " import map " << std::endl;
    for(int i = 0; i < map_hd.grid_map.num_cel_x; i++){
        for(int j = 0; j < map_hd.grid_map.num_cel_y; j++){
            iter_cell.id_x_ = i;
            iter_cell.id_y_ = j;
            iter_cell.cell_size_ = map_hd.cell_size_;
            if(map_hd.grid_map.grid_matrix[i][j].getOverThree()){
                iter_cell.m_cost_ = map_hd.grid_map.grid_matrix[i][j].getMeanCenterCoordinate()[2];
                iter_cell.height_ = iter_cell.m_cost_;
                iter_cell.is_obstacle_ = false;
                iter_cell.roughness_ = map_hd.grid_map.grid_matrix[i][j].getRMS() ; 
                // if(i == 20 && j == 20){
                //     std::cout << " rough ness id 20 20 " << iter_cell.roughness_;
                // }
            }
            else if(map_hd.grid_map.grid_matrix[i][j].getOverThree() == false){
                
                iter_cell.m_cost_ = 1e10;
                iter_cell.height_ = iter_cell.m_cost_;
                iter_cell.is_obstacle_= true;
                iter_cell.is_unknown_= true;
            }
            iter_id.setID(i, j);
            planning_map_constructing_.insert(std::pair<ID,Cell>(iter_id, iter_cell)); //constructing 
        }
    }       
}


void DefaultPlanner::importMap(){
    ID iter_id;
    Cell iter_cell;

    cell_size_ =  map_hd_.grid_map.cell_size;
    for(int i = 0; i < map_hd_.grid_map.num_cel_x; i++){
        for(int j = 0; j < map_hd_.grid_map.num_cel_y; j++){
            iter_cell.id_x_ = i;
            iter_cell.id_y_ = j;
            iter_cell.cell_size_ = cell_size_;
            if(map_hd_.grid_map.grid_matrix[i][j].getOverThree()){
                iter_cell.m_cost_ = map_hd_.grid_map.grid_matrix[i][j].getMeanCenterCoordinate()[2];
                iter_cell.height_ = iter_cell.m_cost_;
                iter_cell.is_obstacle_ = false;
                iter_cell.roughness_ = map_hd_.grid_map.grid_matrix[i][j].getRMS() ; 
                // if(i == 20 && j ==20){
                //     std::cout << " rough ness id 20 20 " << iter_cell.roughness_;
                // }
            }
            else if(map_hd_.grid_map.grid_matrix[i][j].getOverThree() == false){
                
                iter_cell.m_cost_ = 1e10;
                iter_cell.height_ = iter_cell.m_cost_ ;
                iter_cell.is_obstacle_ = true;
                iter_cell.is_unknown_= true;
            }
            iter_id.setID(i, j);
            planning_map_constructing_.insert(std::pair<ID,Cell>(iter_id, iter_cell)); //constructing 

        }

    }       
}

void DefaultPlanner::transferMap(std::map<ID,Tile> base_planning_map){
    Cell iter_cell;
    ID iter_id;
    for (auto iter : base_planning_map)   {
        // iter_id = iter.first;
        iter_id.x = iter.first.x;
        iter_id.y = iter.first.y;
        iter_cell.cell_size_ = iter.second.tile_size_;
        iter_cell.height_ = iter.second.height_;
        iter_cell.m_cost_ = iter.second.height_;
        iter_cell.masked_height_ = iter.second.masked_height_;
        iter_cell.tile_size_ = iter.second.tile_size_;
        iter_cell.is_obstacle_ = iter.second.is_obstacle_;
        iter_cell.hidden_obstacle_ = iter.second.hidden_obstacle_;
        iter_cell.roughness_ = iter.second.roughness_;
        if(iter.second.max_id_ == true){
            max_num_cel_x = iter_id.x;
            max_num_cel_y = iter_id.y;
        }
        planning_map_constructing_.insert(std::pair<ID,Cell>(iter_id, iter_cell)); //constructing 
    }
    cell_size_ = iter_cell.cell_size_;
}



void DefaultPlanner::setStartGoalID(ID start, ID goal){
    
    start_node_= start;
    current_node_ = start;
    goal_node_ = goal;
    planning_map_constructing_[start_node_].g_cost_ = 0;
    
    std::cout << " start x y z " << start.x<< " " << start.y
                << " " << planning_map_constructing_[start_node_].m_cost_
    << " goal x y z " << goal_node_.x << " " << goal_node_.y
                << " " << planning_map_constructing_[goal_node_].m_cost_ << std::endl;
}


bool DefaultPlanner::checkOKStartGoal(){
    if(start_id_x_<0||start_id_y_<0||finalgoal_id_x_<0||finalgoal_id_y_<0){
        return false;
    }
    return true;
}

// double DefaultPlanner::cellToCellCost(double x_cur,double y_cur,double m_cur,double x_goal,double y_goal,double m_goal){
//     return sqrt(pow(x_goal - x_cur,2) + pow(y_goal - y_cur,2) + pow(m_goal - m_cur,2));
// }

double DefaultPlanner::cellToCellCost(double distance_sqr,double m_cur,double m_goal){
    return sqrt(distance_sqr + pow(w_weight_heigh_*(m_goal - m_cur),2));
}

double DefaultPlanner::euclideanDistance(double first_x , double first_y, double second_x, double second_y){
    return sqrt(pow(first_x- second_x, 2) + pow(first_y - second_y,2));
}

double DefaultPlanner::euclideanDistance(double first_x , double first_y, double first_z, double second_x, double second_y, double second_z){
    return sqrt(pow(first_x- second_x, 2) + pow(first_y - second_y,2) + pow(first_z - second_z,2));
}

double DefaultPlanner::calTurnAngle(double first_x , double first_y, double first_z, double second_x, double second_y, double second_z){
    return acos((first_x*second_x + first_y*second_y + first_z*second_z)/
    (sqrt(first_x*first_x +first_y*first_y + first_z*first_z)*sqrt(second_x*second_x +second_y*second_y + second_z*second_z)));
    // return sqrt(pow(first_x- second_x, 2) + pow(first_y - second_y,2) + pow(first_z - second_z,2));
}

void DefaultPlanner::removeIDFromExploringList(PlanningMap& planning_map,int id_x, int id_y){
    planning_map.exploring_cell_list_tmp.clear();
    for(int i = 0; i < (int)(planning_map.exploring_cell_list.size()); i++){
        // std::cout << " exploring cell list " << planning_map.exploring_cell_list[i][0] << " " 
        // << planning_map.exploring_cell_list[i][1] << std::endl;

        if(planning_map.exploring_cell_list[i][0] != id_x || planning_map.exploring_cell_list[i][1] != id_y){
            // std::cout << " adding  " << planning_map.exploring_cell_list[i][0] << " " << planning_map.exploring_cell_list[i][1] << std::endl;
            planning_map.exploring_cell_list_tmp.push_back({planning_map.exploring_cell_list[i][0],planning_map.exploring_cell_list[i][1]});
   
        }
    }
    planning_map.exploring_cell_list.clear();
    planning_map.exploring_cell_list = planning_map.exploring_cell_list_tmp;
    ID iter_id;
    iter_id.setID(id_x, id_y);
    planning_map_constructing_[iter_id].is_exploring_ = false;
}

// void DefaultPlanner::exploringTargetCell(PlanningMap& planning_map){
//     exploreCell(planning_map,cur_id_x_,cur_id_y_);
// }

void DefaultPlanner::chooseNextTargetCell(PlanningMap& planning_map){
    // findMinCostCellID(planning_map);
    findMinCostCellID();
}


bool DefaultPlanner::planForward(ID start, ID goal, double omega, double beta, double gamma, double mu){
    found_path_ = false;
    clearOldPath();
    // std::cout << "plan forward " << " omega " << omega << std::endl;
    setStartGoalID(start,goal);
    // setStartGoalID(goal,start);
    // addObstacle(obstacle_array);
    w_weight_gcost_ = omega;
    w_weight_hcost_ = omega;
    omega_ = omega;
    max_slope_threshold_ = mu;
    gamma_weight_ = gamma;
    beta_weight_ = beta;
    if(matchID(start_node_, goal_node_) == true){
        path_traced_id_.push_back(goal);
        return true;
    }
    // int i;
    for(int i=0;i<max_compute_path_iter;i++){
        // std::cout << " iter " << i << std::endl; 
        if(found_path_ == true){

            traceBackPathID();
            break;
        }
        exploreCell();
        findMinCostCellID();
    }

    resetCellMapCost();
    resetExploreVariables();
    

    return found_path_;
}

std::vector<ID> DefaultPlanner::scanMap(ID current_id, int range){
    //range = 1 total surrounding = 3x3 -1
    //range = 2 total surrounding = 5x5 -1
    //range = 3 total surrounding = 7x7 -1 (rangex2+1 x rangex2+1 - 1)
    std::vector<ID> changed_ids;
    ID iter_id;
    for(int i = -1*range; i <= range; i++){
        for(int j = -1*range; j <= range; j++){
            if( current_id.x + i >=0 && current_id.y + j >=0 && (i*j + i+j !=0)){
                iter_id.x = current_id.x + i;
                iter_id.y = current_id.y + j;

                if(planning_map_constructing_[iter_id].hidden_obstacle_ == true && planning_map_constructing_[iter_id].is_obstacle_ == false){
                    planning_map_constructing_[iter_id].is_obstacle_ =true;
                    changed_ids.push_back(iter_id);
                }
            }
            else{
                continue;
            }
        }
    }
    return changed_ids;
}

void DefaultPlanner::updateMap(std::vector<ID> changed_id){
    for(int i=0;i<(int)(changed_id.size()); i++){
        planning_map_constructing_[changed_id[i]].height_ = planning_map_constructing_[changed_id[i]].masked_height_;
        planning_map_constructing_[changed_id[i]].m_cost_ = planning_map_constructing_[changed_id[i]].masked_height_;
        if(planning_map_constructing_[changed_id[i]].is_obstacle_ == false){
            planning_map_constructing_[changed_id[i]].is_obstacle_ = planning_map_constructing_[changed_id[i]].hidden_obstacle_;
        }
    }
}


bool DefaultPlanner::moveAndScan(int scan_range){
    // std::cout << "moving" << std::endl;
    path_changed_ = false;
    // last_node_ = path_traced_id_[0];
    
    path_traced_id_.erase(path_traced_id_.begin());
    path_traced_id_.shrink_to_fit();
    start_node_ = path_traced_id_[0];
    // std::cout << "at " << start_node_.x << "-" << start_node_.y << std::endl;
    if(matchID(start_node_, goal_node_) == true){
        std::cout << "goal reached" << std::endl;
        return false;
    }
    std::vector<ID> changed_ids = scanMap(start_node_, scan_range);
    if((int)(changed_ids.size()) > 0){
        std::cout << "detect changes " << std::endl;
        path_changed_ = true;
        planForward(start_node_,goal_node_, omega_,beta_weight_, gamma_weight_, max_slope_threshold_);
        // planForward(start_node_,goal_node_, static_obstacle_, omega_);

    }

    return true;
}

// void DefaultPlanner::exploreCell(PlanningMap& planning_map,int cur_id_x, int cur_id_y){ //dummy function, not used 
// }



void DefaultPlanner::exploreCell(){

    planning_map_constructing_[current_node_].is_explored_ = true;
    planning_map_constructing_[current_node_].is_exploring_ = false;

    if(matchID(current_node_, start_node_) == true){
        // which mean we are at the start node
        planning_map_constructing_[current_node_].parent_id_ = current_node_; // newly addeed 
    }
    for(int i = 0; i < 8 ; i ++ ){
        target_node_.setID(current_node_.x + d_heuristic_[i][0],current_node_.y + d_heuristic_[i][1]); // newly added

        if(target_node_.x * target_node_.y < 0 || planning_map_constructing_[target_node_].is_explored_ == true || isOutMap() || planning_map_constructing_[target_node_].is_obstacle_ == true){
            continue;
        }
        else if(target_node_.x * target_node_.y >=0 ){

            m_current_ = planning_map_constructing_[current_node_].m_cost_; // newly added 
            m_goal_ = planning_map_constructing_[target_node_].m_cost_;

            distance_sqr_ = pow(d_heuristic_[i][0]*cell_size_, 2) + pow(d_heuristic_[i][1]*cell_size_, 2);
            // distance_sqr_ = sqrt(distance_sqr_);
            distance_h_sqr_ = pow((target_node_.x-goal_node_.x)*cell_size_, 2) + pow((target_node_.y-goal_node_.y)*cell_size_, 2);

            
            if(planning_map_constructing_[current_node_].parent_id_x_ - current_node_.x != 0){
                turn_cost_current_ = beta_weight_ * calTurnAngle(d_heuristic_[i][0]*cell_size_, d_heuristic_[i][1]*cell_size_, m_goal_, 

                planning_map_constructing_[current_node_].parent_id_x_*cell_size_ - current_node_.x*cell_size_, 
                planning_map_constructing_[current_node_].parent_id_y_*cell_size_ - current_node_.y*cell_size_,
                m_current_);
            }


            h_current_ = euclideanDistance(target_node_.x*cell_size_, target_node_.y*cell_size_, m_goal_, 
            goal_node_.x*cell_size_, goal_node_.y*cell_size_,planning_map_constructing_[goal_node_].m_cost_) + turn_cost_current_;
            if(distance_h_sqr_>0){
               h_current_ = h_current_ + w_weight_hcost_*abs((planning_map_constructing_[goal_node_].m_cost_ - m_goal_))/distance_h_sqr_;
            }
            g_current_ =  cellToCellCost(distance_sqr_,m_current_ ,m_goal_) + planning_map_constructing_[current_node_].g_cost_;
            distance_cost_current_ = cellToCellCost(distance_sqr_,m_current_ ,m_goal_) + + planning_map_constructing_[current_node_].distance_cost_;
            if(g_current_>0){
                g_current_ = g_current_ + w_weight_gcost_*abs((m_goal_-m_current_))/distance_sqr_ + 
                gamma_weight_*planning_map_constructing_[current_node_].roughness_;  
            }

            //add slope cost 

            slope_cost_current_ = abs((m_goal_-m_current_))/sqrt(distance_sqr_);

            if(slope_cost_current_ > max_slope_threshold_){          //setting base slope value //magic number 
                slope_cost_current_ = 1e10;
            }

            f_current_ = h_current_ + g_current_ + exp(w_weight_gcost_ * slope_cost_current_); // new cost function 
            // planning_map_constructing_[goal_node_].setCellNewCost(m_current_,g_current_, h_current_,f_current_,cur_id_x,cur_id_y,slope_cost_current_);
            planning_map_constructing_[target_node_].setCellNewCost(m_current_,g_current_, h_current_,f_current_,current_node_,slope_cost_current_, distance_cost_current_);


            // add to exploring list 
            if(planning_map_constructing_[target_node_].is_exploring_ == false){
                planning_map_constructing_[target_node_].is_exploring_ = true;
            }

            if(matchID(target_node_,goal_node_) == true){

                found_path_ = true;
                planning_map_constructing_[goal_node_].is_goal_ = true;
                final_f_cost_ = planning_map_constructing_[goal_node_].f_cost_;
                final_distance_cost_ = planning_map_constructing_[goal_node_].distance_cost_;
                // std::cout << " found path" << std::endl;
                
                return; 
            }
        }
    }
}

void DefaultPlanner::findMinCostCellID(PlanningMap& planning_map){
    min_cost_h_= planning_map.cell_map[planning_map.exploring_cell_list[0][0]][planning_map.exploring_cell_list[0][1]].h_cost_;
    min_cost_f_= planning_map.cell_map[planning_map.exploring_cell_list[0][0]][planning_map.exploring_cell_list[0][1]].f_cost_;
    cur_id_x_ = planning_map.exploring_cell_list[0][0];
    cur_id_y_ = planning_map.exploring_cell_list[0][1];
       for(int i =1; i < (int)(planning_map.exploring_cell_list.size()); i++){

        if(min_cost_f_ > planning_map.cell_map[planning_map.exploring_cell_list[i][0]][planning_map.exploring_cell_list[i][1]].f_cost_){
            min_cost_f_ = planning_map.cell_map[planning_map.exploring_cell_list[i][0]][planning_map.exploring_cell_list[i][1]].f_cost_;

            cur_id_x_ = planning_map.exploring_cell_list[i][0];
            cur_id_y_ = planning_map.exploring_cell_list[i][1];
        }
    }

}

void DefaultPlanner::findMinCostCellID(){
    min_cost_f_ = 1e10;
    for (auto iter : planning_map_constructing_){
        if(planning_map_constructing_[iter.first].is_exploring_ == true && planning_map_constructing_[iter.first].is_explored_ == false){
            if(planning_map_constructing_[iter.first].f_cost_ < min_cost_f_){
                min_cost_f_ = planning_map_constructing_[iter.first].f_cost_ ;
                current_node_ = iter.first;
            }
        }
    }
}


bool DefaultPlanner::isInExplored(PlanningMap& planning_map){
    
    for(int i = 0; i < (int)(planning_map.explored_cell_list.size()); i++){
        if(goal_id_x_ ==planning_map.explored_cell_list[i][0] && goal_id_y_ ==planning_map.explored_cell_list[i][1]){
            return true;
        }
    }
    return false;
}

bool DefaultPlanner::isObstacle(PlanningMap& planning_map){
    
    for(int i = 0; i < (int)(planning_map.obstacle_cell_list.size()); i++){
        if(goal_id_x_ ==planning_map.obstacle_cell_list[i][0] && goal_id_y_ ==planning_map.obstacle_cell_list[i][1]){
            return true;
        }
    }
    return false;
}

bool DefaultPlanner::isOutMap(PlanningMap& planning_map){
    
    for(int i = 0; i < (int)(planning_map.map_id_list.size()); i++){
        if(goal_id_x_ == planning_map.map_id_list[i][0] && goal_id_y_ == planning_map.map_id_list[i][1]){
            return false;
        }
    }
    // std::cout << " is out map " << std::endl;
    return true;
}

bool DefaultPlanner::isOutMap(){
    
    if(target_node_.x > max_num_cel_x || target_node_.y > max_num_cel_y){
        planning_map_constructing_[target_node_].is_explored_ = true;
        return true;
    }
    else{
        return false; 
    }
    // return false;
    
}

bool DefaultPlanner::isInExploring(PlanningMap& planning_map){
    
    for(int i = 0; i < (int)(planning_map.exploring_cell_list.size()); i++){
        if(goal_id_x_ == planning_map.exploring_cell_list[i][0] && goal_id_y_ ==planning_map.exploring_cell_list[i][1]){
            return true;
        }
    }
    return false;
}


void DefaultPlanner::traceBackPathID(){
    bool complete_tracing = false;

    int estop = 0;
    double total_slope_cost = 0;

    ID iter_id;
    ID tmp_id;
    iter_id = goal_node_;
    path_traced_id_.push_back(iter_id);
    while(complete_tracing == false && estop < 700){
        // usleep(50000);
        estop+=1;

        tmp_id = planning_map_constructing_[iter_id].parent_id_ ;

        total_slope_cost+=planning_map_constructing_[iter_id].sl_cost_;

        iter_id = tmp_id;
        path_traced_id_.push_back(iter_id);


        //check reach path 
        if(matchID(iter_id, start_node_) == true){
            complete_tracing = true;
            // std::cout << " path is " ;
            path_traced_id_ = reversePath(path_traced_id_);

            // for(int i = 0; i < (int)(path_traced_id_.size());i++){
            //     std::cout << " " << path_traced_id_[i].x << "-" << path_traced_id_[i].y ;
            // }
            

            std::cout << " path size " << path_traced_id_.size() <<  std::endl;
            // std::cout << " fcost final is " << final_f_cost_ <<  std::endl;
            // std::cout << " distance final is " << final_distance_cost_ <<  std::endl;
            std::cout << " omega " << omega_ << " gamma " << gamma_weight_ << " mu " << max_slope_threshold_ << " beta " << beta_weight_ << std::endl;
            std::cout << " fcost " << final_f_cost_ << " distance " << final_distance_cost_ << " slope cost " << total_slope_cost << std::endl;
            
        }
    }
}

std::vector<ID> DefaultPlanner::reversePath(std::vector<ID> original_path){
    std::vector<ID> reverse_path;
    for(int i = int(original_path.size()-1); i >= 0; i--){
        reverse_path.push_back(original_path[i]);
    }
    return reverse_path;
}


void DefaultPlanner::clearOldPath(){
    path_traced_.clear();
    path_traced_.shrink_to_fit();
    path_traced_id_.clear();
    path_traced_id_.shrink_to_fit();
}

std::map<ID,Tile> DefaultPlanner::convertToUniversalMap(){
    ID iter_ID;
    Tile iter_tile;
    std::map<ID,Tile> universal_map;

    for (auto iter : planning_map_constructing_)   {
        iter_ID = iter.first;
        // iter_ID.y = i.first.y;
        iter_tile.height_ =  iter.second.height_;
        iter_tile.tile_size_ = iter.second.cell_size_;
        iter_tile.is_obstacle_ = iter.second.is_obstacle_;
        // if(iter.second.is_obstacle_ == true){
        //     std::cout << " convert / obs " << iter_ID.x << " " << iter_ID.y << std::endl;
        // }
        
        universal_map.insert(std::pair<ID,Tile>(iter_ID, iter_tile));
    }
    // std::cout << " convert uni map cell size " << iter_tile.tile_size << std::endl;
    return universal_map;
}

