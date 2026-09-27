#include "path_planner/dstar_lite_planner.hpp"

DStarLitePlanner::DStarLitePlanner()
{
    
}

DStarLitePlanner::~DStarLitePlanner()
{

}

bool DStarLitePlanner::planForward(ID start, ID goal){
 
    start_node_ = start;
    goal_node_ = goal;
    path_traced_id_.clear();
    queue_.clear();
    clearMap();
    forward_path_found_ = false;

    // createTestMap(planning_map_,start_node_,goal_node_);
    initialize(queue_,start_node_,goal_node_,k_m_);
    computeShortestPath(queue_,start_node_,goal_node_,k_m_);
    if(forward_path_found_ == true){
        traceBackPath();
        std::cout << "after path tracing " << std::endl;
        return forward_path_found_;
    }
    else{
        return forward_path_found_;
    }
    // printQueue();
    
    // printMap();
}

bool DStarLitePlanner::planForward(ID start, ID goal , std::vector<std::vector<int>> obstacle_array, double omega){
 
    start_node_ = start;
    goal_node_ = goal;
    omega_ = omega;
    path_traced_id_.clear();
    queue_.clear();
    clearMap();
    addObstacle(obstacle_array);
    // printMap();
    forward_path_found_ = false;

    // createTestMap(planning_map_,start_node_,goal_node_);
    initialize(queue_,start_node_,goal_node_,k_m_);
    computeShortestPath(queue_,start_node_,goal_node_,k_m_);
    if(forward_path_found_ == true){
        traceBackPath();
        std::cout <<  " after trace path" <<std::endl;
        return forward_path_found_;
    }
    else{
        return forward_path_found_;
    }
    // printQueue();
    
    // printMap();
}

bool DStarLitePlanner::planForward(ID start, ID goal, double omega, double beta, double gamma, double mu){

    start_node_ = start;
    goal_node_ = goal;
    omega_ = omega;
    mu_   = mu;
    gamma_ = gamma;
    beta_  = beta;
    path_traced_id_.clear();
    queue_.clear();
    clearMap();
    forward_path_found_ = false;

    initialize(queue_,start_node_,goal_node_,k_m_);
    computeShortestPath(queue_,start_node_,goal_node_,k_m_);
    if(forward_path_found_ == true){
        traceBackPath();
        std::cout <<  " after trace path" <<std::endl;
        return forward_path_found_;
    }
    else{
        return forward_path_found_;
    }

}

void DStarLitePlanner::printMap(){
    for (auto i : planning_map_)    // auto keyword 
		std::cout << " id " << i.first.x <<  " " << i.first.y <<  " rhs " << i.second.rhs_ << " g " << i.second.g_ << 
        " key " << i.second.key_.first << " " << i.second.key_.second << " obs " << i.second.is_obstacle_ <<  std::endl;
}

void DStarLitePlanner::printQueue(){
    for (int i =0; i< (int)(queue_.size()); i++){   // auto keyword 
		std::cout << " key " << queue_[i].first <<  " " << queue_[i].second <<  " id " << queue_[i].id.x << " " << queue_[i].id.y;
    }
    std::cout << std::endl;
}
void DStarLitePlanner::printQueue(std::vector<Key> &queue){
    for (int i =0; i< (int)(queue.size()); i++){   // auto keyword 
		std::cout << " (key " << queue[i].first <<  " " << queue[i].second <<  " id " << queue[i].id.x << " " << queue[i].id.y <<" )" ;
    }
    std::cout << std::endl;
}

void DStarLitePlanner::addToQueue(std::vector<Key> &queue, Key key){
   
    queue.push_back(key);
    planning_map_[key.id].key_ = key;
}

int DStarLitePlanner::calculateHeuristic( ID current_id){
    return (std::abs(start_node_.x - current_id.x)+std::abs(start_node_.y - current_id.y));
}

Key DStarLitePlanner::calculateKey( ID &current_id, int k_m){
    // std::cout << " calculate key " << std::endl;
    iter_key_ = Key();
    iter_key_.first = std::min(planning_map_[current_id].g_, planning_map_[current_id].rhs_) + calculateHeuristic(current_id) + k_m;
    iter_key_.second = std::min(planning_map_[current_id].g_, planning_map_[current_id].rhs_);
    iter_key_.id.x = current_id.x;
    iter_key_.id.y = current_id.y;
    return iter_key_;
}

double DStarLitePlanner::calTurnAngle(ID &current_id,  ID &target_id){
    double first_x, first_y, first_z, second_x, second_y, second_z;
    first_x = target_id.x - current_id.x;
    first_y = target_id.y - current_id.y;
    first_z = planning_map_[target_id].height_ - planning_map_[current_id].height_;

    second_x = current_id.x - planning_map_[current_id].parent_id_.x;
    second_y = current_id.y - planning_map_[current_id].parent_id_.y;
    second_z = planning_map_[target_id].height_ - planning_map_[current_id].height_;

    return acos((first_x*second_x + first_y*second_y + first_z*second_z)/
    (sqrt(first_x*first_x +first_y*first_y + first_z*first_z)*sqrt(second_x*second_x +second_y*second_y + second_z*second_z)));
}


void DStarLitePlanner::initialize(std::vector<Key> &queue, ID s_start, ID s_goal, int k_m){
    queue.clear();
    k_m_ = k_m;
    //set all g and rhs to infinity
    planning_map_[s_goal].rhs_ = 0;
    // addToQueue(queue,calculateKey(planning_map_, s_goal,s_start,k_m_));
    addToQueue(queue,calculateKey(s_goal,k_m_));
    // printMap();
    // printQueue();
}

std::vector<ID> DStarLitePlanner::getNeighbor(ID u){
    std::vector<ID> result;
    ID iter_id;
    for(int i = -1; i <= 1; i++){
        for(int j = -1; j <= 1; j++){
            if( u.x + i >=0 && u.y + j >=0 && (i*j + i+j !=0)){
                iter_id.setID(u.x + i, u.y + j);
                result.push_back(iter_id);
                // std::cout << "i " << i << " j " << j << std::endl;
            }
            else{
                continue;
            }
        }
    }
    return result;
}

bool DStarLitePlanner::compareCoordinate(ID u, ID v){
    return (u.x == v.x && u.y == v.y);
}

double DStarLitePlanner::cost( ID target_id, ID current_id){
    double distance;
    double slope;
    if(planning_map_[current_id].is_obstacle_ == true || planning_map_[target_id].is_obstacle_ == true) {
        return 1e10;
    }
    else{

        // return(std::sqrt(std::pow(target_id.x*1.0 - current_id.x*1.0, 2) + std::pow(target_id.y*1.0 - current_id.y*1.0, 2)));
        distance = std::sqrt(std::pow(target_id.x*cell_size_ - current_id.x*cell_size_, 2) + 
        std::pow(target_id.y*cell_size_ - current_id.y*cell_size_, 2) +
        std::pow(planning_map_[target_id].height_ - planning_map_[current_id].height_, 2));
        slope = std::abs(planning_map_[target_id].height_ - planning_map_[current_id].height_)/distance;
        // std::cout << " slope " << slope << std::endl;
        if(slope > mu_){
            slope = 1e10;
        }

        // return(distance + omega_*slope );
        return(distance + omega_*slope + gamma_*planning_map_[target_id].roughness_ + beta_*calTurnAngle(current_id, target_id));
    }
    
    // return 1;
}

void DStarLitePlanner::updateVertex( std::vector<Key> &queue, ID current_id, int k_m){
    if(compareCoordinate(current_id,goal_node_) == false){
        double min_rhs{1e10}; 
        for(int i=0; i < (int)(getNeighbor(current_id).size()); i++){
            
            // min_rhs = std::min(min_rhs, cost(planning_map_, current_id, getNeighbor(current_id)[i]));
            min_rhs = std::min(min_rhs, cost( current_id, getNeighbor(current_id)[i]) + planning_map_[getNeighbor(current_id)[i]].g_);
        }
        planning_map_[current_id].rhs_ = min_rhs;
        // std::cout << "rhs " << planning_map_[current_id].rhs_<< " g " << planning_map_[current_id].g_ <<std::endl;
        // std::cout << "g " << planning_map_[current_id].g_<< std::endl;
    }

    for (int i = 0; i< (int)(queue_.size()); i++){
        if(compareCoordinate(queue[i].id, current_id) == true){
            queue.erase(queue.begin() + i);
        }
    }

    if(planning_map_[current_id].g_ != planning_map_[current_id].rhs_){
        //  std::cout << "    add to queue " << current_id.x << " " << current_id.y << std::endl;
         Key tgkey = calculateKey( current_id,k_m);
        //  std::cout << "     key here " << tgkey.first << " " << tgkey.second << " " << tgkey.id.x <<" " << tgkey.id.y <<std::endl;
        //  queue.push_back(tgkey);
        addToQueue(queue, calculateKey( current_id,k_m));
    }
    
}

bool DStarLitePlanner::compareKey(Key key1,Key key2){
    if(key1.first != key2.first){
        return key1.first < key2.first;
    }
    else{
        return key1.second < key2.second;
    }
}

bool DStarLitePlanner::matchID(ID id1, ID id2){
    if(id1.x == id2.x && id1.y == id2.y){
        return true;
    }
    else{
        return false;
    }
}

Key DStarLitePlanner::getTopKey(std::vector<Key> &queue){
    //add condition check queue size = 0
    Key min_key = queue[0];
    // std::cout << " getTopKey ";
    if((int)(queue_.size()) > 1){
    for (int i = 1; i< (int)(queue_.size()); i++){   
		if(compareKey(queue[i],min_key) == true){
            min_key = queue[i]; 
        }
    }  
    }
    return min_key;  		
}

Key  DStarLitePlanner::popTopKey(std::vector<Key> &queue){
    //add condition check queue size = 0
    Key min_key = queue_[0];
    int min_index = 0;
    for (int i = 1; i< (int)(queue_.size()); i++){   
		if(compareKey(queue[i],min_key) == true){
            min_key = queue[i];
            min_index = i;
        }
    }  
    queue.erase(queue.begin() + min_index);
    return min_key;  		
}

void DStarLitePlanner::computeShortestPath(std::vector<Key> &queue,ID s_start, ID s_goal, int k_m ){

    Key k_old;
    ID u;  //iteration id

    int count_loop_break = 0;
    while( compareKey(getTopKey(queue) , calculateKey(s_start,k_m)) || planning_map_[s_start].rhs_ != planning_map_[s_start].g_){

        planning_map_[getTopKey(queue).id].parent_id_ = u;  // assign to trace path
        unordered_path_.push_back(u);  //add to unodered path 
        k_old = popTopKey(queue);
        u = k_old.id;

        // printQueue(queue);
        // if(k_old < calculateKey(u,k_m)){
        if(compareKey(k_old ,calculateKey(u,k_m))){
            // unordered_path_.push_back(u);
            addToQueue(queue,calculateKey(u,k_m));
        }
        else if(planning_map_[u].g_ > planning_map_[u].rhs_){
            // std::cout << "second if " << std::endl;
            planning_map_[u].g_ = planning_map_[u].rhs_;
            // std::cout <<  planning_map_[s_start].rhs_ << " " << planning_map_[s_start].g_ << std::endl;
            for(int i = 0; i< (int)(getNeighbor(u).size()); i++){
                // std::cout << "  inspecting neighbor " << getNeighbor(u)[i].x << " " << getNeighbor(u)[i].y << std::endl;;

                updateVertex(queue, getNeighbor(u)[i],k_m);

                // planning_map_[getNeighbor(u)[i]].parent_id_ = u;
                // printQueue(queue);
            }
        }
        else{
            // std::cout << "else " << std::endl;
            planning_map_[u].g_ = 1e10;
            // updateVertex(queue, u ,k_m);
            for(int i = 0; i< (int)(getNeighbor(u).size()); i++){
                updateVertex(queue,getNeighbor(u)[i],k_m);

            }

        }
        // std::cout << "getTopKey " << u.x << " " << u.y <<  " rhs " << planning_map_[u].rhs_ <<  " g " <<planning_map_[u].g_ << " key " <<
        // planning_map_[u].key_.first << " " << planning_map_[u].key_.second << std::endl;
        count_loop_break ++;
        if(count_loop_break == max_compute_path_iter){
            break;
        }
    }
    if(k_old.id.x == start_node_.x && k_old.id.y == start_node_.y){
        std::cout << " found forward path " << std::endl;
        forward_path_found_ = true;

    }

}


bool DStarLitePlanner::isNeighbor(ID id1, ID id2){
    if(std::abs(id1.x - id2.x) <=1 && std::abs(id1.y - id2.y) <=1){
        if(compareCoordinate(id1,id2) == false){
            return true;
        }
    }
    return false;
}

void DStarLitePlanner::traceBackPath(){
    std::cout << " trace path begin " << std::endl;
    path_traced_id_.push_back(start_node_);
    // unordered_path_.pop_back();
    // unordered_path_.shrink_to_fit();
    // for(int i=0; i< (int)(unordered_path_.size());i++){
    //     std::cout << " unordered path " << unordered_path_[i].x << "-" << unordered_path_[i].y << std::endl;
    // }
    ID iter_id = start_node_;
    bool goal_reach = false;
    std::cout << " path is " << start_node_.x << "-" << start_node_.y << " " ; 
    int loop_break_count = 0;
    //  std::vector<ID>::iterator it_find;
    ID min_id;
    int min_iter;
    double min_key2 = planning_map_[start_node_].key_.second;
    
    while(goal_reach == false && loop_break_count < max_trace_path_iter){
        
        for(int i=0; i< (int)(unordered_path_.size());i++){
            // if(compareCoordinate(iter_id,unordered_path_[i]) == true){
            //     unordered_path_.erase(unordered_path_.begin() + i);
            //     unordered_path_.shrink_to_fit();
            //     break;
            // }
            
            if( isNeighbor(iter_id,unordered_path_[i]) == true ){
                // std::cout << " checking " << unordered_path_[i].x << "-" << unordered_path_[i].y << std::endl;
                if(planning_map_[unordered_path_[i]].key_.second < min_key2){
                    min_id = unordered_path_[i];
                    min_iter = i;
                    min_key2 = planning_map_[unordered_path_[i]].key_.second;
                }
            }
        }
        iter_id = min_id;
        path_traced_id_.push_back(min_id);
        unordered_path_.erase(unordered_path_.begin() + min_iter);
        unordered_path_.shrink_to_fit();
        std::cout << min_id.x << "-" << min_id.y << " " ; 
        if(compareCoordinate(min_id, goal_node_) == true){
            goal_reach = true;
            unordered_path_.clear();
        }
        loop_break_count++;
    }
    std::cout << " path size " << path_traced_id_.size() <<  std::endl;

    
    
}

std::vector<ID> DStarLitePlanner::scanMap(ID current_id, int range){
    std::vector<ID> changed_ids;
    ID iter_id;
    for(int i = -1*range; i <= range; i++){
        for(int j = -1*range; j <= range; j++){
            if( current_id.x + i >=0 && current_id.y + j >=0 && (i*j + i+j !=0)){
                iter_id.x = current_id.x + i;
                iter_id.y = current_id.y + j;

                if(planning_map_[iter_id].hidden_obstacle_ == true && planning_map_[iter_id].is_obstacle_ == false){
                    planning_map_[iter_id].is_obstacle_ =true;
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


bool DStarLitePlanner::moveAndScan(){
    path_changed_ = false;
    last_node_ = path_traced_id_[0];
    path_traced_id_.erase(path_traced_id_.begin());
    path_traced_id_.shrink_to_fit();
    start_node_ = path_traced_id_[0];
    if(matchID(start_node_, goal_node_) == true){
        return false;
    }
    std::vector<ID> changed_ids = scanMap(start_node_, 2);
    if((int)(changed_ids.size()) > 0){
        std::cout << "detect changes " << std::endl;
        path_changed_ = true;
        k_m_ = k_m_ + calculateHeuristic(last_node_);
        for(int i =0;i<(int)(changed_ids.size()); i++){
            updateVertex(queue_,changed_ids[i],k_m_ );
        }

        path_traced_id_.clear();
        queue_.clear();
        clearMap();
        initialize(queue_,start_node_,goal_node_,k_m_);
        
        computeShortestPath(queue_,start_node_,goal_node_,k_m_);
        traceBackPath();
    }
    
    return true;
}

void DStarLitePlanner::addObstacle(std::vector<std::vector<int>> obstacle_array){
    ID iter_id;
    for(int i=0; i< (int)(obstacle_array.size()); i++ ){
        
        iter_id.x = obstacle_array[i][0];
        iter_id.y = obstacle_array[i][1];
        
        if(matchID(iter_id, start_node_) == true || matchID(iter_id, goal_node_) == true){
            continue;
        }
        planning_map_[iter_id].is_obstacle_ = true;
        // std::cout << " id " << iter_id.x << "-" << iter_id.y << " obstacle " << std::endl;
    } 
}

void DStarLitePlanner::addHiddenObstacle(std::vector<std::vector<int>> hidden_obstacle_array){
    ID iter_id;
    for(int i=0; i< (int)(hidden_obstacle_array.size()); i++ ){
        
        iter_id.x = hidden_obstacle_array[i][0];
        iter_id.y = hidden_obstacle_array[i][1];
        
        if(matchID(iter_id, start_node_) == true || matchID(iter_id, goal_node_) == true){
            continue;
        }
        planning_map_[iter_id].hidden_obstacle_ = true;

    } 
}

void DStarLitePlanner::importMap(MapHandler& map_hd){
    ID iter_id;
    Cell cell_iter;
    cell_size_ = map_hd.cell_size_;
    // std::cout << "import map cell size " << cell_size_ << std::endl;

    for(int i = 0; i < map_hd.grid_map.num_cel_x; i++){
        for(int j = 0; j < map_hd.grid_map.num_cel_y; j++){
            // iter_id.x = map_hd.grid_map.grid_matrix[i][j].getID().first;
            // iter_id.y = map_hd.grid_map.grid_matrix[i][j].getID().second;
            iter_id.x = i;
            iter_id.y = j;
            cell_iter.cell_size_ = cell_size_;
            if(map_hd.grid_map.grid_matrix[i][j].getOverThree() == true){
                cell_iter.height_ = map_hd.grid_map.grid_matrix[i][j].getMeanCenterCoordinate()[2];
                // std::cout << " get height from map " << cell_iter.height_ << std::endl;
                cell_iter.is_obstacle_ = false;
                
            }
            else if(map_hd.grid_map.grid_matrix[i][j].getOverThree() == false){
                // cell_iter.height_ = map_hd.grid_map.grid_matrix[i][j].getMeanCenterCoordinate()[2];
                cell_iter.is_obstacle_ = true;
            }
            planning_map_.insert(std::pair<ID,Cell>(iter_id, cell_iter)); 
        }
    }
}

void DStarLitePlanner::transferMap(std::map<ID,Tile> base_planning_map){
    Cell iter_cell;
    ID iter_id;
    for (auto iter : base_planning_map)   {
        // iter_id = iter.first;
        iter_id.x = iter.first.x;
        iter_id.y = iter.first.y;
        iter_cell.cell_size_ = iter.second.tile_size_;
        iter_cell.height_ = iter.second.height_;

        iter_cell.masked_height_ = iter.second.masked_height_;
        iter_cell.tile_size_ = iter.second.tile_size_;
        iter_cell.is_obstacle_ = iter.second.is_obstacle_;
        iter_cell.hidden_obstacle_ = iter.second.hidden_obstacle_;
        iter_cell.roughness_ = iter.second.roughness_;
        if(iter.second.max_id_ == true){
            max_num_cel_x = iter_id.x;
            max_num_cel_y = iter_id.y;
        }
        planning_map_.insert(std::pair<ID,Cell>(iter_id, iter_cell)); //constructing 
    }
    cell_size_ = iter_cell.cell_size_;
}

void DStarLitePlanner::createTestMap(){
    ID iter_id;
    Cell cell_iter;

    for (int i=0;i<20;i++){
        for (int j=0;j<20;j++){
            iter_id.x = i;
            iter_id.y = j;
            // if(i == s_goal.x && j == s_goal.y){
            //     cell_iter.rhs_ = 0;
            // }
            // planning_map_[iter_id] = cell_iter;
            planning_map_.insert(std::pair<ID,Cell>(iter_id, cell_iter));
            // cell_iter.rhs_ = 1e10;
        }
    }
    // printMap();
}

void DStarLitePlanner::clearMap(){
    // Cell empty_cell;
    for (auto i : planning_map_)    {
        planning_map_[i.first].g_ = 1e10;
        planning_map_[i.first].rhs_ = 1e10;
    }
		
}


void DStarLitePlanner::runTestFunc(){
    ID start_node;
    ID goal_node;
 
    start_node.x = 3;
    start_node.y = 9;
    goal_node.x = 13;
    goal_node.y = 1;
    start_node_ = start_node;
    goal_node_ = goal_node;
    // std::map<ID,Cell> planning_map_;
    // createTestMap();
    initialize(queue_,start_node,goal_node,k_m_);
    computeShortestPath(queue_,start_node,goal_node,k_m_);
    // printQueue();
    traceBackPath();
    // printMap();
}


std::map<ID,Tile> DStarLitePlanner::convertToUniversalMap(){
    ID iter_ID;
    Tile iter_tile;

    for (auto iter : planning_map_)   {
		// std::cout << i.first.x <<  " " << i.first.y <<  " rhs " << i.second.rhs_ << " g " << i.second.g_ << std::endl;
        iter_ID = iter.first;
        // iter_ID.y = i.first.y;
        iter_tile.height_ =  iter.second.height_;
        iter_tile.tile_size_ = iter.second.cell_size_;
        iter_tile.is_obstacle_ = iter.second.is_obstacle_;
        universal_map_.insert(std::pair<ID,Tile>(iter_ID, iter_tile));
    }
    // std::cout << " convert uni map cell size " << iter_tile.tile_size << std::endl;
    return universal_map_;
}

