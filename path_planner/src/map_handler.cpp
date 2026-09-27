#include "path_planner/map_handler.hpp"

//the header file was included, which mean we can now declare the detail of it's context
MapHandler::MapHandler()
{
    max_x_ = 0;  
    grid_map.point_per_cell = 10; //default value 
    grid_map.size_adj_ = 1.0;
 
}
MapHandler::~MapHandler()
{
     max_x_ = 0;
     
}
void MapHandler::setParam(double cell_size, double point_per_cell, double size_adjust){
    grid_map.point_per_cell = point_per_cell;
    grid_map.size_adj_ = size_adjust;
    grid_map.cell_size = cell_size;
}

void MapHandler::getMap(std::string file_dir){
    collectMapData(file_dir);
    shiftToZeroBase();
    sortIntoGridMap();
    grid_map.findMeanOfCells();
    transfer_params_[5] = grid_map.max_rms_;
    transfer_params_[6] = grid_map.min_rms_;

}

void MapHandler::collectMapData(std::string file_name){ 

    int lcount = 0;
    // int count = 0;
    std::string val;
    std::string val1;
    std::string val2;
    std::string val3;
    double size_adj = grid_map.size_adj_;
    // std::vector<double> copy_raw_line;
    std::string raw_line__;
    
    long limit_test_line = 1e13 ;  // set to unlimited
    
    std::fstream input_file_str;
    input_file_str.open(file_name, std::ios::in); 
    // std::cout << "open the file - '"<< file_name << "'" << std::endl;
    if (!input_file_str.is_open()) {
        std::cout << "Could not open the file - '"<< file_name << "'" << std::endl;
    }
    
    std::stringstream str_s;
    // std::cout << "collect   ===map data " << std::endl;
    while (std::getline(input_file_str, raw_line__)){
        // std::cout << "loop === check " << std::endl; 
        if (lcount < limit_test_line){
            // std::cout << "===" << std::endl;
            // std::cout << raw_line__ << std::endl;
            str_s.str(raw_line__);
            str_s >> val1;
            str_s >> val2;
            str_s >> val3;
            // std::cout << stof(val1) << std::endl;
            // std::cout << val2 << std::endl;
            // std::cout << val3 << std::endl;
            // std::cout << " <<<+<  <  " << std::endl;

            // this->raw_point_.push_back(1);  

            raw_point_.push_back(stof(val1)*size_adj);  
            raw_point_.push_back(stof(val2)*size_adj);  
            raw_point_.push_back(stof(val3)*size_adj);  

            point.x = stof(val1)*size_adj;
            findMaxMin(point.x, max_x_, min_x_);
            point.y = stof(val2)*size_adj;
            findMaxMin(point.y, max_y_, min_y_);
            point.z = stof(val3)*size_adj;
            findMaxMin(point.z, max_z_, min_z_);

            number_of_point ++ ;
            points_list.push_back(point);
            // cell.addPoint(point);
            // count =0;
            raw_xyz_map.push_back(raw_point_);
            lcount ++ ;
        }
        raw_point_.clear();
        str_s.str(std::string());
        str_s.clear();
    }
    // std::cout << "check here << " << std::endl;
    // for(int i = 0;i<raw_xyz_map.size();i++){
    //     std::cout << " " << std::endl;
    //     for(int j = 0;j<raw_xyz_map[0].size();j++){
    //         std::cout << raw_xyz_map[i][j] << std::endl;
    //     }
    // }
    // std::cout << "test one data point " <<  cell.getPoints()[2000].x << std::endl;
    // std::cout << "test one data point " <<  points_list[2000].x << std::endl;
    // std::cout << "max x min x " <<  max_x << " " << min_x << std::endl;
    // std::cout << "max y min y " <<  max_y << " " << min_y << std::endl;
    std::cout << "number of point  " <<  number_of_point << std::endl;
}

void MapHandler::printTestData(){
    for(int i = 0;i<20;i++){
        std::cout << points_list[i].x << std::endl;
    }
}

void MapHandler::shiftToZeroBase(){
    for(int i = 0; i< number_of_point; i++){
        points_list[i].x-=min_x_;
        points_list[i].y-=min_y_;
        points_list[i].z-=min_z_;
    }
    max_x_-=min_x_;
    max_y_-=min_y_;
    max_z_-=min_z_;
    min_x_ = 0;
    min_y_ = 0;
    min_z_ = 0;
    // also find resolution 
    resolution_ = number_of_point/(max_x_*max_y_); // resolution in 2D number of points/meter
    std::cout << "resolution " << resolution_ << std::endl;
    transfer_params_[0] = resolution_;
    std::cout << "max z " << max_z_ <<std::endl;
    transfer_params_[1] = max_z_;


}

void MapHandler::findMaxMin(double& target, double& cur_max, double& cur_min){
    if(target <= cur_min){
        cur_min = target;
    }
    if(target >= cur_max){
        cur_max = target;
    }
}

void MapHandler::sortIntoGridMap(){

    grid_map.size_x = max_x_;
    grid_map.size_y = max_y_;
    grid_map.resolution = resolution_;
    std::vector<GridCell> grid_array_y;

    // grid_map.cell_size = grid_map.point_per_cell/resolution_;

    // grid_map.point_per_cell = int(std::ceil(grid_map.cell_size*resolution_));
    // grid_map.cell_size = grid_map.point_per_cell/resolution_;


    cell_size_ = grid_map.cell_size;
    grid_map.num_cel_x = int(std::ceil(max_x_/grid_map.cell_size));
    grid_map.num_cel_y = int(std::ceil(max_y_/grid_map.cell_size));
    std::cout << "cell size, numcell x, numcell y " << grid_map.cell_size <<" " << grid_map.num_cel_x << " " << grid_map.num_cel_y <<std::endl;
    transfer_params_[2] = grid_map.cell_size;
    transfer_params_[3] = grid_map.num_cel_x ;
    transfer_params_[4] = grid_map.num_cel_y ;
    
    for(int i = 0; i< grid_map.num_cel_x; i++){
        for(int j = 0; j< grid_map.num_cel_y; j++){
            cell.setCell(i*grid_map.cell_size,j*grid_map.cell_size,0,grid_map.cell_size);
            cell.setID(i, j);
            grid_array_y.push_back(cell);
        }
        // std::cout << "grid y size " << grid_array_y.size() << std::endl;
        grid_map.grid_matrix.push_back(grid_array_y);
        grid_array_y.clear();
    }
    int index_cell_x,index_cell_y;
    for(int e = 0;e < number_of_point;e ++){
        index_cell_x = std::floor(points_list[e].x/grid_map.cell_size);
        index_cell_y = std::floor(points_list[e].y/grid_map.cell_size);
        // std::cout << "add to " << index_cell_x << " " << index_cell_y << std::endl;
        grid_map.grid_matrix[index_cell_x][index_cell_y].addPoint(points_list[e]);

        // test index 
        // if(e == 12){
        //     std::cout << " point index 12 " << points_list[12].x<< " " <<  points_list[12].y << " " << index_cell_x << " " << index_cell_y << std::endl;
        // }
        // test index and values 
        // if(index_cell_x == 39 && index_cell_y == 44){
        //     std::cout << "point is " << points_list[e].x << " " << points_list[e].y << std::endl;
        // }
    }
    //for testing 
    // for(int e = 0;e < grid_map.grid_matrix[35][83].getNumberOfPoint();e ++){
    //     std::cout << " test grid map "<< grid_map.grid_matrix[35][83].getPoints()[e].x
    //     << " " <<    grid_map.grid_matrix[35][83].getPoints()[e].y 
    //     << " " <<    grid_map.grid_matrix[35][83].getPoints()[e].z 
    //     << std::endl;
    // }
}

void GridCell::setCell(double val_x,double val_y,double val_z,double val_d){
    cb_x = val_x;
    cb_y = val_y;
    cb_z = val_z;
    cb_d = val_d;
}

void GridCell::setID(int val_x, int val_y){
    index_xy.first = val_x;
    index_xy.second = val_y;
    // std::cout << "set id " << index_xy.first << " " << index_xy.second  << std::endl;
}

std::pair<int, int> GridCell::getID(){
    // std::cout << "get id " << index_xy.first << " " << index_xy.second << std::endl;
    return index_xy;
}

void GridCell::setOverThree(bool over){
    over_3_values = over;
}

bool GridCell::getOverThree(){
    return over_3_values ;
}

int GridCell::getNumberOfPoint(){
    return cell_points.size();
}

std::vector<double> GridCell::getCoordinate(){
    return std::vector<double> {cb_x,cb_y,cb_z,cb_d};
}

std::vector<double> GridCell::getMeans(){
    return std::vector<double> {mean_x,mean_y,mean_z};
}

void GridCell::addPoint(MapPoint point){
    this->cell_points.push_back(point);
}
void GridCell::setMeans(double val_x,double val_y,double val_z){
    mean_x = val_x;
    mean_y = val_y;
    mean_z = val_z;
}

void GridCell::setMeanCenterCoordinate(double val_x,double val_y,double val_z){
    center_mean_x = val_x;
    center_mean_y = val_y;
    center_mean_z = val_z;
}

void GridCell::setPlaneCoef(double val_a,double val_b,double val_c){
    coef_a_ = val_a;
    coef_b_ = val_b;
    coef_c_ = val_c;
    coef_vect = {1,0,0};
    coef_vect[0] = coef_a_;
    coef_vect[1] = coef_b_;
    coef_vect[2] = coef_c_;
    // std::cout << " plane center cooef from set is  " << coef_vect[0] << " " << coef_vect[1] << " " << coef_vect[2] << std::endl;

}

std::vector<double> GridCell::getPlaneCoef(){
    // std::cout << " plane center cooef is  " << coef_vect[0] << " " << coef_vect[1] << " " << coef_vect[2] << std::endl;
    // return this->coef_vect;
    // std::cout << " plane center cooef is  " << coef_a_<< " " << coef_b_ << " " << coef_c_ << std::endl;
    return coef_vect;
}

void GridMap::setMeanCenterOfCells(GridCell& target_cell){

    c_coef_a = target_cell.getPlaneCoef()[0];
    c_coef_b = target_cell.getPlaneCoef()[1];
    c_coef_c = target_cell.getPlaneCoef()[2];
    // std::cout << " plane center cooef is  " << c_coef_a << " " << c_coef_b << " " << c_coef_c << std::endl;
    c_d = target_cell.getCoordinate()[3];
    c_x = target_cell.getCoordinate()[0] + c_d/2;
    c_y = target_cell.getCoordinate()[1] + c_d/2;
    c_z = c_x*c_coef_a + c_y*c_coef_b + c_coef_c;
    target_cell.setMeanCenterCoordinate(c_x, c_y, c_z);

    // std::cout << " mean cente z is " << c_z << std::endl;

}

std::vector<double> GridCell::getMeanCenterCoordinate(){
    std::vector<double> mean_vect{center_mean_x,center_mean_y,center_mean_z};
    return mean_vect;
}

double GridCell::calRMS(){
    // double rms;
    double distance_sqr;
    for(int i=0;i<(int)(cell_points.size()); i++){
        distance_sqr+= pow(abs(coef_a_*cell_points[i].x + coef_b_*cell_points[i].y - cell_points[i].z + coef_c_)
                    /(sqrt(coef_a_*coef_a_ + coef_b_*coef_b_ +1)), 2);
    }
    rms_ = sqrt(distance_sqr/cell_points.size());
    // std::cout << " cell " << index_xy.first << " " << index_xy.second << " rms " << rms_*100 << std::endl;
    return rms_;
}

double GridCell::getRMS(){
    return rms_;
}



std::vector<MapPoint> GridCell::getPoints(){
    return cell_points;
}
void GridMap::fitPlaneOfCellPoints(GridCell& target_cell){
    int n = target_cell.getNumberOfPoint();
    matrix_A.resize(n,3);
    matrix_AT.resize(3,n);
    matrix_AT_A.resize(3,3);
    matrix_AT_A_inv.resize(3,3);
    matrix_A_sum.resize(3,n);
    matrix_B.resize(n,1);
    matrix_X.resize(3,1);
    
    if(target_cell.getOverThree()){
        // std::cout << "number of points " << target_cell.getNumberOfPoint() << std::endl;
        for(int i = 0; i < target_cell.getNumberOfPoint() ; i ++){
            matrix_A(i,0) = target_cell.getPoints()[i].x ; 
            matrix_A(i,1) = target_cell.getPoints()[i].y ; 
            matrix_A(i,2) = 1 ; 
            matrix_B(i,0) = target_cell.getPoints()[i].z ; 
            // std::cout << " points " << matrix_A(i,0) << " " << matrix_A(i,1) << " " << matrix_B(i,0) << std::endl;

        }
        matrix_AT = matrix_A.transpose();
        matrix_AT_A = matrix_AT*matrix_A;
        // matrix_AT_A_inv = matrix_AT_A.array().pow(-1);
        matrix_AT_A_inv = matrix_AT_A.inverse();
        matrix_A_sum = matrix_AT_A_inv*matrix_AT;
        matrix_X = matrix_A_sum*matrix_B;
        // std::cout << " cell id " << target_cell.getID().first << " " << target_cell.getID().second 
        // << " coef is " <<  matrix_X(0,0) << " " << matrix_X(1,0) << " "  <<matrix_X(2,0) << std::endl;
        target_cell.setPlaneCoef(matrix_X(0,0),matrix_X(1,0),matrix_X(2,0));

        target_cell.calRMS();
        if(target_cell.getRMS() > max_rms_){
            max_rms_ = target_cell.getRMS() ;
        }
        if(target_cell.getRMS() < min_rms_){
            min_rms_ = target_cell.getRMS() ;
        }
        //usleep(10000000);
        // target_cell.setMeanCenterCoordinate();
    }
    //reset matrix 
    // matrix_A.resize(0,0);
    // matrix_AT.resize(0,0);
    // matrix_AT_A.resize(0,0);
    // matrix_AT_A_inv.resize(0,0);
    // matrix_B.resize(0,0);
    //example 
    // matrix_A.array().pow(-1);  // exponential 
    // matrix_A.resize(0,0); // use to clear matrix 
}


void GridMap::findMeanOfCells(){
    double tmp_mean_x = 0;
    double tmp_mean_y = 0;
    double tmp_mean_z = 0;
    
    for(int i = 0; i < num_cel_x ; i ++){
        for(int j = 0; j < num_cel_y ; j ++){
            //need to filter empty cell
            if(grid_matrix[i][j].getNumberOfPoint() < 3){
                grid_matrix[i][j].setOverThree(false);
                // if(grid_matrix[i][j].getNumberOfPoint() == 0){
                //     continue;
                // }

                continue;
            }
            else{
                
                grid_matrix[i][j].setOverThree(true);
                for(int k = 0; k < grid_matrix[i][j].getNumberOfPoint(); k++){
                    tmp_mean_x += grid_matrix[i][j].getPoints()[k].x;
                    tmp_mean_y += grid_matrix[i][j].getPoints()[k].y;
                    tmp_mean_z += grid_matrix[i][j].getPoints()[k].z;
                }
                fitPlaneOfCellPoints(grid_matrix[i][j]);
                setMeanCenterOfCells(grid_matrix[i][j]);
                tmp_mean_x = tmp_mean_x/grid_matrix[i][j].getNumberOfPoint();
                tmp_mean_y = tmp_mean_y/grid_matrix[i][j].getNumberOfPoint();
                tmp_mean_z = tmp_mean_z/grid_matrix[i][j].getNumberOfPoint();
                grid_matrix[i][j].setMeans(tmp_mean_x,tmp_mean_y,tmp_mean_z);
                // std::cout << "mean of pos " << i <<" " << j << " " << tmp_mean_x << " " << tmp_mean_y << " " << tmp_mean_z << std::endl; 
                tmp_mean_x = 0;
                tmp_mean_y = 0;
                tmp_mean_z = 0;
                // std::cout << "========== " << std::endl;
            }
        }

    }
    std::cout << "range rms " << max_rms_ << " " << min_rms_ << std::endl;
    
}