#ifndef __MAP_HANDLER__
#define __MAP_HANDLER__


#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <iostream>
// #include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <istream>
#include <Eigen/Core>
#include <Eigen/Dense>
// #include "tf_conversions/tf_eigen.h"

struct MapPoint
{ 
public:
    double x, y, z;   //single point in mpa with x y z coordinate 
};

struct GridCell
{
    // GridCell(double cb_x, double cb_y, double cb_z, double cb_d);

public:
    void setCell(double val_x,double val_y,double val_z,double val_d);
    void setID(int val_x, int val_y);
    std::pair<int, int> getID();
    void addPoint(MapPoint point);
    void setOverThree(bool over);
    bool getOverThree();
    std::vector<MapPoint> getPoints();
    int getNumberOfPoint();
    void setMeans(double val_x,double val_y,double val_z);
    void setPlaneCoef(double val_a,double val_b,double val_c);
    std::vector<double> getCoordinate();
    std::vector<double> getMeans(); 
    std::vector<double> getPlaneCoef(); // get the plane function cooef that fit into cell, defined as coef_a*x + coef_b*y + coef_c = z 
    void setMeanCenterCoordinate(double val_x,double val_y,double val_z); // set the coordinate of the center point mean wise 
    std::vector<double> getMeanCenterCoordinate(); // get the coordinate of the center point mean wise 
    double calRMS(); //get root mean square of distances between points to fitted plane 
    double getRMS(); //get root mean square of distances between points to fitted plane 


private:
    std::pair<int, int> index_xy; // grid cell id x and y
    bool over_3_values{false};
    double cb_x, cb_y, cb_z, cb_d ; // grid cell coordinate and dimension (length of edge)
    double mean_x,mean_y,mean_z;    // mean of points 
    double center_mean_x,center_mean_y, center_mean_z;    // center mean wise of cell
    double coef_a_, coef_b_, coef_c_;
    std::vector<double> coef_vect;
    std::vector<MapPoint> cell_points;
    double rms_;
};
class GridMap
{
public:
    double size_x, size_y;
    double resolution, cell_size;
    int num_cel_x,num_cel_y;
    int point_per_cell{40};
    double size_adj_ ;

    // individual cell operation
    Eigen::MatrixXd matrix_A;   // matrix A 
    Eigen::MatrixXd matrix_B;   // matrix B 
    Eigen::MatrixXd matrix_AT;  // matrix A transpose 
    Eigen::MatrixXd matrix_AT_A;  // matrix (AT*A) 
    Eigen::MatrixXd matrix_AT_A_inv;  // matrix (AT*A) power of (-1)
    Eigen::MatrixXd matrix_A_sum;  // matrix ((AT*A)^(-1))*AT
    Eigen::MatrixXd matrix_X;  // matrix X
    double c_coef_a, c_coef_b, c_coef_c;
    double c_x, c_y, c_z , c_d;

    double max_rms_{0};
    double min_rms_{1};

    std::vector<std::vector<GridCell>> grid_matrix;
    void fitPlaneOfCellPoints(GridCell& target_cell);    
    void findMeanOfCells();
    void setMeanCenterOfCells(GridCell& target_cell);
};

class MapHandler //a class for a specific object , in this case is pid controller 
{
public:
    MapHandler(); 
    ~MapHandler();

    // these are variable that will be used  
    std::vector<std::vector<double>> raw_xyz_map;
    std::vector<double> raw_point_;
    std::string raw_line_;
    MapPoint point;
    double number_of_point{0};
    double resolution_{0};
    double cell_size_;
    std::vector<MapPoint> points_list;
    GridCell cell;
    GridMap grid_map;
    
    
    // 
    
    // MapPoint point; //this point is for iteration
    double max_x_ = 0;
    double max_y_ = 0;
    double max_z_ = 0;
    double min_x_ = 0;
    double min_y_ = 0;
    double min_z_ = 0;

    double max_rms_{0};
    double min_rms_{0};

    //resolution, max_z, cell size, numcell x, numcell y, upper range roughness, lower range roughness
    std::vector<double> transfer_params_{0,0,0,0,0,0,0};



    // these are the functions, which is going to do something when being called 
    void setParam(double cell_size, double point_per_cell, double size_adjust);
    void getMap(std::string file_dir);
    void collectMapData(std::string file_name);
    void printTestData();
    void findMaxMin(double& target, double& cur_max, double& cur_min);
    void shiftToZeroBase();
    void sortIntoGridMap();


};


#endif