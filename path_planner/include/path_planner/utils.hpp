#ifndef __UTILS__
#define __UTILS__

#include <map>
#include <vector>


struct ID{
    int x;
    int y;
    bool operator < (const ID & rhs) const {
        return ((x < rhs.x) || ((x == rhs.x) && (y < rhs.y)));
    }
    bool operator==(const ID& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
    void setID(int set_x,int set_y);
};
struct Key{
    float first;
    float second;
    ID id;
    bool operator < (const Key & rhs) const {
        return ((first < rhs.first) || ((first == rhs.first) && (second < rhs.second)));
    }

};

struct Tile{
    double height_;
    double masked_height_{0};
    double tile_size_;
    bool is_obstacle_{false};
    bool hidden_obstacle_{false};
    bool max_id_{false};
    double roughness_{0};

};
    

#endif