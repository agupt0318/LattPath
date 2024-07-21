#ifndef GraphFuncs_H
#define GraphFuncs_H

#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/headers/GraphUtils/GraphFuncs.h>
#include <utility>
using namespace std;

class node
{
protected:
    float cost_;
    pair<float,float> loc_;

public:

    node(float init_cost, pair<float,float> loc)
    {
        this->cost_ = init_cost;
        this->loc_ = loc;
    }

    float get_cost() {return this->cost_;}
    pair<float,float> get_loc() {return this->loc_;}

    void set_cost(float val) {this->cost_ = val;}
    
};

#endif
