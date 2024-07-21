#ifndef VehicleTypeFuncs_H
#define VehicleTypeFuncs_H

#include <chrono>
#include <list>
#include <fstream>
#include <string>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/lib/rapidjson/writer.h>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/lib/rapidjson/document.h>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/lib/rapidjson/stringbuffer.h>
#include <iostream>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/headers/VehicleTypeUtils//VehicleTypeFuncs.h>
#include <vector>

namespace rapidjson
{
    class FileWriteStream;
}

using namespace std;
using namespace std::chrono;

class car
{
protected:
    
    duration<float> delta_time_, delta_init_time_; // make delta_init_time_ readonly after first assignment
    int id_;
    int collision_level_;
    float route_efficiency_;
    float pred_route_efficiency_;
    float* driver_chars_;
    string folder_;
    pair<float,float> curr_loc_, dest_loc_, init_loc_;
    pair<high_resolution_clock::time_point, high_resolution_clock::time_point> time_;

public:
    
    car(pair<float,float> currloc, pair<float,float> destloc, int id, float* driverChars)
    {
        
        // 2 values uninitialized b/c itsok
        this->delta_init_time_ = this->delta_time_;
        this->curr_loc_ = currloc;
        this->init_loc_ = currloc;
        this->dest_loc_ = destloc;
        this->route_efficiency_ = 0.0f;
        this->id_ = id;
        this->collision_level_ = 0;
        this->driver_chars_ = driverChars;
        // this->predict_route_efficiency();
        this->folder_ = "C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/results";
        
    }

    bool startTimer(high_resolution_clock::time_point simClock)
    {
        this->time_ = make_pair(simClock, high_resolution_clock::now());
        this->delta_init_time_ = duration_cast<duration<float>>(this->time_.second-this->time_.first);
    }
    
    void arrived()
    {
        this->time_.second = high_resolution_clock::now();
        this->delta_time_ = duration_cast<duration<float>>(time_.second - time_.first);
        this->calc_route_efficiency();
        this->report(this->folder_); // insert folderpath
    }

    void collision(list<car> parties, pair<float,float> loc)
    {
        this->collision_level_ = 3; // make determiner
        this->arrived();
    }

    virtual void report(string folderpath)
    {
        rapidjson::Document d;
        d.SetObject();

        d.AddMember("id", this->id_, d.GetAllocator());
        d.AddMember("collided", this->collision_level_, d.GetAllocator());
        d.AddMember("spawnLoc", this->init_loc_, d.GetAllocator());
        d.AddMember("DestLoc", this->dest_loc_, d.GetAllocator());

        ofstream file("AutonCar"+to_string(this->id_)+".json");
        rapidjson::FileWriteStream os(file, buffer, sizeof(buffer));
        rapidjson::Writer<rapidjson::FileWriteStream> writer(os); 
        d.Accept(writer); 
    }

    virtual void proposed_move(vector<float>* ptr) // AD* Lite x,y
    {
        vector<float> a;
        ptr = &a;
    }

    virtual void real_move(vector<float> movement) // In the lattice theoretic phase, the proposed_move is ignored
    {
        if (curr_loc_ == dest_loc_) {this->arrived();}
    }

    float get_route_efficiency() {return this->route_efficiency_;}

    void calc_route_efficiency() {this->route_efficiency_ = static_cast<float>(1.0-(this->delta_time_/this->pred_route_efficiency_).count());}
    // void predict_route_efficiency() {this->pred_route_efficiency_ = 0.0;} // fill in

};

#endif