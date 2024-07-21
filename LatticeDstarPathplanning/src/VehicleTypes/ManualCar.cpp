#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/headers/VehicleTypeUtils/VehicleTypeFuncs.h>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/lib/rapidjson/prettywriter.h>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/lib/rapidjson/allocators.h>
using namespace std;

class ManualCar : public car {
public:

    int mode;
    
    ManualCar(pair<float,float> currloc, pair<float,float> destloc, int id, high_resolution_clock::time_point simClock, float* driverChars) :
        car(currloc, destloc, id, driverChars)
    {
        this->folder_ = ""; // pls set lol
    }

    // is unfazed by everything, Lattice theory + D* inapplicable, only A*/dijkstra based pathplanning
    void proposed_move(vector<float>* ptr) override // always AD*
    {
        
        // implementation
        // car::curr_loc_; something to change this value
    }

    void set_mode(int arg) {this->mode = arg;}

    void real_move(vector<float> movement) override
    {
        // impl.
    }

    void report(string folderpath) override
    {
        rapidjson::Document d;
        d.SetObject();

        d.AddMember("id", this->id_, d.GetAllocator());
        d.AddMember("collided", this->collision_level_, d.GetAllocator());
        d.AddMember("spawnLoc", this->init_loc_, d.GetAllocator());

        if (this->mode == 1)
        {
            d.AddMember("Dijkstra's", this->route_efficiency_, d.GetAllocator());
        }

        if (this->mode == 2)
        {
            d.AddMember("A*", this->route_efficiency_, d.GetAllocator());
        }

        else
        {
            d.AddMember("DFS", this->route_efficiency_, d.GetAllocator());
        }

        rapidjson::StringBuffer strbuf;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strbuf);
        d.Accept(writer);
    }

};