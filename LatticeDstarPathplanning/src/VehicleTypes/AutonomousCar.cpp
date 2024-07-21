#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/headers/VehicleTypeUtils/VehicleTypeFuncs.h>
#include <vector>
using namespace std;

class AutonomousCar : public car {
public:

    AutonomousCar(pair<float,float> currloc, pair<float,float> destloc, int id, high_resolution_clock::time_point simClock, float* driverChars) :
        car(currloc, destloc, id, driverChars) {}

    void proposed_move(vector<float>* ptr) override // always AD*
    {
        // implementation
        // car::curr_loc_; something to change this value
    }

    vector<float,float> real_move(vector<float, float> movement) override
    {
        // impl.
    }

};