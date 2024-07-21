#ifndef GeneralFuncs_H
#define GeneralFuncs_H

#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/headers/GeneralUtils/GeneralFuncs.h>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/VehicleTypes/AutonomousCar.cpp>
#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/VehicleTypes/ManualCar.cpp>
#include <c:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/GraphBuilder/Graph.cpp>
#include <c:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/headers/GraphUtils/GraphFuncs.h>
#include <utility>
#include <vector>
#include <thread>

#include "../../../lib/matplotlib/matplotlibcpp.h"
using namespace std;

class controlflow // helper class controlflow
{
public:

    int threads_;
    int num_auto_cars_;
    int num_manual_cars_;
    int total_cars_;
    high_resolution_clock::time_point simClock;
    
    vector<thread> autonomousThreads;
    vector<car> autonomousCars;
    vector<car> autonomousCarsCopy;
    vector<thread> manualThreads;
    vector<car> manualCars;
    vector<car> manualCarsCopy;

    Graph environment;

    controlflow()
    {
        this->threads_ = 8;
        this->num_auto_cars_ = 0;
        this->num_manual_cars_ = 0;
        this->total_cars_ = 0;
        this->testGroup(); // gg we pray
    }

    bool duplicateTo()
    {
        copy(this->manualCars.begin(), this->manualCars.end(), this->manualCarsCopy.begin());
        copy(this->autonomousCars.begin(), this->autonomousCars.end(), this->autonomousCarsCopy.begin());

        return true;
    }

    bool duplicateFrom()
    {
        copy(this->manualCarsCopy.begin(), this->manualCarsCopy.end(), this->manualCars.begin());
        copy(this->autonomousCarsCopy.begin(), this->autonomousCarsCopy.end(), this->autonomousCars.begin());

        return true;
    }

    bool init_env(int envId) // 1,2,3
    {
        if (envId == 1) {}

        if (envId == 2)
        {
            // code here to read the file from lib/HStationGraph.csv
        }

        else {}
        
        return true;
    }

    float* randomInitiation(pair<float,float> seed, float clustering)
    {
        float *arr;
        return arr;
    }

    bool init_threads()
    {
        // Create threads for AutonomousCar
        for (int i = 0; i < this->threads_/2; i++) {
            float* ptr;
            auto autonomous_car = AutonomousCar(make_pair(0.0f,0.0f), make_pair(0.0f,0.0f), i, high_resolution_clock::now(), ptr);
            this->autonomousCars.emplace_back(autonomous_car);
            autonomousThreads.emplace_back(autonomous_car.startTimer, ref(autonomous_car), this->simClock);
        }

        // Create threads for ManualCar
        for (int i = 0; i < this->threads_/2; i++) {
            float* ptr;
            auto manual_car = ManualCar(make_pair(0.0f,0.0f), make_pair(0.0f,0.0f), i+(this->threads_/2), high_resolution_clock::now(), ptr);
            this->manualCars.emplace_back(manual_car);
            manualThreads.emplace_back(manual_car.startTimer, ref(manual_car), this->simClock);
        }

        return true;
    }

    bool destroy_thread(int index, bool carType) // false for manual, true for auto -> 0,1
    {
        
        if (carType)
        {
            int i = 0;
            for (auto& thread : this->autonomousThreads) {
                if (i == index)
                {
                    thread.join();
                    break;
                }
                i++;
            }
        }
        
        else
        {
            int i = 0;
            for (auto& thread : this->manualThreads) {
                if (i == index)
                {
                    thread.join();
                    break;
                }
                i++;
            }
        }

        return true;
    }

    bool clearThreads(vector<thread> vec)
    {
        for(auto i : vec) {i.join();}
        return true;
    }

    bool testGroup()
    {
        list<float> p;

        // control group

        this->threads_ = 800; // 1:1
        this->total_cars_ = 800;
        this->num_auto_cars_ = static_cast<int>(0.0*this->total_cars_);
        this->num_manual_cars_ = static_cast<int>(1.0*this->total_cars_);

        for (int j = 0; j < 3; j++) // dijkstra, A*, AD*/LDA*
        {
            
        }
        
        // if (this->init_threads())
        // {
        //
        //     cout << "Timers Live!" << endl << endl;
        //     // while (!allThreadsTerminated)
        //     // repeated poll thread for proposed_move()
        //     // consider this move immediately
        //     // if proposed move is 0 or something, make thread report() and then terminate process via destroy_thread(int index, bool carType)
        //
        //     while(!(autonomousThreads.empty() and manualThreads.empty())) // DeMorgan's Law simplification
        //     {
        //         
        //         for(unsigned int i = 1; i < this->autonomousThreads.size(); i++) // only works if the threads and cars are 1:1
        //         {
        //             vector<float>* res;
        //             this->autonomousThreads.erase(this->autonomousThreads.begin());
        //             this->autonomousThreads.emplace_back(this->autonomousCars.at(i).proposed_move, ref(this->autonomousCars.at(i)), res);
        //         }
        //
        //         for (unsigned int i = 0; i < this->manualThreads.size(); i++)
        //         {
        //             vector<float>* res;
        //             this->manualThreads.erase(this->manualThreads.begin());
        //             this->manualThreads.emplace_back(this->manualCars.at(i).proposed_move, ref(this->manualCars.at(i)), res);
        //             // consider and pass back
        //         }
        //     }
        //     
        //     cout << "Demo subgroup 1 results compiled! (hopefully)" << endl << endl;
        //     
        // }

        // cout << "Timers Live!" << endl << endl;
        //     // while (!allThreadsTerminated)
        //     // repeated poll thread for proposed_move()
        //     // consider this move immediately
        //     // if proposed move is 0 or something, make thread report() and then terminate process via destroy_thread(int index, bool carType)

        this->threads_ = 800; // 1:1
        this->total_cars_ = 800;
        this->num_auto_cars_ = static_cast<int>(0.2*this->total_cars_);
        this->num_manual_cars_ = static_cast<int>(0.8*this->total_cars_);
        
        this->init_env(2);
        this->simClock = high_resolution_clock::now();

        for (int j = 0; j < 3; j++)
        {

            this->duplicateTo();
            
            while(!(autonomousCars.empty() and manualCars.empty())) // DeMorgan's Law simplification
            {

                for (unsigned int i = 0; i < this->autonomousCars.size(); i++)
                {
                    vector<float> *res;
                                            
                    if (res == nullptr)
                    {
                        this->autonomousCars.at(i).arrived();
                        this->autonomousCars.erase(this->autonomousCars.begin()+i);

                        if (i-1 < 0)
                        {
                            i = -1;
                        }
                    
                    }

                    else
                    {
                        vector<float> realMove;
                        
                        // always use LDA*
                        
                        this->autonomousCars.at(i).real_move(realMove);
                    }
                }

                for (unsigned int i = 0; i < this->manualCars.size(); i++)
                {

                    if (i == 1)
                    {
                        this->manualCars.at(i).set_mode(1);
                    }
                    if (i == 2)
                    {
                        this->manualCars.at(i).set_mode(2);
                    }
                    else
                    {
                        this->manualCars.at(i).set_mode(3);
                    }
                    
                    vector<float> *res;
                    this->manualCars.at(i).proposed_move(res);

                    if (res == nullptr)
                    {
                        this->autonomousCars.at(i).arrived();
                        this->autonomousCars.erase(this->autonomousCars.begin()+i);

                        if (i-1 < 0)
                        {
                            i = -1;
                        }
                    
                    }

                    else
                    {
                        vector<float> realMove;
                        
                        this->manualCars.at(i).real_move(realMove);
                    }
                }
            }

            this->duplicateFrom();
        }
        
        cout << "Demo subgroup 1 results compiled! (hopefully)" << endl << endl;
        
        this->threads_ = 800; // 1:1
        this->total_cars_ = 800;
        this->num_auto_cars_ = static_cast<int>(0.6*this->total_cars_);
        this->num_manual_cars_ = static_cast<int>(0.4*this->total_cars_);

        if (this->init_threads())
        {
            //
        }
        

        this->threads_ = 800; // 1:1
        this->total_cars_ = 800;
        this->num_auto_cars_ = static_cast<int>(1.0*this->total_cars_);
        this->num_manual_cars_ = static_cast<int>(0.0*this->total_cars_);

        if (this->init_threads())
        {
            //
        }

        return true;
        
    }
};

// bool controlflow::group1(bool s1, bool s2, bool s3)
// {
//
//     this->threads_ = 0; // change
//         
//     if (s1)
//     {
//         this->total_cars_ = 0;
//         this->num_auto_cars_ = static_cast<int>(0.2*this->total_cars_);
//         this->num_manual_cars_ = static_cast<int>(0.8*this->total_cars_);
//
//         if (this->init_threads()) // timers have started running
//             {
//             cout << "Timers Live!" << endl << endl;
//             // while (!allThreadsTerminated)
//             // repeated poll thread for proposed_move()
//             // consider this move immediately
//             // if proposed move is 0 or something, make thread report() and then terminate process
//             cout << "Demo subgroup 1 results compiled! (hopefully)" << endl << endl;
//             }
//     }
//
//     if (s2)
//     {
//         //
//     }
//
//     if (s3)
//     {
//         //
//     }
//
//     return true;
        
// }

#endif