//
// Created by redwan on 7/20/21.
//

#ifndef GENERICFILTERS_FILTER_INTERFACE_H
#define GENERICFILTERS_FILTER_INTERFACE_H

#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <memory>
#include "robot_localization/ekf.h"
#include "robot_localization/filter_base.h"
using namespace std;
namespace RobotLocalization
{
    class filter_interface {
    public:
        filter_interface();
        // update filter with pose information
        void measurement_update(const Eigen::VectorXd& Z);
        // return current state and update the detection lag
        Eigen::VectorXd estimate_state(double ref, double dt);
        virtual ~filter_interface();

    private:
        // cpp chrono library is used to track time
        chrono::time_point<std::chrono::high_resolution_clock> start_time_;
        // hassle free free memory managment
        unique_ptr<Ekf> ekf_;
        // prediction time stamp
        double dt_;

    protected:
        // return elapsed time in millisecond
        long get_ms();

    };
}


#endif //GENERICFILTERS_FILTER_INTERFACE_H
