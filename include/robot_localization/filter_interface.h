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
        void measurement_update(const Eigen::VectorXd& Z);
        Eigen::VectorXd estimate_state(double ref, double dt);
        virtual ~filter_interface();

    private:
        chrono::time_point<std::chrono::high_resolution_clock> start_time_;
        unique_ptr<Ekf> ekf_;
        double dt_;

    protected:
        long get_ms();

    };
}


#endif //GENERICFILTERS_FILTER_INTERFACE_H
