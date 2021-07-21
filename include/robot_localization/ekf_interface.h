//
// Created by redwan on 7/20/21.
//

#ifndef GENERICFILTERS_EKF_INTERFACE_H
#define GENERICFILTERS_EKF_INTERFACE_H

#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <memory>
#include "robot_localization/ekf.h"
#include "robot_localization/filter_base.h"
#include <yaml-cpp/yaml.h>


using namespace std;


namespace RobotLocalization
{
    class ekf_interface {
    public:
        ekf_interface(double processNoise, double measurementNoise, double mahalanobisThresh);
        // update filter with pose information
        void measurement_update(const Eigen::VectorXd& Z);
        // return current state and update the detection lag
        Eigen::VectorXd estimate_state(double ref, double dt);
        // global elapsed time in millisecond
        static long get_ms();


        virtual ~ekf_interface();

    private:
        // hassle free  memory management
        unique_ptr<Ekf> ekf_;
        // prediction time stamp
        double dt_;
        // filter parameters
        double process_noise_, measurement_noise_;
        double mahalanobisThresh_;

    };
}


#endif //GENERICFILTERS_EKF_INTERFACE_H
