//
// Created by redwan on 7/20/21.
//

#include "robot_localization/filter_interface.h"

#include <memory>
using namespace RobotLocalization;

filter_interface::filter_interface() {
    start_time_ = std::chrono::high_resolution_clock::now();
    ekf_ = std::make_unique<Ekf>();
    // set initial covariance R matrix
    Eigen::MatrixXd initialCovar(STATE_SIZE, STATE_SIZE);
    initialCovar.setIdentity();
    initialCovar *= 0.5;
    ekf_->setEstimateErrorCovariance(initialCovar);
    dt_ = 0.0;

}
filter_interface::~filter_interface()= default;

long filter_interface::get_ms() {
    const auto end_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();
}

void filter_interface::measurement_update(const Eigen::VectorXd &Z) {
    double time_elapsed = (double) get_ms()/ 1000.0;
//    cout << "[FilterInterface] update time " << time_elapsed << "sec \n Z: \n" << Z << endl;
    // measurement dimension
    size_t  dim = STATE_SIZE;
    // convert Z to measurement
    Measurement measurement;
    measurement.time_ = measurement.latestControlTime_ = time_elapsed;
    measurement.topicName_ = "camera_odom";
    // here we only update 6 parameters out of 15
    std::vector<int>updateVector(dim, 0);
    updateVector[0] = updateVector[1] = updateVector[2] = updateVector[3] = updateVector[4] = updateVector[5] = 1;
    std::copy(updateVector.begin(), updateVector.end(), back_inserter(measurement.updateVector_));

    measurement.measurement_ = Z;


    // Q matrix
    Eigen::MatrixXd measurementCovariance(dim, dim);
    measurementCovariance.setIdentity();
    for (size_t i = 0; i < dim; ++i)
    {
        measurementCovariance(i, i) = 1e-9;
    }
    measurement.covariance_ = measurementCovariance;

    //https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/params/ekf_template.yaml
    //odom0_pose_rejection_threshold: 5
    //odom0_twist_rejection_threshold: 1
    measurement.mahalanobisThresh_ = 5;

    // predict first
    ekf_->predict(time_elapsed,  time_elapsed + dt_);
    // with that measurement is ready for the correction step
    ekf_->correct(measurement);
}

Eigen::VectorXd filter_interface::estimate_state(double ref, double current) {

    dt_ = current - ref;
    return ekf_->getState();
}
