//
// Created by redwan on 7/20/21.
//

#include "robot_localization/ekf_interface.h"

#include <memory>
using namespace RobotLocalization;
// initialize a timer reference here to compute get_ms() during runtime
// cpp chrono library is used to track time
static chrono::time_point<std::chrono::high_resolution_clock> startTime  = std::chrono::high_resolution_clock::now();

ekf_interface::ekf_interface(double processNoise, double measurementNoise, double mahalanobisThresh)
: process_noise_(processNoise), measurement_noise_(measurementNoise), mahalanobisThresh_(mahalanobisThresh)
{
    // initialize base ekf
    ekf_ = std::make_unique<Ekf>();
    // set initial covariance R matrix
    Eigen::MatrixXd initialCovar(STATE_SIZE, STATE_SIZE);
    initialCovar.setIdentity();
    // TODO replace the magic number from yaml parameter
    initialCovar *= process_noise_;
    ekf_->setEstimateErrorCovariance(initialCovar);
    // initially we only predict for current step
    // as the time progress dt automatically get updated based on the detection and
    // prediction time difference
    dt_ = 0.0;

}
ekf_interface::~ekf_interface()= default;

long ekf_interface::get_ms() {
    // cpp chrono library is used to track time elapse for this class
    const auto end_time = std::chrono::high_resolution_clock::now();
    // elapsed time returned in millisecond
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - startTime).count();
}

void ekf_interface::measurement_update(const Eigen::VectorXd &Z) {
    double time_elapsed = (double) get_ms()/ 1000.0;
//    cout << "[FilterInterface] update time " << time_elapsed << "sec \n Z: \n" << Z << endl;
    // measurement dimension 15
    size_t  dim = STATE_SIZE;
    // convert Z to measurement
    Measurement measurement;
    measurement.time_ = measurement.latestControlTime_ = time_elapsed;
    measurement.topicName_ = "camera_odom";
    // here we only update 6 parameters out of 15
    std::vector<int>updateVector(dim, 0);
    // the first 6 parameters are pose information
    updateVector[0] = updateVector[1] = updateVector[2] = updateVector[3] = updateVector[4] = updateVector[5] = 1;
    // deep copy
    std::copy(updateVector.begin(), updateVector.end(), back_inserter(measurement.updateVector_));
    // detected pose from the apriltag
    measurement.measurement_ = Z;


    // Q matrix
    Eigen::MatrixXd measurementCovariance(dim, dim);
    measurementCovariance.setIdentity();
    for (size_t i = 0; i < dim; ++i)
    {
        measurementCovariance(i, i) = measurement_noise_;
    }
    measurement.covariance_ = measurementCovariance;

    //https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/params/ekf_template.yaml
    //odom0_pose_rejection_threshold: 5
    //odom0_twist_rejection_threshold: 1
    measurement.mahalanobisThresh_ = mahalanobisThresh_;

    // predict first
    ekf_->predict(time_elapsed,  time_elapsed + dt_);
    // with that measurement is ready for the correction step
    ekf_->correct(measurement);
}

Eigen::VectorXd ekf_interface::estimate_state(double ref, double current) {
    // update delta
    dt_ = current - ref;
    return ekf_->getState();
}
