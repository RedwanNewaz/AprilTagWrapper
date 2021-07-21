#include <iostream>
#include "robot_localization/ekf.h"
#include "april_tag_wrapper/april_tag_wrapper.h"
#include "robot_localization/ekf_interface.h"

using namespace cv;
using namespace std;
using namespace RobotLocalization;


void overlay_detections(cv::Mat& frame, const apriltag_detection_t* det)
{
    line(frame, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[1][0], det->p[1][1]),
         Scalar(0, 0xff, 0), 2);
    line(frame, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[3][0], det->p[3][1]),
         Scalar(0, 0, 0xff), 2);
    line(frame, Point(det->p[1][0], det->p[1][1]),
         Point(det->p[2][0], det->p[2][1]),
         Scalar(0xff, 0, 0), 2);
    line(frame, Point(det->p[2][0], det->p[2][1]),
         Point(det->p[3][0], det->p[3][1]),
         Scalar(0xff, 0, 0), 2);

    stringstream ss;
    ss << det->id;
    String text = ss.str();
    int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 1.0;
    int baseline;
    Size textsize = getTextSize(text, fontface, fontscale, 2,
                                &baseline);
    putText(frame, text, Point(det->c[0]-textsize.width/2,
                               det->c[1]+textsize.height/2),
            fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

}


int main(int argc, char *argv[]) {

    assert(argc > 1 && "config param file is missing");


    const string config_param = argv[1];
    //Read parameters from yaml file
    YAML::Node config = YAML::LoadFile(config_param);
    std::cout << "Parsed YAML:\n" << config << std::endl;

    auto cameraIndex = config["Camera"]["index"].as<int>();

    // Initialize camera
    VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }
    const int num_tags = config["AprilTag"]["numTags"].as<int>();
    const int tag_offset = config["AprilTag"]["tag_start"].as<int>();

    // TODO send a vector of process and measurement noises for generating their corresponding covariances
    auto processNoise = config["EKF"]["processNoise"].as<double>();
    auto measurementNoise = config["EKF"]["measurementNoise"].as<double>();
    auto mahalanobisThresh = config["EKF"]["mahalanobisThresh"].as<double>();

    // extended kalman filter initialize for each tag
    vector<unique_ptr<ekf_interface>>filters;
    for (int i = 0; i < num_tags; ++i) {
        filters.emplace_back(std::make_unique<ekf_interface>(processNoise, measurementNoise, mahalanobisThresh));
    }


    Mat frame, gray;
    april_tag_wrapper decoder;

    double ref_time, dt;
    while (true)
    {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        auto detections = decoder.detect(gray);
        // Draw detection outlines
        for (auto &  det: detections) {
            ref_time = filters[0]->get_ms()/ 1000.0;
            // overlay detection on frame
            overlay_detections(frame, det);
            // get transformation matrix
            auto pose =  decoder.get_pose(det);
            // update specific filter
            int index = det->id - tag_offset;
            // update corresponding filter
            filters[index]->measurement_update(pose);

        }

        dt = filters[0]->get_ms()/ 1000.0;

        for (int i = 0; i < num_tags; ++i) {
            auto predict = filters[i]->estimate_state(ref_time, dt);
            cout << i <<" > [prediction] \n " << predict << endl;
        }

        cv::imshow("frame", frame);
        // press ESC for quit
        if (waitKey(1) == 27)
            break;
    }



    return 0;
}
