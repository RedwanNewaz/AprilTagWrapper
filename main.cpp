#include <iostream>
#include "robot_localization/ekf.h"
#include "april_tag_wrapper/april_tag_wrapper.h"
#include "robot_localization/filter_interface.h"

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


int main() {


    // Initialize camera
    VideoCapture cap(2);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }
    const int num_tags = 4;
    const int tag_offset = 20;

    filter_interface filters[num_tags];

    Mat frame, gray;
    april_tag_wrapper decoder;
    auto t1 = std::chrono::high_resolution_clock::now();
    double ref_time, dt;
    while (true)
    {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        auto detections = decoder(gray);
        // Draw detection outlines
        for (auto &  det: detections) {
            auto t2 = std::chrono::high_resolution_clock::now();
            auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            ref_time = elapse/ 1000.0;
            // overlay detection on frame
            overlay_detections(frame, det);
            // get transformation matrix
            auto pose =  decoder.get_pose(det);
            // update specific filter
            int index = det->id - tag_offset;
            // update corresponding filter
            filters[index].measurement_update(pose);

        }

        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        dt = elapse/ 1000.0;

        for (int i = 0; i < num_tags; ++i) {
            auto predict = filters[i].estimate_state(ref_time, dt);
            cout << i <<" > [prediction] \n " << predict << endl;
        }

        cv::imshow("frame", frame);
        // press ESC for quit
        if (waitKey(1) == 27)
            break;
    }

    return 0;
}
