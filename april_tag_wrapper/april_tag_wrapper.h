//
// Created by redwan on 7/20/21.
//

#ifndef GENERICFILTERS_APRIL_TAG_WRAPPER_H
#define GENERICFILTERS_APRIL_TAG_WRAPPER_H

#include <opencv2/core.hpp>
#include "apriltag.h"
#include "tag36h11.h"
#include <Eigen/Dense>
#include "common/homography.h"
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace cv;
using namespace std;

class april_tag_wrapper {
public:
    april_tag_wrapper();
    vector<apriltag_detection_t*> operator()(const cv::Mat & frame);
    Eigen::Matrix4d get_world_coords(apriltag_detection_t * detection);
    Eigen::VectorXd get_pose(apriltag_detection_t * detection);
    virtual ~ april_tag_wrapper();

private:
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    float fx, fy, cx, cy;
    float tag_size;


protected:
    void addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints) const;

    void addImagePoints (apriltag_detection_t *detection, std::vector<cv::Point2d >& imagePoint) const;

    [[nodiscard]] Eigen::Matrix4d getRelativeTransform(
            std::vector<cv::Point3d > objectPoints,
            std::vector<cv::Point2d > imagePoints,
            double fx, double fy, double cx, double cy) const;

    Eigen::Matrix4d transform_world_coords(float tag_size, float fx, float fy, float cx, float cy,  apriltag_detection_t * detection) const;


};


#endif //GENERICFILTERS_APRIL_TAG_WRAPPER_H
