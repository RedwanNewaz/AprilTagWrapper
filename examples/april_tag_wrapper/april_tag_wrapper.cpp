//
// Created by redwan on 7/20/21.
//

#include "april_tag_wrapper.h"

april_tag_wrapper::april_tag_wrapper() {

// TODO read parameters from yaml file
    // parameters for transforming tag to world coordinate
    tag_size = 0.50; // m
    // camera intrinsic parameters

    fx = 1317.1810580277324;
    fy = 1311.5926572264152;
    cx = 959.4999999687046;
    cy = 539.5000001152366;
    // initialize april tag detector
    td = apriltag_detector_create();
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 1;
    td->debug = true;
    td->nthreads = 8;



    // Initialize tag detector
    tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

}

vector<apriltag_detection_t*> april_tag_wrapper::detect(const Mat &gray) {

    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(td, &im);
    // get detection outlines
    vector<apriltag_detection_t*> detected_tags;
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        detected_tags.push_back(det);
    }

    return detected_tags;
}

april_tag_wrapper::~april_tag_wrapper() {
    delete td, tf;
}

Eigen::Matrix4d april_tag_wrapper::get_world_coords(apriltag_detection_t * detection) {
    return transform_world_coords(tag_size, fx, fy, cx, cy, detection);
}


void april_tag_wrapper::addObjectPoints(double s, cv::Matx44d T_oi, vector<cv::Point3d> &objectPoints) const {
// Add to object point vector the tag corner coordinates in the bundle frame
    // Going counterclockwise starting from the bottom left corner
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s,-s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s,-s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s, s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s, s, 0, 1));

}

void april_tag_wrapper::addImagePoints (
        apriltag_detection_t *detection,
        std::vector<cv::Point2d >& imagePoints) const
{
    // Add to image point vector the tag corners in the image frame
    // Going counterclockwise starting from the bottom left corner
    double tag_x[4] = {-1,1,1,-1};
    double tag_y[4] = {1,1,-1,-1}; // Negated because AprilTag tag local
    // frame has y-axis pointing DOWN
    // while we use the tag local frame
    // with y-axis pointing UP
    for (int i=0; i<4; i++)
    {
        // Homography projection taking tag local frame coordinates to image pixels
        double im_x, im_y;
        homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
        imagePoints.push_back(cv::Point2d(im_x, im_y));
    }
}

Eigen::Matrix4d
april_tag_wrapper::getRelativeTransform(std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints,
                                        double fx, double fy, double cx, double cy) const {
    // perform Perspective-n-Point camera pose estimation using the
    // above 3D-2D point correspondences
    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix(fx,  0, cx,
                             0,  fy, cy,
                             0,   0,  1);
    cv::Vec4f distCoeffs(0,0,0,0); // distortion coefficients
    // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
    // need to first check WHAT is a bottleneck in this code, and only
    // do this if PnP solution is the bottleneck.
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Matx33d R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d wRo;
    wRo << R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2);

    Eigen::Matrix4d T; // homogeneous transformation matrix
    T.topLeftCorner(3, 3) = wRo;
    T.col(3).head(3) <<
                     tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0,0,0,1;
    return T;
}

Eigen::Matrix4d april_tag_wrapper::transform_world_coords(float tag_size, float fx, float fy, float cx, float cy,
                                                          apriltag_detection_t *detection) const {
    std::vector<cv::Point3d > standaloneTagObjectPoints;
    std::vector<cv::Point2d > standaloneTagImagePoints;
    addObjectPoints(tag_size/2, cv::Matx44d::eye(), standaloneTagObjectPoints);
    addImagePoints(detection, standaloneTagImagePoints);
    Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints,
                                                     standaloneTagImagePoints,
                                                     fx, fy, cx, cy);
    return transform;
}

Eigen::VectorXd april_tag_wrapper::get_pose(apriltag_detection_t * detection) {
    Eigen::VectorXd pose(6);
    auto transformation = get_world_coords(detection);

    // rotation matrix R
    Eigen::Matrix3d R;
    R << transformation.topLeftCorner(3, 3);
    // translation vector
    Eigen::VectorXd T(3);
    T << transformation(0, 3), transformation(1, 3), transformation(2, 3);
    //R.eulerAngles(0, 1, 2)  get rotx, roty, rotz
    Eigen::Vector3d ea = R.eulerAngles(0, 1, 2);
    pose << T, ea;

//    cout << "R \n" << R << "\n Transf \n" << transformation << "\n Position \n "<< T <<endl;

    return pose;
}
