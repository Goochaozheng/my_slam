#pragma once
#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include "myslam/common_include.h"
#include <opencv2/features2d.hpp>

namespace myslam{

struct Frame;
struct MapPoint;

/*
 * 2D feature points on image
 * one feature point will be mapped to a 3D map point
 */

struct Feature{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;        // which frame does the feature locates, access frame from feature, but feature has no ownship of frame (weak_ptr)
    cv::KeyPoint position_;             // feature point location
    std::weak_ptr<MapPoint> map_point_; // corresponding 3D map point, access map point from feature, but feature has no ownship of map point (weak_ptr)

    bool is_outlier_ = false;           // whether is outlier
    bool is_on_left_image_ = true;      // which image does the feature locates


    Feature() {}

    Feature(std::weak_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}

};
    

}

#endif // MYSLAM_FEATURE_H