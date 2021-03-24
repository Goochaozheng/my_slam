#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

// #include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam{

struct Feature;
struct MapPoint;

/*
 * one pair of stereo image with unique id & timestamp
 * pose & keyframe is managed by each frame
 */
struct Frame{
public:
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;    // overload operator new for fixed size vectorizable eigen type
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;              // id of this frame
    unsigned long keyframe_id_ = 0;     // id of keyframe
    bool is_keyframe_ = false;          // is keyframe or not
    double time_stamp_;                 // timestamp
    SE3 pose_;                          // pose, in Tcw?
    std::mutex pose_mutex_;             // mutex lock for pose
    cv::Mat left_img_, right_img_;      // stereo image

    std::vector<std::shared_ptr<Feature>> feature_left_;    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> feature_right_;   // corresponding features in right image, set to nullptr if no matches

    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

    // get pose of this frame
    SE3 Pose(){
        std::unique_lock<std::mutex> lck(pose_mutex_); // take the ownership of mutex object and unlock automatically when unique_lock object destoried
        return pose_;
    }

    // set pose
    void SetPose(const SE3 &pose){
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    void SetKeyFrame();

    static Frame::Ptr CreateFrame();

};

}

#endif // MYSLAM_FRAME_H