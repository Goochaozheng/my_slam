#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/feature.h"
#include "myslam/mappoint.h"

namespace myslam{


/*
 * one pair of stereo image with unique id & timestamp
 * one frame can be set as keyframe, which will be added to map
 * pose & keyframe is owned by each frame
 */
struct Frame{
public:
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;    // overload operator new for fixed size vectorizable eigen type
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;              // id of this frame
    unsigned long keyframe_id_ = 0;     // id of keyframe
    bool is_keyframe_ = false;          // is keyframe or not
    double time_stamp_;                 // timestamp
    SE3 pose_;                          // pose, in Tcw, p_frame = Tcw * p_world_origin
    std::mutex pose_mutex_;             // mutex lock for pose
    cv::Mat left_img_, right_img_;      // stereo image

    std::vector<Feature::Ptr> feature_left_;    // extracted features in left image
    std::vector<Feature::Ptr> feature_right_;   // corresponding features in right image, set to nullptr if no matches

    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

    // get pose under world coordinate of this frame
    SE3 Pose(){
        std::unique_lock<std::mutex> lck(pose_mutex_); // take the ownership of mutex object and unlock automatically when unique_lock object destoried
        return pose_;
    }

    // set pose
    void SetPose(const SE3 &pose){
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    // Set current frame to keyframe and assign unique keyframe id
    void SetKeyFrame();

    // Create frame and assign unique frame id using a factory method
    static Frame::Ptr CreateFrame();

};

}

#endif // MYSLAM_FRAME_H