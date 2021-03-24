#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"

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


};


#endif