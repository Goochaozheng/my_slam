#pragma once

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "myslam/common_include.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"


namespace myslam{

/*
 * Map that manage frames and map points
 * Frontend odometry can insert keyframe and map point to map
 * Backend optimizer can access keyframe and map point for optimization and update map
 * A window of active frames will be kept for optimization
 */
class Map{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    // Insert keyframe
    void InsertKeyframe(Frame::Ptr frame);

    // Insert map point
    void InsertMapPoint(MapPoint::Ptr map_point);

    // Get all map points
    LandmarksType GetAllMapPoints(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    // Get active map points
    LandmarksType GetActiveMapPoints(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    // Get all keyframes
    LandmarksType GetAllKeyframe(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    // Get active keyframes
    LandmarksType GetActiveKeyframes(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }


private:

    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;               // all map points
    LandmarksType active_landmarks_;        // active map points
    KeyframesType keyframes_;               // all keyframes
    KeyframesType active_keyframes_;        // active keyframes

    int num_active_keyframes_ = 7;          // number of active keyframes

    Frame::Ptr current_frame_ = nullptr;    // current frame

};    

}

#endif MYSLAM_MAP_H