#pragma once

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam{

struct Frame;
struct Feature;

/*
 * 3D map point on the map
 * triangulated feature point
 * each with unique id
 * different feature points might be mapped to the same map point ?
 */
struct MapPoint{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;                                  // id of map point
    bool is_outlier_ = false;                               // is outlier or not
    Vec3 pos_ = Vec3::Zero();                               // 3D position in world coordinate
    std::mutex data_mutex_;
    int observed_times_ = 0;                                // times being observed by feature match algorithm
    std::list<std::weak_ptr<Feature>> observations_;        // access feature points from map point

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    // get map point position
    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    // set position
    void SetPos(const Vec3 &pos){
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    }

    // 
    void AddObservation(std::shared_ptr<Feature> feature){
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<Feature> feature);

    std::list<std::weak_ptr<Feature>> GetObs(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    static MapPoint::Ptr CreatNewMapPoint();

};
    

}

#endif