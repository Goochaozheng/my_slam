#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam{

MapPoint::Ptr MapPoint::CreatNewMapPoint(){
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

void MapPoint::RemoveObservation(Feature::Ptr feature){
    std::unique_lock<std::mutex> lck(data_mutex_);
    for(auto iter = observations_.begin(); iter != observations_.end(); iter++){
        if(iter->lock() == feature){ // weak_ptr::lock, get the ownership of pointer
            observations_.erase(iter);
            feature->map_point_.reset();
            observed_times_--;
            break;
        }
    }
}

}