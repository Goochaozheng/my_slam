#include "myslam/map.h"

namespace myslam{

void Map::InsertKeyframe(Frame::Ptr frame){
    current_frame_ = frame;
    
    if(keyframes_.find(current_frame_->keyframe_id_) == keyframes_.end()){
        // new key frame
        keyframes_.insert(make_pair(frame->keyframe_id_ ,frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_ ,frame));
    }else{
        // frame already in current set
        // update frame to current frame
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    // Maintain active keyframe num
    if(active_keyframes_.size() > num_active_keyframes_){
        RemoveOldKeyframe();
    }

}


void Map::InsertMapPoint(MapPoint::Ptr map_point){
    if(landmarks_.find(map_point->id_) == landmarks_.end()){
        // new map point
        landmarks_.insert(make_pair(map_point->id_, map_point));
        active_landmarks_.insert(make_pair(map_point->id_, map_point));
    }else{
        // map point already exist
        // update map point
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

void Map::CleanMap(){
    int cnt_landmark_removed = 0;
    for(auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();){
        if(iter->second->observed_times_ == 0){
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        }else{
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}

void Map::RemoveOldKeyframe(){
    if(current_frame_ == nullptr) return; // no frame added

    // find the closest & furthest frame in current active keyframes
    double max_dis = 0,
           min_dis = 9999,
           max_kf_id = 0,
           min_kf_id = 9999;

    auto Twc = current_frame_->pose_.inverse();
    for(auto& kf : active_keyframes_){ // iterate map, iter.first is key, iter.second is value
        if(kf.second == current_frame_) continue;
        //SE3 -> se3, norm of se3 as distance ?
        auto dis = (kf.second->Pose() * Twc).log().norm();  
        if(dis > max_dis){
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if(dis < min_dis){
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;      // threshold for close frame
    Frame::Ptr frame_to_remove = nullptr;
    if(min_dis < min_dis_th){
        // If the closest frame is with the threshold, then remove the close frame first
        frame_to_remove = keyframes_.at(min_kf_id);
    }else{
        // Closest frame not close enough, remove the furthest frame
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_; 
    active_keyframes_.erase(frame_to_remove->keyframe_id_);

    // Remove landmarks observation of old frame
    for(auto feat : frame_to_remove->feature_left_){
        auto mp = feat->map_point_.lock();
        if(mp){
            mp->RemoveObservation(feat);
        }
    }
    for(auto feat : frame_to_remove->feature_right_){
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if(mp){
            mp->RemoveObservation(feat);
        }
    }

    // Remove landmarks with no observation
    CleanMap();
}

}