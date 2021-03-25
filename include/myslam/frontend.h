#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/frame.h"

namespace myslam{

class Backend;
class Viewer;

// different status of frontend
enum class FrontendStatus {
    INITING,            // initialize map
    TRACKING_GOOD,      // tracking
    TRACKING_BAD,       // 
    LOST                // fail to track, re-initialize
};

/*
 * Frontend odometry that track pose of each frame
 * Insert keyframe to map
 */
class Frontend{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    /**
     * @brief Feeding frame into frontend for processing (computing pose and map points etc.)
     * @param frame, frame to be processed
     * @return true if success
     */
    bool AddFrame(Frame::Ptr frame);

    /** 
     * @brief Set map
     */
    void SetMap(Map::Ptr map){ map_ = map; }

    /** 
     * @brief Set backend
     */
    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    /** 
     * @brief Set viewer
     */
    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    /** 
     * @brief Get frontend status
     */
    FrontendStatus GetStatus() const { return status_; }

    /** 
     * @brief Set stereo camera parameter
     */
    void SetCameras(Camera::Ptr left, Camera::Ptr right){
        camera_left_ = left;
        camera_right_ = right;
    }

private:

    /**
     * @brief Track in normal mode
     * @return true if success
     **/
    bool Track();

    /**
     * @brief Reset when lost
     * @return true if success
    */
    bool Reset();

    /** 
     * @brief Track with last frame
     * @param 
     * @return num of tracked points
     */
    int TrackLastFrame();

    /** 
     * @brief Estimate current frame's pose
     * @param 
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /** 
     * @brief Set current frame as keyframe and insert into backend
     * @param 
     * @return true if success
     */
    bool InsertKeyframe();

    /** 
     * @brief Try init the frontend with stereo images saved in current_frame_
     * @param 
     * @return true if success
     */
    bool StereoInit();

    /** 
     * @brief Detect features in left image of current_frame_
     * key points saved in frame object
     * @param 
     * @return num of features ?
     */
    int DetectFeatures();

    /** 
     * @brief Find the corresponding features in right image of current_frame_
     * @param 
     * @return num of features found in right
     */
    int FindFeaturesInRight();

    /** 
     * @brief Build the initial map with single frame
     * @param 
     * @return true if success
     */
    bool BuildInitMap();

    /** 
     * @brief Triangulate the 2D key points in current_frame_
     * @param 
     * @return num of triangulated points
     */
    int TriangulateNewPoints();

    /** 
     * @brief Set the features in keyframe as new observation of the map points
     * @param 
     * @return
     */
    void SetObservationsForKeyframe();

    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;            // frame to be tracked
    Frame::Ptr last_frame_ = nullptr;               // previous frame for flow computing

    Camera::Ptr camera_left_ = nullptr;             
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;                        // pointer to map
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;                           // RT from last frame to current frame

    int tracking_inliers_ = 0;                      // num of inliers, used for testing new keyframes

    // Frontend params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    cv::Ptr<cv::GFTTDetector> gftt_;                // feature detector in opencv

};

}
#endif 