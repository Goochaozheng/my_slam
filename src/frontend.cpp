#include "myslam/frontend.h"
#include "myslam/config.h"
#include "myslam/g2o_types.h"
#include "myslam/algorithm.h"

#include <opencv2/opencv.hpp>


namespace myslam{

Frontend::Frontend(){
    gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(Frame::Ptr frame){
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}


bool Frontend::Track(){
    if(last_frame_){
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if(tracking_inliers_ > num_features_tracking_){
        // Tacking good
        status_ = FrontendStatus::TRACKING_GOOD;
    }else if(tracking_inliers_ > num_features_tracking_bad_){
        // Tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    }else{
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    // VIEWER
    // if(viewer_) 
    return true;

}


int Frontend::EstimateCurrentPose() {
    // Setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()
        )
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //vertex
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    Mat33 K = camera_left_->K();

    // Edge
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for(size_t i = 0; i < current_frame_->feature_left_.size(); ++i){
        auto mp = current_frame_->feature_left_[i]->map_point_.lock();
        if(mp){
            features.push_back(current_frame_->feature_left_[i]);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(toVec2(current_frame_->feature_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // Estimate pose
    const double chi2_th = 5.991;       // ?
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; iteration++)
    {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // Count Outliers
        for (size_t i = 0; i < edges.size(); i++)
        {
            auto e = edges[i];
            if(features[i]->is_outlier_){     
                e->computeError();
            }

            if(e->chi2() > chi2_th){
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            }

            if(iteration == 2){
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose_estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;

    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features)
    {
        if(feat->is_outlier_){
            feat->map_point_.reset();
            feat->is_outlier_ = false;      // features can be used in future
        }
    }

    return features.size() - cnt_outlier;
    

}


}