#pragma once
#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{

// Pinhole Stereo Camera Model
class Camera{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    // Camera Intrinsics
    double  fx_ = 0,
            fy_ = 0,
            cx_ = 0,
            cy_ = 0,
            baseline_ = 0;

    // Camera Extrinsics, from left/right camera to stereo centric
    SE3 pose_;
    SE3 pose_inv_;

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();
    }

    // Get camera extrinsics
    SE3 Pose() const {
        return pose_;
    }

    // Get intrinsics matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 0, 1;
        return k;
    }

    // Coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);       // p_camera = camera_extrinsics * T_c_w * p_world
    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);
    
    Vec2 camera2pixel(const Vec3 &p_c);
    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);
    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w); 

};


} // namespace maslam

#endif