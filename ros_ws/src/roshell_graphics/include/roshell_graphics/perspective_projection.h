#pragma once

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "math.h"

#include "roshell_graphics.h"

namespace roshell_graphics
{

#define PI            3.14159265
#define RAD_TO_DEG    PI/180.0
#define DEG_TO_RAD    180.0/PI

/****************
 * Camera struct
 ****************/
struct Camera
{
    Eigen::Vector3f location;     // as defined in the world frame
    int focal_distance;   // in meters
};

/******************
 * Transform Class
 ******************/
class Transform
{
    public:
        // Assuming Z-axis points to world origin
        Transform(
            const Camera& cam);

        Transform(
            const Eigen::Vector3f& origin);
        
        ~Transform();

        Eigen::Matrix3f get_rotation_matrix() const;
        Eigen::Vector3f get_origin() const;
        Eigen::Matrix4f get_transformation_matrix() const;

        Eigen::Matrix3f angles_to_rotation_matrix(
            const float& theta, 
            const float& phi);
        
        void update(const Eigen::Vector3f& origin);

    private:
        Eigen::Vector3f origin_;
        Eigen::Matrix3f rotation_matrix_;

        float rho_;
        Eigen::Matrix4f T_;

};  // Transform class

Transform::Transform(const Camera& cam)
{
    update(cam.location);
}

Transform::Transform(const Eigen::Vector3f& origin)
{
    update(origin);
}

void Transform::update(const Eigen::Vector3f& origin)
{
    origin_ = origin;

    rho_ = sqrt(pow(origin_(0), 2) + pow(origin_(1), 2) + pow(origin_(2), 2));

    float phi = acos(origin_(2) / rho_);
    float theta = asin(origin_(1) / sqrt(pow(origin_(0), 2) + pow(origin_(1), 2))); 

    rotation_matrix_ = angles_to_rotation_matrix(theta, phi);

    T_.block<3, 3>(0, 0) = rotation_matrix_;

    Eigen::Vector3f last_col(0, 0, rho_);
    Eigen::Vector4f last_row(0, 0, 0, 1);

    T_.block<3, 1>(0, 3) = last_col;
    T_.block<1, 4>(3, 0) = last_row;
}

Transform::~Transform()
{
}

Eigen::Matrix3f Transform::angles_to_rotation_matrix(
    const float& theta,
    const float& phi)
{   
    Eigen::Matrix3f rot_mat;
    rot_mat << -sin(theta),             cos(theta),                 0,
               -cos(phi)*cos(theta),  -cos(phi)*sin(theta),     sin(phi),
               -sin(phi)*cos(theta),  -sin(phi)*cos(theta),     -cos(phi);

    return rot_mat;
}

Eigen::Vector3f Transform::get_origin() const
{
    return origin_;
}

Eigen::Matrix3f Transform::get_rotation_matrix() const
{
    return rotation_matrix_;
}

Eigen::Matrix4f Transform::get_transformation_matrix() const
{
    return T_;
}

/******************************
 * PerspectiveProjection Class
 ******************************/
class PerspectiveProjection
{
    public:
        PerspectiveProjection(const Camera& camera);
        ~PerspectiveProjection();

        void update_camera(const Camera& camera);

        Eigen::Vector3f transform_world_point(const Eigen::Vector3f& point_in_world_frame);
        Eigen::Vector2f project_world_point(const Eigen::Vector3f& point_in_world_frame);
        Eigen::Vector2f project_cam_point(const Eigen::Vector3f& point_in_cam_frame);

    private:
        Camera camera_;
        Transform tf_;

};  // class PerspectiveProjection

PerspectiveProjection::PerspectiveProjection(const Camera& camera):
    camera_(camera),
    tf_(camera)
{
}

PerspectiveProjection::~PerspectiveProjection()
{
}

Eigen::Vector3f PerspectiveProjection::transform_world_point(
    const Eigen::Vector3f& point_in_world_frame)
{
    Eigen::Vector4f point_in_aug, point_out_aug;
    point_in_aug << point_in_world_frame, 1;
    point_out_aug = tf_.get_transformation_matrix() * point_in_aug;
    
    Eigen::Vector3f point_out_in_cam_frame = point_out_aug.block<3, 1>(0, 0);
    return point_out_in_cam_frame;
}

Eigen::Vector2f PerspectiveProjection::project_world_point(
    const Eigen::Vector3f& point_in_world_frame)
{
    Eigen::Vector3f point_in_cam_frame = transform_world_point(point_in_world_frame);
    return project_cam_point(point_in_cam_frame);
}

Eigen::Vector2f PerspectiveProjection::project_cam_point(
    const Eigen::Vector3f& point_in_cam_frame)
{
    Eigen::Vector2f point_in_image_plane;
    point_in_image_plane(0) = point_in_cam_frame(0) * (camera_.focal_distance / point_in_cam_frame(2));
    point_in_image_plane(1) = point_in_cam_frame(1) * (camera_.focal_distance / point_in_cam_frame(2));

    // Fix for lower vertical resolution
    point_in_image_plane(1) = 0.5 * point_in_image_plane(1);

    return point_in_image_plane;
}

void PerspectiveProjection::update_camera(const Camera& camera)
{
    tf_.update(camera.location);
}

}  // namespace roshell_graphics

