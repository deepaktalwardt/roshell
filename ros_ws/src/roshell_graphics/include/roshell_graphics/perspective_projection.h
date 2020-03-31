#pragma once

#include <iostream>
#include <string>
#include <Eigen/Dense>

#include "roshell_graphics.h"

namespace roshell_graphics
{

using Point3D = Eigen::Vector3i;

typedef struct Camera
{
    Point3D location;     // as defined in the world frame
    int focal_distance;   // in meters
};

class Transform
{
    public:
        // Assuming Z-axis points to world origin
        Transform(Eigen::Vector3f origin);

        Transform(Eigen::Vector3f origin, Eigen::Matrix3Xf rotation_matrix);
        Transform(Eigen::Vector3f origin, float roll, float pitch, float yaw);

        Eigen::Matrix3Xf get_rotation_matrix() const;
        Eigen::Vector3f get_origin() const;

        Eigen::Matrix3Xf convert_angles_to_rotation_matrix(float roll, float pitch, float yaw);

    private:
        Eigen::Vector3f origin_;
        Eigen::Matrix3Xf rotation_matrix_;

};

Transform::Transform(Eigen::Vector3f origin, Eigen::Matrix3Xf rotation_matrix):
    origin_(origin),
    rotation_matrix_(rotation_matrix)
{
}

Transform::Transform(Eigen::Vector3f origin, float roll, float pitch, float yaw):
    origin_(origin)
{
    rotation_matrix_ = convert_angles_to_rotation_matrix(roll, pitch, yaw);
}

Eigen::Matrix3Xf Transform::convert_angles_to_rotation_matrix(float roll, float pitch, float yaw)
{   
    Eigen::Matrix3Xf rot_mat;
    rot_mat << 1, 0, 0,
               0, 1, 0,
               0, 0, 1;
    return rot_mat;
}

Eigen::Vector3f Transform::get_origin() const
{
    return origin_;
}

Eigen::Matrix3Xf Transform::get_rotation_matrix() const
{
    return rotation_matrix_;
}


class PerspectiveProjection
{
    public:
        PerspectiveProjection(Camera camera, Transform tf);
        ~PerspectiveProjection();

        void update_camera(Camera camera);

        Point3D transform_coordinate(const Point3D& point);
        Point3D transform_coordinate(const Point3D& point, const Transform& tf);

    private:
        Camera camera_;
        Transform tf_;
};  // class PerspectiveProjection

}  // namespace roshell_graphics

