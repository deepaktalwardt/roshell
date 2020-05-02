#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "math.h"

#include "roshell_graphics.h"

namespace roshell_graphics
{

#define PI            3.14159265
#define RAD_TO_DEG    PI/180.0
#define DEG_TO_RAD    180.0/PI

// unsigned char turbo_srgb_bytes[256][3] = {{48,18,59},{50,21,67},{51,24,74},{52,27,81},{53,30,88},{54,33,95},{55,36,102},{56,39,109},{57,42,115},{58,45,121},{59,47,128},{60,50,134},{61,53,139},{62,56,145},{63,59,151},{63,62,156},{64,64,162},{65,67,167},{65,70,172},{66,73,177},{66,75,181},{67,78,186},{68,81,191},{68,84,195},{68,86,199},{69,89,203},{69,92,207},{69,94,211},{70,97,214},{70,100,218},{70,102,221},{70,105,224},{70,107,227},{71,110,230},{71,113,233},{71,115,235},{71,118,238},{71,120,240},{71,123,242},{70,125,244},{70,128,246},{70,130,248},{70,133,250},{70,135,251},{69,138,252},{69,140,253},{68,143,254},{67,145,254},{66,148,255},{65,150,255},{64,153,255},{62,155,254},{61,158,254},{59,160,253},{58,163,252},{56,165,251},{55,168,250},{53,171,248},{51,173,247},{49,175,245},{47,178,244},{46,180,242},{44,183,240},{42,185,238},{40,188,235},{39,190,233},{37,192,231},{35,195,228},{34,197,226},{32,199,223},{31,201,221},{30,203,218},{28,205,216},{27,208,213},{26,210,210},{26,212,208},{25,213,205},{24,215,202},{24,217,200},{24,219,197},{24,221,194},{24,222,192},{24,224,189},{25,226,187},{25,227,185},{26,228,182},{28,230,180},{29,231,178},{31,233,175},{32,234,172},{34,235,170},{37,236,167},{39,238,164},{42,239,161},{44,240,158},{47,241,155},{50,242,152},{53,243,148},{56,244,145},{60,245,142},{63,246,138},{67,247,135},{70,248,132},{74,248,128},{78,249,125},{82,250,122},{85,250,118},{89,251,115},{93,252,111},{97,252,108},{101,253,105},{105,253,102},{109,254,98},{113,254,95},{117,254,92},{121,254,89},{125,255,86},{128,255,83},{132,255,81},{136,255,78},{139,255,75},{143,255,73},{146,255,71},{150,254,68},{153,254,66},{156,254,64},{159,253,63},{161,253,61},{164,252,60},{167,252,58},{169,251,57},{172,251,56},{175,250,55},{177,249,54},{180,248,54},{183,247,53},{185,246,53},{188,245,52},{190,244,52},{193,243,52},{195,241,52},{198,240,52},{200,239,52},{203,237,52},{205,236,52},{208,234,52},{210,233,53},{212,231,53},{215,229,53},{217,228,54},{219,226,54},{221,224,55},{223,223,55},{225,221,55},{227,219,56},{229,217,56},{231,215,57},{233,213,57},{235,211,57},{236,209,58},{238,207,58},{239,205,58},{241,203,58},{242,201,58},{244,199,58},{245,197,58},{246,195,58},{247,193,58},{248,190,57},{249,188,57},{250,186,57},{251,184,56},{251,182,55},{252,179,54},{252,177,54},{253,174,53},{253,172,52},{254,169,51},{254,167,50},{254,164,49},{254,161,48},{254,158,47},{254,155,45},{254,153,44},{254,150,43},{254,147,42},{254,144,41},{253,141,39},{253,138,38},{252,135,37},{252,132,35},{251,129,34},{251,126,33},{250,123,31},{249,120,30},{249,117,29},{248,114,28},{247,111,26},{246,108,25},{245,105,24},{244,102,23},{243,99,21},{242,96,20},{241,93,19},{240,91,18},{239,88,17},{237,85,16},{236,83,15},{235,80,14},{234,78,13},{232,75,12},{231,73,12},{229,71,11},{228,69,10},{226,67,10},{225,65,9},{223,63,8},{221,61,8},{220,59,7},{218,57,7},{216,55,6},{214,53,6},{212,51,5},{210,49,5},{208,47,5},{206,45,4},{204,43,4},{202,42,4},{200,40,3},{197,38,3},{195,37,3},{193,35,2},{190,33,2},{188,32,2},{185,30,2},{183,29,2},{180,27,1},{178,26,1},{175,24,1},{172,23,1},{169,22,1},{167,20,1},{164,19,1},{161,18,1},{158,16,1},{155,15,1},{152,14,1},{149,13,1},{146,11,1},{142,10,1},{139,9,2},{136,8,2},{133,7,2},{129,6,2},{126,5,2},{122,4,3}};

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

        Eigen::Matrix2Xf  project_multiple_world_points(
            const Eigen::Matrix3Xf& points_in_world_frame);
        
        Eigen::Matrix3Xf project_multiple_world_points_with_z_world(
            const Eigen::Matrix3Xf& points_in_world_frame);
        
        void transform_multiple_world_points(
            const Eigen::Matrix3Xf& points_in_world_frame, /** input */
            Eigen::Matrix3Xf& points_in_cam_frame);  /** output */
        
        void project_multiple_cam_points(
            const Eigen::Matrix3Xf& points_in_cam_frame,
            Eigen::Matrix2Xf& points_in_image_plane);

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

/**
 * This function projects multiple_world_points onto the image_plane
*/
Eigen::Matrix2Xf PerspectiveProjection::project_multiple_world_points(
    const Eigen::Matrix3Xf& points_in_world_frame)
{
    int num_points = points_in_world_frame.cols();
    Eigen::Matrix3Xf points_in_cam_frame(3, num_points);
    Eigen::Matrix2Xf points_in_image_plane(2, num_points);

    transform_multiple_world_points(points_in_world_frame, points_in_cam_frame);
    project_multiple_cam_points(points_in_cam_frame, points_in_image_plane);

    return points_in_image_plane;
}

/**
 * Overloaded function that appends the z-coordinate in the world frame in the third row
*/
Eigen::Matrix3Xf PerspectiveProjection::project_multiple_world_points_with_z_world(
    const Eigen::Matrix3Xf& points_in_world_frame)
{
    int num_points = points_in_world_frame.cols();

    Eigen::Matrix2Xf points_in_image_plane = project_multiple_world_points(points_in_world_frame);
    Eigen::Matrix3Xf points_in_image_plane_with_z_world(3, num_points);

    points_in_image_plane_with_z_world.block(0, 0, 2, num_points) = points_in_image_plane;
    points_in_image_plane_with_z_world.block(2, 0, 1, num_points) = points_in_world_frame.block(2, 0, 1, num_points);

    return points_in_image_plane_with_z_world;
}

/**
 * This function takes in multiple points_in_world_frame and transforms them into camera frame
 * and populates the points_in_cam_frame matrix
*/
void PerspectiveProjection::transform_multiple_world_points(
    const Eigen::Matrix3Xf& points_in_world_frame, /** input */
    Eigen::Matrix3Xf& points_in_cam_frame)   /** output */
{
    int num_points = points_in_world_frame.cols();

    Eigen::Matrix4Xf points_in_world_frame_aug(4, num_points);

    points_in_world_frame_aug.topRows(3) = points_in_world_frame;
    points_in_world_frame_aug.row(3) = Eigen::RowVectorXf::Ones(num_points);

    Eigen::Matrix4Xf points_in_cam_frame_aug = tf_.get_transformation_matrix() * points_in_world_frame_aug;

    points_in_cam_frame = points_in_cam_frame_aug.topRows(3);
}

/**
 * This function takes multiple 3D points_in_cam_frame and projects them into
 * the 2D image plane and populates those points in points_in_image_plane
*/
void PerspectiveProjection::project_multiple_cam_points(
    const Eigen::Matrix3Xf& points_in_cam_frame,
    Eigen::Matrix2Xf& points_in_image_plane)
{
    int num_points = points_in_cam_frame.cols();
    for (int i = 0; i < num_points; i++)
    {
        points_in_image_plane.col(i) = project_cam_point(points_in_cam_frame.col(i));
    }
}

/**
 * This function updates the Transform object's camera object with the input camera
*/
void PerspectiveProjection::update_camera(const Camera& camera)
{
    tf_.update(camera.location);
}

}  // namespace roshell_graphics

