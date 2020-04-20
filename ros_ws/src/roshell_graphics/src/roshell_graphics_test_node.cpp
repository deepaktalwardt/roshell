#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <Eigen/Dense>

#include <roshell_graphics/roshell_graphics.h>
#include <roshell_graphics/perspective_projection.h>

/**
 * Function to test line drawing capabilities
*/
void draw_random_lines(
    roshell_graphics::RoshellGraphics& rg,
    int n,
    int max_w,
    int max_h,
    unsigned long delay)
{
    srand((unsigned) time(0));
    for (int i = 0; i < n; i++)
    {
        roshell_graphics::Point p1, p2;
        p1.first = rand() % max_w;
        p2.first = rand() % max_w;

        p1.second = rand() % max_h;
        p2.second = rand() % max_h;

        rg.line(p1, p2);
        rg.draw_and_clear(delay);
    }
}

void draw_test(
    roshell_graphics::RoshellGraphics& rg,
    roshell_graphics::PerspectiveProjection& pp)
{
    Eigen::Vector3f origin_world(10, 0, 0);
    Eigen::Vector3f origin_world_in_cam_frame =
        pp.transform_world_point(origin_world);

    std::cout << "origin_world_in_cam_frame " << std::endl <<
        origin_world_in_cam_frame << std::endl;
    
    Eigen::Vector2f origin_world_in_image_frame =
        pp.project_cam_point(origin_world_in_cam_frame);
    
    std::cout << "origin_world_in_image_frame " << std::endl <<
        origin_world_in_image_frame << std::endl;
    
}

void draw_axis(
    roshell_graphics::RoshellGraphics& rg,
    roshell_graphics::PerspectiveProjection& pp)
{
    Eigen::Matrix3Xf points_in_world_frame(3, 4);

    points_in_world_frame.col(0) << 0, 0, 0;
    points_in_world_frame.col(1) << 1000, 0, 0;
    points_in_world_frame.col(2) << 0, 1000, 0;
    points_in_world_frame.col(3) << 0, 0, 1000;

    Eigen::Matrix2Xf points_in_image_frame(2, 4);
    for (int i = 0; i < 4; i++)
    {
        points_in_image_frame.col(i) =
            pp.project_world_point(points_in_world_frame.col(i));
    }

    roshell_graphics::Point p1, p2, p3, p4;

    p1.first = points_in_image_frame(0, 0);
    p1.second = points_in_image_frame(1, 0);

    p2.first = points_in_image_frame(0, 1);
    p2.second = points_in_image_frame(1, 1);

    p3.first = points_in_image_frame(0, 2);
    p3.second = points_in_image_frame(1, 2);

    p4.first = points_in_image_frame(0, 3);
    p4.second = points_in_image_frame(1, 3);

    rg.fix_frame(p1);
    rg.fix_frame(p2);
    rg.fix_frame(p3);
    rg.fix_frame(p4);

    rg.line(p1, p2);
    rg.line(p1, p3);
    rg.line(p1, p4);

    rg.draw();
}

void draw_rotating_cube(
    roshell_graphics::RoshellGraphics& rg,
    char c)
{
    // Vertices for the cube
    Eigen::Matrix3Xf points_in_world_frame(3, 8);

    // Cube length
    float cl = 300;
    points_in_world_frame.col(0) << 0,   0,   0;
    points_in_world_frame.col(1) << cl,  0,   0;
    points_in_world_frame.col(2) << cl,  cl,  0;
    points_in_world_frame.col(3) << 0 ,  cl,  0;

    points_in_world_frame.col(4) << 0 ,  0,  cl;
    points_in_world_frame.col(5) << cl,  0,  cl;
    points_in_world_frame.col(6) << cl,  cl, cl;
    points_in_world_frame.col(7) << 0 ,  cl, cl;
    
    Eigen::Matrix2Xf points_in_image_frame(2, 8);

    roshell_graphics::Camera cam;
    Eigen::Vector3f cam_loc(8000, 10000, 5000);
    cam.location = cam_loc;
    cam.focal_distance = 3000;

    roshell_graphics::PerspectiveProjection pp(cam);

    for (float angle = 0.0; angle < 3.1415; angle += 0.1)
    {   
        roshell_graphics::Camera new_cam = cam;
        float new_cam_x = cam_loc(0) * cos(angle);
        float new_cam_y = cam_loc(1) * sin(angle);

        Eigen::Vector3f new_cam_loc(new_cam_x, new_cam_y, cam_loc(2));
        new_cam.location = new_cam_loc; 

        pp.update_camera(new_cam);

        for (int i = 0; i < 8; i++)
        {
            points_in_image_frame.col(i) =
                pp.project_world_point(points_in_world_frame.col(i));
        }

        roshell_graphics::Point p1, p2, p3, p4, p5, p6, p7, p8;
        p1.first = points_in_image_frame(0, 0);
        p1.second = points_in_image_frame(1, 0);

        p2.first = points_in_image_frame(0, 1);
        p2.second = points_in_image_frame(1, 1);

        p3.first = points_in_image_frame(0, 2);
        p3.second = points_in_image_frame(1, 2);

        p4.first = points_in_image_frame(0, 3);
        p4.second = points_in_image_frame(1, 3);

        p5.first = points_in_image_frame(0, 4);
        p5.second = points_in_image_frame(1, 4);

        p6.first = points_in_image_frame(0, 5);
        p6.second = points_in_image_frame(1, 5);

        p7.first = points_in_image_frame(0, 6);
        p7.second = points_in_image_frame(1, 6);

        p8.first = points_in_image_frame(0, 7);
        p8.second = points_in_image_frame(1, 7);

        rg.fix_frame(p1);
        rg.fix_frame(p2);
        rg.fix_frame(p3);
        rg.fix_frame(p4);
        rg.fix_frame(p5);
        rg.fix_frame(p6);
        rg.fix_frame(p7);
        rg.fix_frame(p8);

        // draw_axis(rg, pp);

        rg.line(p1, p2, '&'); // example
        rg.line(p2, p3);
        rg.line(p3, p4);
        rg.line(p4, p1);

        rg.line(p5, p6, '&');
        rg.line(p6, p7);
        rg.line(p7, p8);
        rg.line(p8, p5);

        rg.line(p1, p5, '&');
        rg.line(p2, p6);
        rg.line(p3, p7);
        rg.line(p4, p8);

        rg.draw_and_clear(10e4);
    }
}

int main(int argc, char** argv)
{
    roshell_graphics::RoshellGraphics rg;
    std::pair<int, int> term_size = rg.get_terminal_size();
    // draw_random_lines(rg, 10, term_size.first, term_size.second, 5e5);

    draw_rotating_cube(rg, '/');

    return 0;
}
