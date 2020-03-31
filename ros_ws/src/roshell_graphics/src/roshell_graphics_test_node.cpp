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

int main(int argc, char** argv)
{
    roshell_graphics::RoshellGraphics rg;
    std::pair<int, int> term_size = rg.get_terminal_size();
    draw_random_lines(rg, 10, term_size.first, term_size.second, 5e5);

    Eigen::Vector3i cam_loc(10, 10, 10);
    roshell_graphics::Camera cam;
    cam.location = cam_loc;

    roshell_graphics::Transform tf(cam_loc);

    return 0;
}
