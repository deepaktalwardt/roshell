#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace roshell_graphics
{

class RoshellGraphics
{

public:
    RoshellGraphics();
    void get_terminal_shape();
    void draw();

private:
    uint8_t rgb_to_byte_(const std::vector<int>& rgb_color);

    Eigen::MatrixXf buffer_;
};

}  // namespace roshell_graphics