#pragma once

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

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

    int term_height_;
    int term_width_;
    Eigen::MatrixXf buffer_;
};

RoshellGraphics::RoshellGraphics()
{
    // Default constructor
    term_height_ = 80;
    term_width_ = 120;

    get_terminal_shape();

    std::cout << "Terminal Shape: " << term_height_ << ", " << term_width_ << std::endl;
}

void RoshellGraphics::get_terminal_shape()
{
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

    term_height_ = w.ws_col;
    term_width_ = w.ws_row;
}


}  // namespace roshell_graphics