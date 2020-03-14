#include <roshell-graphics/roshell_graphics.h>

namespace roshell_graphics
{

RoshellGraphics::RoshellGraphics()
{
    // Default constructor
    int term_height_ = 80;
    int term_width_ = 120;

    get_terminal_shape();

    std::cout << "Terminal Shape: " << term_height_ << ", " << term_width_ << std::endl;
}

void RoshellGraphics::get_terminal_shape()
{
    // Do stuff
}

}  // namespace roshell_graphics