#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>

namespace roshell_graphics
{

using Point = std::pair<int, int>;

class RoshellGraphics
{

public:
    // Constructors and Destructors
    RoshellGraphics();
    ~RoshellGraphics();

    // Buffer related functions
    void update_buffer();
    void clear_buffer();

    // Terminal related functions
    std::pair<int, int> get_terminal_size();

    // Drawing related functions
    std::vector<int> line(const Point& pp1, const Point& pp2);
    
    void draw();
    void draw_and_clear(unsigned long delay);
    void fix_frame(Point& p);
    void draw_frame();

private:
    // Utility functions
    uint8_t rgb_to_byte_(const std::vector<int>& rgb_color);
    int encode_point_(const Point& p);
    Point decode_index_(int index);
    void put_within_limits_(Point& p);

    // Class variables
    int term_height_;
    int term_width_;

    const char* term_type_;
    const char* term_color_;
    
    std::string buffer_;
};

/**
 * Constructor
 */
RoshellGraphics::RoshellGraphics()
{   
    // Defaults
    term_height_ = 40; 
    term_width_ = 150;

    update_buffer();

    std::cout << "Terminal Shape (w, h): " << term_width_ << ", " << term_height_ << std::endl;

    term_type_ = std::getenv("TERM");
    term_color_ = std::getenv("COLORTERM");

    // TODO(deepak): Use these to add color to the points
    std::cout << "Term Type: " << term_type_ << std::endl;
    std::cout << "Term Color: " << term_color_ << std::endl;
}

/**
 * Destructor
 */
RoshellGraphics::~RoshellGraphics()
{
}

/**
 * This function updates the terminal shape.
 */
void RoshellGraphics::update_buffer()
{
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

    term_height_ = w.ws_row;
    term_width_ = w.ws_col;

    clear_buffer();
}

/**
 * Clear the buffer
*/
void RoshellGraphics::clear_buffer()
{
    buffer_ = std::string(term_height_ * term_width_, ' ');
}

/**
 * Draws a line between two points
*/
std::vector<int> RoshellGraphics::line(const Point& pp1, const Point& pp2)
{   
    std::vector<int> indices;
    
    // Make copies so they can be modified
    Point p1 = pp1;
    Point p2 = pp2;

    put_within_limits_(p1);
    put_within_limits_(p2);

    if (p1.first == p2.first)
    {
        if (p1.second < p2.second)
        {
            for (int y = p1.second; y < p2.second; y++)
            {
                int idx = encode_point_({p1.first, y});
                indices.push_back(idx);
                buffer_[idx] = '.';
            }
        }
        else
        {
            for (int y = p2.second; y < p1.second; y++)
            {
                int idx = encode_point_({p1.first, y});
                indices.push_back(idx);
                buffer_[idx] = '.';
            }
        }
    }
    else 
    {
        float slope = static_cast<float>((static_cast<float>(p2.second) - static_cast<float>(p1.second)) / 
            (static_cast<float>(p2.first) - static_cast<float>(p1.first)));
        std::cout << slope << std::endl;
        if (abs(slope) <= 1.0)
        {
            if (p1.first < p2.first)
            {
                for (int x = p1.first; x < p2.first; x++)
                {
                    int idx = encode_point_({x, p1.second + slope * (x - p1.first)});
                    indices.push_back(idx);
                    buffer_[idx] = '.';
                }
            }
            else
            {
                for (int x = p2.first; x < p1.first; x++)
                {
                    int idx = encode_point_({x, p1.second + slope * (x - p1.first)});
                    indices.push_back(idx);
                    buffer_[idx] = '.';
                }
            }
        }
        else 
        {
            if (p1.second < p2.second)
            {
                for (int y = p1.second; y < p2.second; y++)
                {
                    int idx = encode_point_({p1.first + (y - p1.second) / slope, y});
                    indices.push_back(idx);
                    buffer_[idx] = '.';
                }
            }
            else
            {
                for (int y = p2.second; y < p1.second; y++)
                {
                    int idx = encode_point_({p1.first + (y - p1.second) / slope, y});
                    indices.push_back(idx);
                    buffer_[idx] = '.';
                }
            }
        }
    }
    return indices;
}

void RoshellGraphics::draw_frame()
{
    Point pl, pr, pt, pb;
    pl = std::make_pair(-term_width_ / 2, 0);
    pr = std::make_pair(term_width_ / 2, 0);
    pt = std::make_pair(0, term_height_ / 2);
    pb = std::make_pair(0, -term_height_ / 2);

    fix_frame(pl);
    fix_frame(pr);
    fix_frame(pt);
    fix_frame(pb);

    line(pl, pr);
    line(pt, pb);
    draw();
}

void RoshellGraphics::fix_frame(Point& p)
{
    p.first = p.first + static_cast<int>(term_width_ / 2);
    p.second = -p.second + static_cast<int>(term_height_ / 2);
}

/**
 * Encode (i, j) into an index location
*/
int RoshellGraphics::encode_point_(const Point& p)
{
    return p.second * term_width_ + p.first;
}


/**
 * Decode encoded point into a Point
*/
Point RoshellGraphics::decode_index_(int index)
{
    return std::make_pair(static_cast<int>(index % term_width_), static_cast<int>(index / term_width_));
}


/**
 * Draw the buffer
*/
void RoshellGraphics::draw()
{
    std::cout << buffer_ << std::endl;
}

/**
 * Draw and clear. Only to be used until I can figure out how to find changes in buffer
*/
void RoshellGraphics::draw_and_clear(unsigned long delay)
{
    draw();
    usleep(delay);
    update_buffer();
}

/**
 * Puts the given point within the limits of the terminal
*/
void RoshellGraphics::put_within_limits_(Point& p)
{
    p.first = std::max(p.first, 0);
    p.first = std::min(p.first, term_width_);

    p.second = std::max(p.second, 0);
    p.second = std::min(p.second, term_height_);
}

/**
 * Returns the current terminal size as a pair (width, height)
*/
std::pair<int, int> RoshellGraphics::get_terminal_size()
{
    return std::make_pair(term_width_, term_height_);
}

}  // namespace roshell_graphics