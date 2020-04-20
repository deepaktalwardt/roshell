#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_map>

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>

#include <Eigen/Dense>

namespace roshell_graphics
{

/**
 * Notation for the point (x, y), which is (col, row)!
*/
using Point = Eigen::Vector2i;

class RoshellGraphics
{

/**
 * Definitions
 * 
 * Natural reference frame: In this frame, the origin is at the
 * center of the terminal. All points provided to roshell_graphics 
 * must be in the Natural reference frame. The functions in this
 * library are responsible of converting them to Screen reference
 * frame, prior to drawing.
 * 
 * Screen reference frame: In this frame, the origin is at the 
 * top left corner of the terminal window. This is what is used
 * to fill in the buffer prior to drawing.
 * 
 * (0,0)   Screen reference frame
 *    +---------------------------------------------------> x_screen
 *    |                          
 *    |                        y_natural
 *    |                           ^
 *    |                           |
 *    |                           |
 *    |                           |
 *    |                           |
 *    |                           +----------------> x_natural
 *    |                         (0,0)
 *    |
 *    |
 *    |
 *    V
 *  y_screen
*/

public:
    // Constructors and Destructors
    RoshellGraphics();
    ~RoshellGraphics();

    // Buffer related functions
    void update_buffer();
    void clear_buffer();

    // Terminal related functions
    std::pair<int, int> get_terminal_size();

    // Geometry functions
    std::vector<int> line(const Point& pp1, const Point& pp2, char c = ' ');
    void add_frame();
    
    // Public Utility functions
    void fix_frame(Point& p);
    void fill_buff(const int& idx, char c = ' ');

    // Drawing functions
    void draw();
    void draw_and_clear(unsigned long delay);

private:
    // Private Utility functions
    uint8_t rgb_to_byte_(const std::vector<int>& rgb_color);
    int encode_point_(const Point& p);
    Point decode_index_(const int& index);
    void put_within_limits_(Point& p);

    // Terminal related variables
    int term_height_;
    int term_width_;
    const char* term_type_;
    const char* term_color_;
    
    // Buffer related variables
    std::string buffer_;
    std::vector<int> buffer_count_;

    // Defines which characters to use for different densities
    std::unordered_map<int, char> count_to_char_map_;
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

    // Count to char density map
    count_to_char_map_[0] = ' ';
    count_to_char_map_[1] = '.';
    count_to_char_map_[2] = ':';
    count_to_char_map_[3] = '*';
    count_to_char_map_[4] = '$';
    count_to_char_map_[5] = '%';
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
    int buffer_len = term_height_ * term_width_;
    buffer_ = std::string(buffer_len, ' ');
    buffer_count_ = std::vector<int>(buffer_len, 0);
}

void RoshellGraphics::fill_buff(const int& idx, char c)
{
    if (c != ' ')
    {
        buffer_[idx] = c;
    }
    else
    {
        buffer_count_[idx]++;
    }
}

/**
 * Draws a line between two points provided in the Natural Reference frame
*/
std::vector<int> RoshellGraphics::line(const Point& pp1, const Point& pp2, char c)
{   
    std::vector<int> indices;
    
    // Make copies so they can be modified
    Point p1 = pp1;
    Point p2 = pp2;

    fix_frame(p1);
    fix_frame(p2);

    put_within_limits_(p1);
    put_within_limits_(p2);

    if (p1(0) == p2(0))
    {
        if (p1(1) < p2(1))
        {
            for (int y = p1(1); y < p2(1); y++)
            {
                int idx = encode_point_({p1(0), y});
                indices.push_back(idx);
                fill_buff(idx, c);
            }
        }
        else
        {
            for (int y = p2(1); y < p1(1); y++)
            {
                int idx = encode_point_({p1(0), y});
                indices.push_back(idx);
                fill_buff(idx, c);
            }
        }
    }
    else 
    {
        float slope = static_cast<float>((static_cast<float>(p2(1)) - static_cast<float>(p1(1))) / 
            (static_cast<float>(p2(0)) - static_cast<float>(p1(0))));
        std::cout << slope << std::endl;
        if (abs(slope) <= 1.0)
        {
            if (p1(0) < p2(0))
            {
                for (int x = p1(0); x < p2(0); x++)
                {
                    int idx = encode_point_({x, p1(1) + slope * (x - p1(0))});
                    indices.push_back(idx);
                    fill_buff(idx, c);
                }
            }
            else
            {
                for (int x = p2(0); x < p1(0); x++)
                {
                    int idx = encode_point_({x, p1(1) + slope * (x - p1(0))});
                    indices.push_back(idx);
                    fill_buff(idx, c);
                }
            }
        }
        else 
        {
            if (p1(1) < p2(1))
            {
                for (int y = p1(1); y < p2(1); y++)
                {
                    int idx = encode_point_({p1(0) + (y - p1(1)) / slope, y});
                    indices.push_back(idx);
                    fill_buff(idx, c);
                }
            }
            else
            {
                for (int y = p2(1); y < p1(1); y++)
                {
                    int idx = encode_point_({p1(0) + (y - p1(1)) / slope, y});
                    indices.push_back(idx);
                    fill_buff(idx, c);
                }
            }
        }
    }
    return indices;
}

void RoshellGraphics::add_frame()
{
    Point pl, pr, pt, pb;

    pl = Point(-term_width_ / 2, 0);
    pr = Point(term_width_ / 2, 0);
    pt = Point(0, term_height_ / 2);
    pb = Point(0, -term_height_ / 2);

    line(pl, pr);
    line(pt, pb);
}

/**
 * Converts point from natural reference frame to the screen reference frame
 * 
 * x_screen = x_natural + width / 2
 * y_screen = -y_natural + height / 2
*/
void RoshellGraphics::fix_frame(Point& p)
{
    p(0) = p(0) + static_cast<int>(term_width_ / 2);
    p(1) = -p(1) + static_cast<int>(term_height_ / 2);
}

/**
 * Encode (col, row) into an index location
*/
int RoshellGraphics::encode_point_(const Point& p)
{
    return p(1) * term_width_ + p(0);
}


/**
 * Decode encoded point into a Point
*/
Point RoshellGraphics::decode_index_(const int& index)
{
    return Point(static_cast<int>(index % term_width_), static_cast<int>(index / term_width_));
}


/**
 * Draw the buffer
*/
void RoshellGraphics::draw()
{
    int buffer_len = buffer_.size();
    for (int i = 0; i < buffer_len; i++)
    {
        if (buffer_[i] != ' ')  // If buffer[i] already filled, ignore
        {
            continue;
        }

        if (buffer_count_[i] < 6)
        {
            buffer_[i] = count_to_char_map_[buffer_count_[i]];
        }
        else
        {
            buffer_[i] = '@';
        }
    } 

    // Print the buffer onto the screen
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
    p(0) = std::max(p(0), 0);
    p(0) = std::min(p(0), term_width_);

    p(1) = std::max(p(1), 0);
    p(1) = std::min(p(1), term_height_);
}

/**
 * Returns the current terminal size as a pair (width, height)
*/
std::pair<int, int> RoshellGraphics::get_terminal_size()
{
    return std::make_pair(term_width_, term_height_);
}

}  // namespace roshell_graphics