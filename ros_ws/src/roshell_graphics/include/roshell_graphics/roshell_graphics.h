#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <unordered_map>

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>

#include <boost/algorithm/string/join.hpp>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"

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
 * (0, 0)   Screen reference frame
 *    +---------------------------------------------------> x_screen
 *    |                          
 *    |                        y_natural
 *    |                           ^
 *    |                           |
 *    |                           |
 *    |                           |
 *    |                           |
 *    |                           +----------------> x_natural
 *    |                         (0, 0)
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
    void add_line(const Point& pp1, const Point& pp2, std::string c = " ");
    void add_natural_frame();
    void add_points(const Eigen::Matrix2Xf& points);
    void add_points(const Eigen::Matrix3Xf& points);

    // Image functions
    void add_image(const cv::Mat& im, bool preserve_aspect = true);

    // Text functions
    void add_text(const Point& start_point, const std::string& text, bool horizontal = true);
    
    // Public Utility functions
    void transform_to_screen_frame(Point& p);
    void fill_buffer(const int& idx, std::string c = " ");
    // Overloaded function that takes point in screen frame
    void fill_buffer(const Point& p, std::string c = " ");
    // Overloaded function that takes in color as RGB vector
    void fill_buffer(const Point& p, const std::vector<unsigned char>& color, std::string c = " ");
    // Fill color at the correct place
    void fill_color(const int& idx, std::vector<unsigned char> color);

    // Drawing functions
    void draw();
    void draw_and_clear(unsigned long delay);

private:
    // Private Utility functions
    int encode_point_(const Point& p);
    Point decode_index_(const int& index);
    void put_within_limits_(Point& p);
    bool is_within_limits_(const Point& p);
    std::string convert_rgb_to_string_(const std::vector<unsigned char>& color, const std::string& c);

    // Terminal related variables
    int term_height_;
    int term_width_;
    const char* term_type_;
    const char* term_color_;
    
    // Buffer related variables
    // std::string buffer_;
    std::vector<std::string> buffer_;
    std::vector<int> buffer_count_;
    std::vector<std::vector<unsigned char>> buffer_colors_;

    // Defines which characters to use for different densities
    std::unordered_map<int, std::string> count_to_char_map_;

    // Rainbow RGB Color Map Blue -> Red
    // Taken from https://ai.googleblog.com/2019/08/turbo-improved-rainbow-colormap-for.html
    std::vector<std::vector<unsigned char>> colormap_ = {{48,18,59},{50,21,67},{51,24,74},{52,27,81},{53,30,88},{54,33,95},{55,36,102},{56,39,109},{57,42,115},{58,45,121},{59,47,128},{60,50,134},{61,53,139},{62,56,145},{63,59,151},{63,62,156},{64,64,162},{65,67,167},{65,70,172},{66,73,177},{66,75,181},{67,78,186},{68,81,191},{68,84,195},{68,86,199},{69,89,203},{69,92,207},{69,94,211},{70,97,214},{70,100,218},{70,102,221},{70,105,224},{70,107,227},{71,110,230},{71,113,233},{71,115,235},{71,118,238},{71,120,240},{71,123,242},{70,125,244},{70,128,246},{70,130,248},{70,133,250},{70,135,251},{69,138,252},{69,140,253},{68,143,254},{67,145,254},{66,148,255},{65,150,255},{64,153,255},{62,155,254},{61,158,254},{59,160,253},{58,163,252},{56,165,251},{55,168,250},{53,171,248},{51,173,247},{49,175,245},{47,178,244},{46,180,242},{44,183,240},{42,185,238},{40,188,235},{39,190,233},{37,192,231},{35,195,228},{34,197,226},{32,199,223},{31,201,221},{30,203,218},{28,205,216},{27,208,213},{26,210,210},{26,212,208},{25,213,205},{24,215,202},{24,217,200},{24,219,197},{24,221,194},{24,222,192},{24,224,189},{25,226,187},{25,227,185},{26,228,182},{28,230,180},{29,231,178},{31,233,175},{32,234,172},{34,235,170},{37,236,167},{39,238,164},{42,239,161},{44,240,158},{47,241,155},{50,242,152},{53,243,148},{56,244,145},{60,245,142},{63,246,138},{67,247,135},{70,248,132},{74,248,128},{78,249,125},{82,250,122},{85,250,118},{89,251,115},{93,252,111},{97,252,108},{101,253,105},{105,253,102},{109,254,98},{113,254,95},{117,254,92},{121,254,89},{125,255,86},{128,255,83},{132,255,81},{136,255,78},{139,255,75},{143,255,73},{146,255,71},{150,254,68},{153,254,66},{156,254,64},{159,253,63},{161,253,61},{164,252,60},{167,252,58},{169,251,57},{172,251,56},{175,250,55},{177,249,54},{180,248,54},{183,247,53},{185,246,53},{188,245,52},{190,244,52},{193,243,52},{195,241,52},{198,240,52},{200,239,52},{203,237,52},{205,236,52},{208,234,52},{210,233,53},{212,231,53},{215,229,53},{217,228,54},{219,226,54},{221,224,55},{223,223,55},{225,221,55},{227,219,56},{229,217,56},{231,215,57},{233,213,57},{235,211,57},{236,209,58},{238,207,58},{239,205,58},{241,203,58},{242,201,58},{244,199,58},{245,197,58},{246,195,58},{247,193,58},{248,190,57},{249,188,57},{250,186,57},{251,184,56},{251,182,55},{252,179,54},{252,177,54},{253,174,53},{253,172,52},{254,169,51},{254,167,50},{254,164,49},{254,161,48},{254,158,47},{254,155,45},{254,153,44},{254,150,43},{254,147,42},{254,144,41},{253,141,39},{253,138,38},{252,135,37},{252,132,35},{251,129,34},{251,126,33},{250,123,31},{249,120,30},{249,117,29},{248,114,28},{247,111,26},{246,108,25},{245,105,24},{244,102,23},{243,99,21},{242,96,20},{241,93,19},{240,91,18},{239,88,17},{237,85,16},{236,83,15},{235,80,14},{234,78,13},{232,75,12},{231,73,12},{229,71,11},{228,69,10},{226,67,10},{225,65,9},{223,63,8},{221,61,8},{220,59,7},{218,57,7},{216,55,6},{214,53,6},{212,51,5},{210,49,5},{208,47,5},{206,45,4},{204,43,4},{202,42,4},{200,40,3},{197,38,3},{195,37,3},{193,35,2},{190,33,2},{188,32,2},{185,30,2},{183,29,2},{180,27,1},{178,26,1},{175,24,1},{172,23,1},{169,22,1},{167,20,1},{164,19,1},{161,18,1},{158,16,1},{155,15,1},{152,14,1},{149,13,1},{146,11,1},{142,10,1},{139,9,2},{136,8,2},{133,7,2},{129,6,2},{126,5,2},{122,4,3}};

};

/**
 * Constructor
 */
RoshellGraphics::RoshellGraphics()
{   
    // Defaults
    term_height_ = 40; 
    term_width_ = 150;

    std::ios::sync_with_stdio(false);

    update_buffer();

    std::cout << "Terminal Shape (w, h): " << term_width_ << ", " << term_height_ << std::endl;

    term_type_ = std::getenv("TERM");
    term_color_ = std::getenv("COLORTERM");

    // TODO(deepak): Use these to add color to the points
    std::cout << "Term Type: " << term_type_ << std::endl;
    std::cout << "Term Color: " << term_color_ << std::endl;

    // Count to char density map
    count_to_char_map_[0] = " ";
    count_to_char_map_[1] = ".";
    count_to_char_map_[2] = ":";
    count_to_char_map_[3] = "*";
    count_to_char_map_[4] = "$";
    count_to_char_map_[5] = "%";
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
    buffer_ = std::vector<std::string>(buffer_len, " ");
    buffer_count_ = std::vector<int>(buffer_len, 0);
    buffer_colors_ = std::vector<std::vector<unsigned char>>(buffer_len, {255, 255, 255});
}

/**
 * Fill buffer given encoded index and a character (optional)
*/
void RoshellGraphics::fill_buffer(const int& idx, std::string c)
{
    if (c != " ") 
    {
        buffer_[idx] = c;
    }
    else
    {
        buffer_count_[idx]++;
    }
}

/**
 * Overloaded method that takes in a Point in screen coordinates to fill the buffer
 * if in bounds.
*/
void RoshellGraphics::fill_buffer(const Point& p, std::string c)
{
    if (is_within_limits_(p))
    {
        int idx = encode_point_(p);
        fill_buffer(idx, c);
    }
}

/**
 * Overloaded method that takes in a Point in screen coordinates to fill the buffer
 * if in bounds and also adds color to it
*/
void RoshellGraphics::fill_buffer(const Point& p, const std::vector<unsigned char>& color, std::string c)
{
    if (is_within_limits_(p))
    {
        int idx = encode_point_(p);
        fill_buffer(idx, c);
        fill_color(idx, color);
    }
}

/**
 * This color fills the color buffer with the color_str at index idx
*/
void RoshellGraphics::fill_color(const int& idx, std::vector<unsigned char> color)
{
    buffer_colors_[idx] = color;
}

/**
 * Convert RGB vector to encoded string
*/
std::string RoshellGraphics::convert_rgb_to_string_(const std::vector<unsigned char>& color, const std::string& c)
{
    // set color using ANSI escape sequences
    // Excelent explanation here:
    // https://stackoverflow.com/questions/4842424/list-of-ansi-color-escape-sequences

    std::string r, g, b;

    r = std::to_string(color[0]);
    g = std::to_string(color[1]);
    b = std::to_string(color[2]);

    return "\033[38;2;" + r + ";" + g + ";" + b + "m" + c + "\033[0m";
}

/**
 * Adds points in natural frame to the buffer
*/
void RoshellGraphics::add_points(const Eigen::Matrix2Xf& points)
{
    for (int i = 0; i < points.cols(); i++)
    {
        Point p(static_cast<int>(points.col(i)[0]), static_cast<int>(points.col(i)[1]));
        transform_to_screen_frame(p);
        fill_buffer(p);
    }
}

/**
 * Overloaded add_points function that adds points and also adds color
*/
void RoshellGraphics::add_points(const Eigen::Matrix3Xf& points)
{
    float min_val = points.block(2, 0, 1, points.cols()).minCoeff();
    float max_val = points.block(2, 0, 1, points.cols()).maxCoeff();

    float slope =  256.0 / (max_val - min_val);
    int intercept = static_cast<int>(-slope * min_val);

    for (int i = 0; i < points.cols(); i++)
    {
        Point p(static_cast<int>(points.col(i)[0]), static_cast<int>(points.col(i)[1]));
        transform_to_screen_frame(p);

        int color_idx = static_cast<int>(slope * points.col(i)[2]) + intercept;
        fill_buffer(p, colormap_[color_idx]);
    }
}

/**
 * Draws a line between two points provided in the Natural Reference frame
*/
void RoshellGraphics::add_line(const Point& pp1, const Point& pp2, std::string c)
{   
    // Make copies so they can be modified
    Point p1 = pp1;
    Point p2 = pp2;

    transform_to_screen_frame(p1);
    transform_to_screen_frame(p2);

    put_within_limits_(p1);
    put_within_limits_(p2);

    if (p1(0) == p2(0))
    {
        if (p1(1) < p2(1))
        {
            for (int y = p1(1); y < p2(1); y++)
            {
                Point p = {p1(0), y};
                fill_buffer(p, c);
            }
        }
        else
        {
            for (int y = p2(1); y < p1(1); y++)
            {
                Point p = {p1(0), y};
                fill_buffer(p, c);
            }
        }
    }
    else 
    {
        float slope = static_cast<float>((static_cast<float>(p2(1)) - static_cast<float>(p1(1))) / 
            (static_cast<float>(p2(0)) - static_cast<float>(p1(0))));

        if (abs(slope) <= 1.0)
        {
            if (p1(0) < p2(0))
            {
                for (int x = p1(0); x < p2(0); x++)
                {
                    Point p = {x, p1(1) + slope * (x - p1(0))};
                    fill_buffer(p, c);
                }
            }
            else
            {
                for (int x = p2(0); x < p1(0); x++)
                {
                    Point p = {x, p1(1) + slope * (x - p1(0))};
                    fill_buffer(p, c);
                }
            }
        }
        else 
        {
            if (p1(1) < p2(1))
            {
                for (int y = p1(1); y < p2(1); y++)
                {
                    Point p = {p1(0) + (y - p1(1)) / slope, y};
                    fill_buffer(p, c);
                }
            }
            else
            {
                for (int y = p2(1); y < p1(1); y++)
                {
                    Point p = {p1(0) + (y - p1(1)) / slope, y};
                    fill_buffer(p, c);
                }
            }
        }
    }
}

/**
 * Adds a 2D Natural frame to the buffer, helps with debugging
*/
void RoshellGraphics::add_natural_frame()
{
    Point pl, pr, pt, pb;

    pl = Point(-term_width_ / 2, 0);
    pr = Point(term_width_ / 2, 0);
    pt = Point(0, term_height_ / 2);
    pb = Point(0, -term_height_ / 2);

    add_line(pl, pr);
    add_line(pt, pb);
}

/**
 * Adds text to the buffer
 * 
 * TODO(deepak): Fix issue where space becomes a '.' because of density
*/
void RoshellGraphics::add_text(const Point& start_point, const std::string& text, bool horizontal)
{
    Point curr_point = start_point;
    transform_to_screen_frame(curr_point);

    // Ignore if starting position is off screen
    if (!is_within_limits_(curr_point))
    {
        return;
    }

    for(int i = 0; i < text.size(); i++)
    {
        int idx = encode_point_(curr_point);
        fill_buffer(idx, std::to_string(text[i]));
        
        if (horizontal) // iterate over cols
        {
            curr_point(0)++;
        }
        else            // iterate over rows
        {
            curr_point(1)++;
        }

        // Stop as soon as out of limits
        if (!is_within_limits_(curr_point))
        {
            break;
        }
    }
}

/**
 * Converts point from natural reference frame to the screen reference frame
 * 
 * x_screen = x_natural + width / 2
 * y_screen = -y_natural + height / 2
*/
void RoshellGraphics::transform_to_screen_frame(Point& p)
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
    int buffer_len = term_height_ * term_width_;
    std::string out_buffer;
    for (int i = 0; i < buffer_len; i++)
    {
        if (buffer_[i] != " ")  // If buffer[i] already filled, ignore
        {
            out_buffer += convert_rgb_to_string_(buffer_colors_[i], buffer_[i]);
            continue;
        }

        if (buffer_count_[i] < 6)
        {
            buffer_[i] = count_to_char_map_[buffer_count_[i]];
        }
        else
        {
            buffer_[i] = "@";
        }

        out_buffer += convert_rgb_to_string_(buffer_colors_[i], buffer_[i]);
    }

    // Stream buffer to the terminal
    std::cout << out_buffer;
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

/**
 * Returns true if point is within limits, else returns false. Point must be in Screen frame
*/
bool RoshellGraphics::is_within_limits_(const Point& p)
{
    return p(0) >= 0 && p(0) < term_width_ && p(1) >= 0 && p(1) < term_height_;
}

/**
 * Adds image to the buffer. Resizes im to the correct size depending on the size of the terminal
 * and whether preserve_aspect is set to true or false
*/
void RoshellGraphics::add_image(const cv::Mat& im, bool preserve_aspect)
{
    cv::Size new_size;
    cv::Mat image_resized;

    if (preserve_aspect)
    {
        double s = std::min((double) term_height_ / im.rows, 
            (double) term_width_ / im.cols);
        new_size = cv::Size(2 * im.cols * s, im.rows * s);
        image_resized = cv::Mat(new_size, CV_8UC3, cv::Scalar(0, 0, 0));
    }
    else // fullscreen
    {
        new_size = cv::Size(term_width_, term_height_);
        image_resized = cv::Mat(new_size, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::resize(im, image_resized, new_size);

    for (int r = 0; r < image_resized.rows; r++)
    {
        for (int c = 0; c < image_resized.cols; c++)
        {
            int p = encode_point_({c, r});                        /* Point takes in (col, row) */
            cv::Vec3b pixel = image_resized.at<cv::Vec3b>(r, c);  /* But cv::Mat is indexed (row, col) */

            std::vector<unsigned char> color = {pixel[2], pixel[1], pixel[0]};  /* BGR -> RGB */
            fill_buffer(Point(c, r), color, "█");
        }
    }
}

}  // namespace roshell_graphics