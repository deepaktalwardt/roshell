#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>
#include "roshell_graphics.h"

// Works as (columns,rows) instead of the opposite
namespace roshell_graphics
{

class PlotGraph:public RoshellGraphics
{
public:
    //Constructors and Destructors
    PlotGraph();
    ~PlotGraph();

    //Useful methods
    void draw_axis(const std::string& ylabel);

    void plot_points(
        const std::vector<float>& points_list,
        const float& min_y,
        const float& max_y,
        const std::string& ylabel);

private:
    // Varibles for axis limits
    int Xlim_;
    int Ylim_;
    // For axis drawing
    int pad_h_;
    int pad_w_;
    // Tick values
    int one_tick_x_ ;
    //Points
    Point origin_;
    Point xlimit_;
    Point ylimit_;

};

PlotGraph::PlotGraph()
{
    pad_h_ = static_cast<int>(0.1*(term_height_));
    pad_w_ = static_cast<int>(0.05*(term_width_));
}

PlotGraph::~PlotGraph() = default;

void PlotGraph::draw_axis(const std::string& ylabel)
{
    /*
    Define points in the natural frame:
    origin_: pixel where origin is situated
    xlimit_: pixel upto which we will draw x-axis
    ylimit_: pixel upto which we will draw y-axis
    */
    origin_ = Eigen::Vector2i((-term_width_/2 + pad_w_ ), (-term_height_/2 + pad_h_));
    xlimit_ = Eigen::Vector2i((-term_width_/2 + pad_w_ + (0.85*term_width_)), (-term_height_/2 + pad_h_));
    ylimit_ = Eigen::Vector2i((-term_width_/2 + pad_w_ ), (-term_height_/2 + pad_h_ + (0.85*term_height_)));

    //Points to put text labels
    Point text_x;
    if(term_height_ > 28)
    {
        text_x = Eigen::Vector2i((xlimit_[0]-5),(xlimit_[1] - 2));
    }
    else
    { 
        text_x = Eigen::Vector2i((xlimit_[0]+2),(xlimit_[1]));
    }
    Point text_y = Eigen::Vector2i((ylimit_[0] - 2),(ylimit_[1] + 1));

    // Draw X-axis and y-axis on terminal
    add_line(origin_,xlimit_,"-");
    add_line(origin_,ylimit_,"|");

    // Add labels for X-axis and Y-axis
    Point x_end = Eigen::Vector2i((xlimit_[0] + 2),(xlimit_[1]));
    Point y_end = Eigen::Vector2i((ylimit_[0]),(ylimit_[1] + 1));
    add_text(xlimit_,">");
    add_text(ylimit_,"^");
    add_text(text_x,"Time (s)");
    add_text(text_y,ylabel);

    //Add time axis ticks
    one_tick_x_ = ((xlimit_[0] - origin_[0])/30);
    for (int i = 1; i <= 30; i++)
    {
        Point mark = Eigen::Vector2i((origin_[0] + (one_tick_x_ * i) ),(origin_[1]));
        Point value = Eigen::Vector2i((origin_[0] + (one_tick_x_ * i) ),(origin_[1] - 1));
        add_text(mark,"+");
        if(i%3==0)
        {
            add_text(value,std::to_string(i));
        }
    }
}

void PlotGraph::plot_points(
    const std::vector<float>& points_list,
    const float& min_y,
    const float& max_y,
    const std::string& ylabel)
{
    Point p1 = origin_;
    Point p2 = origin_;
    // int min_value = *std::min_element(points_list.begin(), points_list.end());
    // int max_value = *std::max_element(points_list.begin(), points_list.end());
    int y_len;
    draw_axis(ylabel);

    for (int i = 0;i < points_list.size();i++)
    {
        if(i >= 1)
        {
            p2=p1;
        }
        y_len = (ylimit_[1]-origin_[1]);
        float distance_from_origin =  ((float) points_list[i] * y_len / max_y);

        p1[0] = origin_[0] + (one_tick_x_*(i+1));
        p1[1] = origin_[1] + (int)distance_from_origin;

        if(i >= 1)
        {
            // Calculate slope
            float slope = static_cast<float>(p1[1] - p2[1]) / static_cast<float>(p1[0] - p2[0]);
            if (slope >= -0.8 && slope <= 0.8)
            {
                add_line(p1, p2, "-");
            }
            else if (slope > 0.8)
            {
                add_line(p1, p2, "/");
            }
            else if (slope < -0.8)
            {
                add_line(p1, p2, "\\");
            }
            else
            {
                add_line(p1, p2,  "|");
            }
            add_text(p2, "*");   
        }

        Point y_max_text = Eigen::Vector2i((ylimit_[0] - 4), (ylimit_[1]));
        Point y_min_text = Eigen::Vector2i((origin_[0] - 4), (origin_[1]));

        add_text(y_max_text, std::to_string(static_cast<int>(max_y)));
        add_text(y_min_text, std::to_string(static_cast<int>(min_y)));

        float value_to_write = static_cast<float>(static_cast<int>(points_list[i] * 10.)) / 10.;
        add_text(p1,"*");
        add_text(Point(p1[0] + 2, p1[1] + 0), std::to_string(value_to_write).substr(0, 4));
    }
}

}




