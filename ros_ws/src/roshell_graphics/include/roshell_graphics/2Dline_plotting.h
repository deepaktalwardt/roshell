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
    void DrawAxis(int xlim, int ylim,std::string& xlabel,std::string& ylabel);
    // PlotPoint(int column,int row);

private:
    //terminal height and width
    int term_width;
    int term_height;
    // Varibles for axis limits
    int Xlim;
    int Ylim;
    // For axis drawing
    int dh;
    int dw;
};

PlotGraph::PlotGraph(){
std::pair <int,int> term_size=get_terminal_size();
term_height = term_size.second;
term_width = term_size.first;
dh = static_cast<int>(0.1*(term_height));
dw = static_cast<int>(0.05*(term_width));
}

PlotGraph::~PlotGraph(){

}
void PlotGraph :: DrawAxis(int xlim,int ylim,std::string& xlabel,std::string& ylabel)
{
Xlim=xlim;
Ylim=ylim;

/*
Define points:
Origin: pixel where origin is situated
xlimit: pixel upto which we will draw x-axis
ylimit: pixel upto which we will draw y-axis
*/
Point origin = Eigen::Vector2i((-term_width/2 + dw ), (-term_height/2 + dh));
Point xlimit = Eigen::Vector2i((-term_width/2 + dw + (0.85*term_width)), (-term_height/2 + dh));
Point ylimit = Eigen::Vector2i((-term_width/2 + dw ), (-term_height/2 + dh + (0.85*term_height)));

//Points to put text labels
Point text_x;
if(term_height > 32)
    text_x = Eigen::Vector2i((xlimit[0]-5),(xlimit[1] - 2));
else 
     text_x = Eigen::Vector2i((xlimit[0]+1),(xlimit[1]));
Point text_y = Eigen::Vector2i((ylimit[0] - 2),(ylimit[1] + 1));
clear_buffer();

// Draw X-axis and y-axis on terminal
line(origin,xlimit,'_');
line(origin,ylimit,'|');

// Add labels for X-axis and Y-axis
add_text(xlimit,">");
add_text(ylimit,"^");
add_text(text_x,xlabel);
add_text(text_y,ylabel);

draw();
}

}




