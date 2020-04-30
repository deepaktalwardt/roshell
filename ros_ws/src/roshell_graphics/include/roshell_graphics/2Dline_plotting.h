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
    void DrawAxis(int xlim, int ylim);
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
Point term_size=get_terminal_size();
term_height = term_size.second;
term_width = term_size.first;
dh = static_cast<int>(0.1*(term_height));
dw = static_cast<int>(0.1*(term_width));
}

PlotGraph::~PlotGraph(){

}
void PlotGraph :: DrawAxis(int xlim,int ylim)
{
Xlim=xlim;
Ylim=ylim;

Point origin = std::make_pair((-term_width/2 + dw ), (-term_height/2 + dh));
Point xlimit = std::make_pair((-term_width/2 + dw + (0.8*term_width)), (-term_height/2 + dh));
Point ylimit = std::make_pair((-term_width/2 + dw ), (-term_height/2 + dh + (0.8*term_height)));

fix_frame(origin);
fix_frame(xlimit);
fix_frame(ylimit);

clear_buffer();
line(origin,xlimit,'_');
line(origin,ylimit,'|');
draw();
}

}




