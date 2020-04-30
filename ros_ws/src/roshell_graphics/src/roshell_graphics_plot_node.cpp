#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include <roshell_graphics/2Dline_plotting.h>

void draw_axis(roshell_graphics::PlotGraph& pg,int max_x,int max_y){
    pg.DrawAxis(max_x,max_y);
}

int main(int argc, char** argv){
    roshell_graphics::PlotGraph pg;
    int max_x=100;
    int max_y=100;
    draw_axis(pg,max_x,max_y);
}