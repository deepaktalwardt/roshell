#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include <roshell_graphics/line_plotting.h>

int main(int argc, char** argv){
    roshell_graphics::PlotGraph pg;
    int max_x=100;
    int max_y=100;
    std::string xlabel="X-axis";
    std::string ylabel="Y-axis";
    // Draw axis on terminal
    pg.DrawAxis(ylabel);
    // int arr[2]={0,1};
    // pg.PlotPoints(arr);
}