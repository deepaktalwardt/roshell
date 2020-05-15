#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include <roshell_graphics/line_plotting.h>

int main(int argc, char** argv){
    roshell_graphics::PlotGraph pg;
    int min_y=1;
    int max_y=100;
    std::string xlabel="X-axis";
    std::string ylabel="Y-axis";
    // Draw axis on terminal
    // pg.DrawAxis(ylabel); //Add y-axis value
    std::vector<int> points_list{30,1};
    pg.PlotPoints(points_list,min_y,max_y,ylabel);
    pg.draw();
}