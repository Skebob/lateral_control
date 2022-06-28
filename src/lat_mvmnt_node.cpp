#include "lat_mvmnt/lat_mvmnt_controller.hpp"
#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "pacmod_lateral_control");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(4); // use 4 threads

    ros::Time::waitForValid();

    LateralControl LC(&nh);

    spinner.start();
    ros::waitForShutdown();

    return 0;
}