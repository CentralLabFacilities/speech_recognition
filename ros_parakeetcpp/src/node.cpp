#include "parakeet/parakeet_capi.h"
#include "ros_parakeetcpp/parakeet.hpp"
#include <cstdio>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parakeet");

    // keep alive
    ros::NodeHandle nh;
    std::string model;

    ros::NodeHandle pnh("~"); 
    pnh.param<std::string>("model", model, "model.gguf");
     
    auto pros = ParakeetRos(nh,model);

    ros::spin();

}