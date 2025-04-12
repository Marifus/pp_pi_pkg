#include "pp_pi_pkg/controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("~");

    pp_pi::PP_PI Baslat(nh);

    ros::spin();
    return 0;
}