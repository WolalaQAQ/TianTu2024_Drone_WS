#include"takeoff_landing_sys.h"



int main(int argc, char** argv) {
    ros::init(argc, argv, "takeoff_landing_sys");
    ros::NodeHandle nh;
    TakeoffLandingSys takeoffLandingSys(nh);
    ros::spin();
    return 0;
}
