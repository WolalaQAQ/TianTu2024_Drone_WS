#include"uav_decision.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "uav_decision");
    ros::NodeHandle nh;
    ros::service::waitForService("flightByVel");

    UavDecision uav_decision(nh);

    ros::spin();

    return 0;
}