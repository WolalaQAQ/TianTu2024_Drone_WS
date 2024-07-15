#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <numeric>


#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"


#include "ttauav_node/flightByVel.h"
#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "std_srvs/Trigger.h" 


