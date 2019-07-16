#include "BalloonCircleDestroy.h"


#include <pluginlib/class_list_macros.h>

namespace balloon_circle_destroy {

  void BalloonCircleDestroy::onInit() {
    got_odom_uav_     = false;
    got_odom_gt_      = false;
    got_tracker_diag_ = false;

    is_tracking_      = false;

    ros::NodeHandle nh("~");

    ros::Time::waitForValid();

    
  // | ------------------- load ros parameters ------------------ |
    mrs_lib::ParamLoader param_loader(nh, "BalloonCircleDestroy");


  
  }
} // namespace balloon_circle_destroy
