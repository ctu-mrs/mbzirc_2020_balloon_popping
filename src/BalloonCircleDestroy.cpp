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
    param_loader.load_param("arena_width", _arena_width_);
    param_loader.load_param("arena_length", _arena_length_);
    param_loader.load_param("height", _height_);
    param_loader.load_param("idle_time", _idle_time_);

    param_loader.load_param("simulation", _simulation_);
    
    param_loader.load_param("rate/check_subscribers", _rate_timer_check_subscribers_);
    param_loader.load_param("rate/publish_goto", _rate_timer_publish_goto_);

    ROS_INFO_STREAM_ONCE("[BalloonCircleDestroy]: params loaded");

    /* subscribe ground truth only in simulation, where it is available */
    if (_simulation_) {
      sub_odom_gt_ = nh.subscribe("odom_gt_in", 1, &BalloonCircleDestroy::callbackOdomGt, this, ros::TransportHints().tcpNoDelay());
    }
    sub_odom_uav_     = nh.subscribe("odom_uav_in", 1, &BalloonCircleDestroy::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());
    sub_tracker_diag_ = nh.subscribe("tracker_diagnostics_in", 1, &BalloonCircleDestroy::callbackTrackerDiag, this, ros::TransportHints().tcpNoDelay());


    sub_balloon_point = nh.subscribe("balloon_point_in",1,&BalloonCircleDestroy::callbackBalloonPoint,this, ros::TransportHints().tcpNoDelay());

    // init service client for publishing trajectories for the drone
    srv_client_trajectory_ = nh.serviceClient<mrs_msgs::TrackerTrajectorySrv>("trajectory_srv");

  // | ---------- initialize dynamic reconfigure server --------- |
  /* reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh)); */
  /* ReconfigureServer::CallbackType f = boost::bind(&BalloonCircleDestroy::callbackDynamicReconfigure, this, _1, _2); */
  /* reconfigure_server_->setCallback(f); */

  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_        = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &BalloonCircleDestroy::callbackTimerCheckSubscribers, this);
  // you can disable autostarting of the timer by the last argument
  /* timer_publish_goto_ = nh.createTimer(ros::Rate(_rate_timer_publish_goto_), &BalloonCircleDestroy::callbackTimerPublishGoTo, this, false, false); */

  timer_check_balloons_ = nh.createTimer(ros::Rate(_rate_timer_check_balloons_), &BalloonCircleDestroy::callbackTimerCheckBalloonPoints, this, false, true);

  ROS_INFO_ONCE("[BalloonCircleDestroy]: initialized");
  is_initialized_ = true;
  
  }

 void BalloonCircleDestroy::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg) {
  if (!is_initialized_) {
    return;
    }
  {
    std::scoped_lock lock(mutex_odom_uav_);
    odom_uav_ = *msg;
  }

  if (!got_odom_uav_) {
    got_odom_uav_ = true;
    ROS_INFO("[%s]: Got first odom", ros::this_node::getName().c_str());
  }

    time_last_odom_uav_ = ros::Time::now();

 }

 void BalloonCircleDestroy::callbackBalloonPoint(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
   if (!is_initialized_) {
    return;
    }
  
   {
    std::scoped_lock lock(mutex_is_balloon_incoming_);
    balloon_point_ = *msg;
   }

   if(!got_balloon_point_) {
    got_balloon_point_ = true;
    ROS_INFO("[%s]: got first balloon point", ros::this_node::getName().c_str());
   }

   time_last_balloon_point_ = ros::Time::now();

 }

 void BalloonCircleDestroy::callbackTrackerDiag(const mrs_msgs::TrackerDiagnosticsConstPtr& msg) {
  if(!is_initialized_) {
    return;
  }
  if (!got_tracker_diag_) {
    got_tracker_diag_ = true;
    ROS_INFO("[%s]: got first tracker diagnostics msg", ros::this_node::getName().c_str());
  }


  if (is_tracking_ && !msg->tracking_trajectory) {
    ROS_INFO("[%s]: reached final point", ros::this_node::getName().c_str());
    std::scoped_lock lock(mutex_is_tracking_);
    is_idling_ =  true;
    ros::NodeHandle nh("~");
    timer_idling_ = nh.createTimer(ros::Duration(_idle_time_), &BalloonCircleDestroy::callbackTimerIdling, this, true);  // the last boolean argument makes the timer run only once
    ROS_INFO("[WaypointFlier]: Idling for %2.2f seconds.", _idle_time_);



  }

  time_last_tracker_diagnostics_ = ros::Time::now();

 }
void BalloonCircleDestroy::callbackTimerIdling([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[%s]: Idling stopped", ros::this_node::getName().c_str());
  is_idling_ = false;
}
/* callbackPoseGt() //{ */

void BalloonCircleDestroy::callbackOdomGt(const nav_msgs::OdometryConstPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  {
    std::scoped_lock lock(mutex_odom_gt_);
    odom_gt_ = *msg;
  }

  if (!got_odom_gt_) {
    got_odom_gt_ = true;
    ROS_INFO("[%s]: got first ground truth odom msg", ros::this_node::getName().c_str());
  }

  time_last_odom_gt_ = ros::Time::now();
}

//}

void BalloonCircleDestroy::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }
  ros::Time time_now = ros::Time::now();
  if(!got_odom_uav_) {
    ROS_WARN_THROTTLE(1.0,"[%s]: No odom msg since node launch", ros::this_node::getName().c_str());

  } else {
    if((time_now - time_last_odom_uav_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0,"[%s]: didnt receive any odom msg for %f secs", ros::this_node::getName().c_str(),(time_now - time_last_odom_uav_).toSec());
    }
  }

/* check whether tracker diagnostics msgs are coming */
  if (!got_tracker_diag_) {
    ROS_WARN_THROTTLE(1.0, "Not received tracker diagnostics msg since node launch.");
  } else {
    if ((time_now - time_last_tracker_diagnostics_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "Not received uav odom msg for %f sec.", (time_now - time_last_tracker_diagnostics_).toSec());
    }
  }

  /* check whether ground truth pose msgs are coming */
  if (_simulation_) {
    if (!got_odom_gt_) {
      ROS_WARN_THROTTLE(1.0, "Not received ground truth odom msg since node launch.");
    } else {
      if ((time_now - time_last_odom_gt_).toSec() > 1.0) {
        ROS_WARN_THROTTLE(1.0, "Not received ground truth odom msg for %f sec.", (time_now - time_last_odom_gt_).toSec());
      }
    }
  }

  /* check whether balloons msgs are coming */
 if (!got_balloon_point_) {
  ROS_WARN_THROTTLE(1.0, "[%s]: haven't received balloon point since launch", ros::this_node::getName().c_str());
 } else {
  if ((time_now - time_last_balloon_point_).toSec() > 1.0) {
    ROS_WARN_THROTTLE(1.0, "[%s]: haven't received any balloon points for %f", ros::this_node::getName().c_str(), (time_now - time_last_balloon_point_).toSec());
  }
 }

}

void BalloonCircleDestroy::callbackTimerCheckBalloonPoints([[maybe_unused]] const ros::TimerEvent& te) {
  
  if(!is_initialized_) {
    return;
  }
 ROS_INFO_THROTTLE(1.0, "[%s]: Got balloon point at x %f ", ros::this_node::getName().c_str(), balloon_point_.pose.pose.position.x); 


}

bool BalloonCircleDestroy::callbackCircleAround([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized_) {
      res.success = false;
      res.message = "Circle Destroyer isn't initialized";

      ROS_WARN("[BalloonCircleDestroy]: couldn't start circling, I am not initialized ");

      return true;
    
    }

    
    return true;
  
}



} // namespace balloon_circle_destroy

PLUGINLIB_EXPORT_CLASS(balloon_circle_destroy::BalloonCircleDestroy, nodelet::Nodelet);
