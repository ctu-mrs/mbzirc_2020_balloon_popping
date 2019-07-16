#pragma once
#ifndef BALLOON_CIRCLE_DESTROY_H
#define BALLOON_CIRCLE_DESTROY_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script dynparam.cfg */
#include <balloon_circle_destroy/dynparamConfig.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/TrackerPointStamped.h>
#include <mrs_msgs/TrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>

/* custom helper functions from our library */
#include <mrs_lib/ParamLoader.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

/* for operations with matrices */
#include <Eigen/Dense>

//}

namespace balloon_circle_destroy
{

/* class BalloonCircleDestroy //{ */
class BalloonCircleDestroy : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  bool _simulation_;

  // | ---------------------- msg callbacks --------------------- |

  void               callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
  ros::Subscriber    sub_odom_uav_;
  nav_msgs::Odometry odom_uav_;
  bool               got_odom_uav_ = false;
  std::mutex         mutex_odom_uav_;
  ros::Time          time_last_odom_uav_;

  void               callbackOdomGt(const nav_msgs::OdometryConstPtr& msg);
  ros::Subscriber    sub_odom_gt_;
  nav_msgs::Odometry odom_gt_;
  bool               got_odom_gt_ = false;
  std::mutex         mutex_odom_gt_;
  ros::Time          time_last_odom_gt_;

  void            callbackTrackerDiag(const mrs_msgs::TrackerDiagnosticsConstPtr& msg);
  ros::Subscriber sub_tracker_diag_;
  bool            got_tracker_diag_ = false;
  bool            is_tracking_      = false;
  std::mutex      mutex_is_tracking_;
  ros::Time       time_last_tracker_diagnostics_;

  void            callbackBalloonPoint(const geometry_msgs::PoseWithCovarianceStamped& msg);
  ros::Subscriber sub_balloon_point;
  bool            got_balloon_point = false;
  bool            is_ballon_incoming      = false;
  std::mutex      mutex_is_balloon_incoming;
  ros::Time       time_last_balloon_point;


  // | --------------------- timer callbacks -------------------- |

  void           callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);
  ros::Publisher pub_dist_to_waypoint_;
  ros::Timer     timer_publish_dist_to_waypoint_;
  int            _rate_timer_publish_dist_to_waypoint_;

  void           callbackTimerPublishGoTo(const ros::TimerEvent& te);
  ros::Publisher pub_goto_;
  ros::Timer     timer_publish_goto_;
  int            _rate_timer_publish_goto_;

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | ---------------- service server callbacks ---------------- |
  // | ------------------- dynamic reconfigure ------------------ |

  typedef waypoint_flier::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<waypoint_flier::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                              mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                reconfigure_server_;
  void                                                                callbackDynamicReconfigure(Config& config, uint32_t level);
  waypoint_flier::dynparamConfig                                      last_drs_config_;

  // | --------------------- waypoint idling -------------------- |


  // | -------------------- support functions ------------------- |

};
//}

}  // namespace waypoint_flier
#endif
