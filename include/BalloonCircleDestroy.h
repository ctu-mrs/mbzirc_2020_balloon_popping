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
#include <mrs_msgs/TrackerTrajectorySrvRequest.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
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

  bool is_idling_ = false;
  ros::Timer timer_idling_;
  double _idle_time_;
  void       callbackTimerIdling(const ros::TimerEvent& te);
  /* ros parameters */
  bool _simulation_;
  // | ---------------- loading circle params -- ---------------- |
  int            _arena_width_;
  int            _arena_length_;
  float          _height_;



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

  void            callbackBalloonPoint(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  ros::Subscriber sub_balloon_point;
  geometry_msgs::PoseWithCovarianceStamped balloon_point_;
  bool            got_balloon_point_ = false;
  bool            is_ballon_incoming_= false;
  std::mutex      mutex_is_balloon_incoming_;
  ros::Time       time_last_balloon_point_;


  // | --------------------- timer callbacks -------------------- |

 void           callbackTimerPublishGoTo(const ros::TimerEvent& te);
  ros::Publisher pub_goto_;
  ros::Timer     timer_publish_goto_;
  int            _rate_timer_publish_goto_;

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;


  void       callbackTimerCheckBalloonPoints(const ros::TimerEvent& te);
  ros::Timer timer_check_balloons_;
  int        _rate_timer_check_balloons_;
  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_trajectory_;
  bool               _trajectory_published_;


  // | ---------------- service server callbacks ---------------- |
  bool       callbackCircleAround(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_circle_around_;

  // | ------------------- dynamic reconfigure ------------------ |

/*   typedef balloon_circle_destroy::dynparamConfig                              Config; */
/*   typedef dynamic_reconfigure::Server<balloon_circle_destroy::dynparamConfig> ReconfigureServer; */
/*   boost::recursive_mutex                                              mutex_dynamic_reconfigure_; */
/*   boost::shared_ptr<ReconfigureServer>                                reconfigure_server_; */
/*   void                                                                callbackDynamicReconfigure(Config& config, uint32_t level); */
/*   balloon_circle_destroy::dynparamConfig                             last_drs_config_; */

  // | --------------------- waypoint idling -------------------- |


  // | -------------------- support functions ------------------- |

};
//}

}  // namespace waypoint_flier
#endif
