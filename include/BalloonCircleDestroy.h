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
/*Better enums  */
#include "enum.h"

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerPoint.h>
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
/* math  */
#include <math.h>
// | ------------------- transfroms include ------------------- |
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



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
  bool           is_initialized_ = false;

  bool           is_idling_ = false;
  ros::Timer     timer_idling_;
  /* ros parameters */
  double         _idle_time_;
  void           callbackTimerIdling(const ros::TimerEvent& te);
  bool           _simulation_;
  int            _arena_width_;
  int            _arena_length_;
  int            _arena_accuracy_;
  double         _arena_center_x_;
  double         _arena_center_y_;
  float          _height_;
  double         _circle_radius_;
  int            _circle_accuracy_;
  double         _vel_;
  double         _dist_to_balloon_;
  int            _traj_len_;
  int            _traj_time_;
  // | ------------------------- state machine params ------------------------- |
  enum State     {IDLE,GOING_AROUND, GOING_TO_ANGLE,GOING_TO_BALLOON,AT_BALLOON,CIRCLE_AROUND, DESTROYING };
  State          _state_ = IDLE;

  bool           _is_state_machine_active_ = false;
  // | --------------------- class variables -------------------- |
  double         _closest_on_arena_ = 999.9;

  // | ----------------------- transforms ----------------------- |


  std::string                                 world_frame_id_;
  double                                      world_point_x_;
  double                                      world_point_y_;
  double                                      world_point_z_;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;


  bool transformPointFromWorld(geometry_msgs::Point& point, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::Point& point_out);
  bool getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::TransformStamped& transform_out);

  // | ---------------------- msg callbacks --------------------- |

  void               callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
  ros::Subscriber    sub_odom_uav_;
  nav_msgs::Odometry odom_uav_;
  Eigen::Vector3d    odom_vector_;
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
  ros::Subscriber sub_balloon_point_;
  geometry_msgs::PoseWithCovarianceStamped balloon_point_;
  Eigen::Vector3d balloon_vector_;
  bool            got_balloon_point_ = false;
  bool            is_ballon_incoming_= false;
  std::mutex      mutex_is_balloon_incoming_;
  ros::Time       time_last_balloon_point_;

  void            callbackBalloonPointCloud(const sensor_msgs::PointCloudConstPtr& msg);
  ros::Subscriber           sub_balloon_point_cloud_;
  sensor_msgs::PointCloud   balloon_point_cloud_;
  bool                      got_balloon_point_cloud_ = false;
  bool                      is_ballon_cloud_incoming_= false;
  std::mutex                mutex_is_balloon_cloud_incoming_;
  ros::Time                 time_last_balloon_cloud_point_;



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

  void       callbackTimerStateMachine(const ros::TimerEvent& te);
  ros::Timer timer_state_machine_;
  int        _rate_timer_state_machine_;


  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_trajectory_;
  bool               _trajectory_published_;

  ros::ServiceClient srv_planner_reset;
  bool               _planner_reset_;

  // | ---------------- service server callbacks ---------------- |
  bool       callbackCircleAround(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_circle_around_;

  bool       callbackGoCloser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_go_closer_;
  bool       callbackStartStateMachine(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_state_machine_;
  ros::ServiceServer srv_server_stop_state_machine_;

  // | ------------------- dynamic reconfigure ------------------ |
/* dynamic server //{ */


/*   typedef balloon_circle_destroy::dynparamConfig                              Config; */
/*   typedef dynamic_reconfigure::Server<balloon_circle_destroy::dynparamConfig> ReconfigureServer; */
/*   boost::recursive_mutex                                              mutex_dynamic_reconfigure_; */
/*   boost::shared_ptr<ReconfigureServer>                                reconfigure_server_; */
/*   void                                                                callbackDynamicReconfigure(Config& config, uint32_t level); */
/*   balloon_circle_destroy::dynparamConfig                             last_drs_config_; */



//}


  // | -------------------- support functions ------------------- |

/* Support Functions //{ */

  void getCloseToBalloon();
  void getAngleToBalloon();
  void generateTrajectory();
  void goAroundArena();
  double getBalloonHeading();
  std::string getStateName();

//}
};
//}

}  // namespace waypoint_flier
#endif
