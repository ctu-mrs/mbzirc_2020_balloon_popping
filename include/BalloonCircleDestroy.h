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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>
/* visulization msgs */
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
/* custom msgs of MRS group */
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/TrackerPointStamped.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
/* custom helper functions from our library */
#include <mrs_lib/ParamLoader.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <mrs_msgs/TrackerTrajectorySrvRequest.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
/* Planner services headers */

#include <balloon_planner/AddExclusionZone.h>
#include <balloon_planner/StartEstimation.h>
/* for operations with matrices */
#include <Eigen/Dense>
/* math  */
#include <math.h>
#include <cmath>
// | ------------------- transfroms include ------------------- |
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


//}

namespace balloon_circle_destroy
{

/* Forbidden struct //{ */

struct Forbidden_t
{
  Eigen::Vector3d vect_;
  double          r;
};
//}

/* class BalloonCircleDestroy //{ */
class BalloonCircleDestroy : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  bool       is_idling_ = false;
  ros::Timer timer_idling_;
  /* ros parameters */
  double _idle_time_;
  void   callbackTimerIdling(const ros::TimerEvent& te);
  bool   _simulation_;
  int    _arena_width_;
  int    _min_arena_width_;
  int    _arena_length_;
  int    _min_arena_length_;
  int    _arena_accuracy_;
  int    _min_arena_accuracy_;
  int    _arena_area_;
  double _arena_center_x_;
  double _arena_center_y_;
  float  _height_;
  float  _min_height_;
  float  _max_height_;
  float  _height_tol_;
  double _circle_radius_;
  int    _circle_accuracy_;
  double _vel_;
  double _vel_attack_;
  double _vel_arena_;
  double _vel_arena_min_;
  double _dist_to_balloon_;
  double _dist_acc_;
  double _dist_to_overshoot_;
  double _traj_len_;
  double _traj_time_;
  double _dist_error_;
  double _wait_for_ball_;
  int    _reset_tries_;
  int    _balloon_tries_;
  double _forbidden_radius_;
  double _height_offset_;
  double _max_time_balloon_;
  double _x_min_;
  double _x_max_;
  double _y_min_;
  double _y_max_;
  double _z_min_;
  double _z_max_;


  // | ------------------------- state machine params ------------------------- |
  enum State
  {
    IDLE,
    GOING_AROUND,
    GOING_TO_ANGLE,
    CHECKING_BALLOON,
    CHOOSING_BALLOON,
    GOING_TO_BALLOON,
    AT_BALLOON,
    CIRCLE_AROUND,
    READY_TO_DESTROY,
    DESTROYING,

  };
  State _state_ = IDLE;

  bool                     _is_state_machine_active_ = false;
  bool                     _is_destroy_enabled_      = false;
  bool                     _height_checking_         = false;
  double                   _closest_on_arena_        = 999.9;
  double                   _closest_angle_           = 0;
  int                      _reset_count_;
  double                   _time_to_land_;
  int                      _cur_arena_width_;
  int                      _cur_arena_length_;
  std::vector<Forbidden_t> _forb_vect_;
  Eigen::Vector3d          _estimate_vect_;
  Eigen::Vector3d          _prev_closest_;
  Eigen::Vector3d          _last_goal_;
  bool                     _last_goal_reached_;
  int                      _balloon_try_count_;
  bool                     _is_going_around_;
  ros::Time                _last_time_balloon_seen_;


  // | ----------------------- transforms ----------------------- |


  std::string                                 world_frame_id_;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;


  bool transformPointFromWorld(const geometry_msgs::Point32& point, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::Point& point_out);
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


  void            callbackTrackerDiag(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg);
  ros::Subscriber sub_tracker_diag_;
  bool            got_tracker_diag_ = false;
  bool            is_tracking_      = false;
  std::mutex      mutex_is_tracking_;
  ros::Time       time_last_tracker_diagnostics_;


  void                                     callbackBalloonPoint(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  ros::Subscriber                          sub_balloon_point_;
  geometry_msgs::PoseWithCovarianceStamped balloon_point_;
  Eigen::Vector3d                          balloon_vector_;
  bool                                     got_balloon_point_  = false;
  bool                                     is_ballon_incoming_ = false;
  std::mutex                               mutex_is_balloon_incoming_;
  ros::Time                                time_last_balloon_point_;

  void                    callbackBalloonPointCloud(const sensor_msgs::PointCloudConstPtr& msg);
  ros::Subscriber         sub_balloon_point_cloud_;
  sensor_msgs::PointCloud balloon_point_cloud_;
  Eigen::Vector3d         balloon_closest_vector_;
  bool                    got_balloon_point_cloud_  = false;
  bool                    is_ballon_cloud_incoming_ = false;
  std::mutex              mutex_is_balloon_cloud_incoming_;
  ros::Time               time_last_balloon_cloud_point_;


  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;


  void       callbackTimerCheckBalloonPoints(const ros::TimerEvent& te);
  ros::Timer timer_check_balloons_;
  int        _rate_timer_check_balloons_;

  void       callbackTimerStateMachine(const ros::TimerEvent& te);
  ros::Timer timer_state_machine_;
  int        _rate_timer_state_machine_;

  // | ------------------ visulization markers ------------------ |
  void           callbackTimerPublishRviz(const ros::TimerEvent& te);
  ros::Timer     timer_publish_rviz_;
  ros::Publisher rviz_pub_;
  std::mutex     mutex_rviz_;
  int            _rate_time_publish_rviz_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_trajectory_;
  bool               _trajectory_published_;
  ros::Time          time_last_traj_published_;

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

  ros::ServiceClient srv_client_stop_;
  bool               _mpc_stop_ = false;
  // | ------------------- Estimation services ------------------ |

  ros::ServiceClient srv_planner_reset_estimation_;
  bool               _planner_reset_;

  ros::ServiceClient srv_planner_start_estimation_;
  ros::ServiceClient srv_planner_stop_estimation_;
  bool               _planner_active_ = false;

  ros::ServiceClient srv_planner_add_zone_;
  bool               _planner_zone_added_;


  ros::Time time_last_planner_reset_;

  // | -------------------- Planner functions ------------------- |

  void plannerActivate(Eigen::Vector3d estimation, double radius_);
  void plannerStop();
  bool plannerReset();
  void addForbidden(Eigen::Vector3d forb, double radius);
  bool checkIfForbidden(Eigen::Vector3d forb, double raidus);


  // | ---------------- service server callbacks ---------------- |
  bool               callbackCircleAround(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_circle_around_;

  bool               callbackGoCloser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_go_closer_;

  bool               callbackStartStateMachine(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_state_machine_;
  ros::ServiceServer srv_server_stop_state_machine_;

  bool               callbackToggleDestroy(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_toggle_destroy_;

  // | ------------------- dynamic reconfigure ------------------ |
  /* dynamic server //{ */


  typedef balloon_circle_destroy::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<balloon_circle_destroy::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                                      mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                        reconfigure_server_;
  void                                                                        callbackDynamicReconfigure(Config& config, uint32_t level);
  balloon_circle_destroy::dynparamConfig                                      last_drs_config_;


  //}


  // | -------------------- support functions ------------------- |

  /* Support Functions //{ */

  void                       getCloseToBalloon(Eigen::Vector3d dest_, double dist, double speed_);
  void                       circleAroundBalloon();
  void                       getAngleToBalloon();
  void                       generateTrajectory();
  void                       goAroundArena();
  void                       goToChosenBalloon();
  double                     getBalloonHeading(Eigen::Vector3d dest_);
  double                     getArenaHeading();
  std::string                getStateName();
  bool                       pointInForbidden(Eigen::Vector3d vect_);
  void                       checkForbidden();
  void                       addToForbidden(Eigen::Vector3d dest_);
  bool                       balloonOutdated();
  void                       landAndEnd();
  Eigen::Vector3d            getClosestBalloon();
  bool                       isBalloonVisible(Eigen::Vector3d baloon_);
  bool                       droneStop();
  visualization_msgs::Marker fillArenaBounds(int id_);
  //}
};
//}

}  // namespace balloon_circle_destroy
#endif
