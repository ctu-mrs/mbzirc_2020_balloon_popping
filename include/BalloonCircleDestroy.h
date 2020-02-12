#pragma once
#ifndef BALLOON_CIRCLE_DESTROY_H
#define BALLOON_CIRCLE_DESTROY_H

#define N_ARENAS 3
#define N_POINTS 7

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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>
/* visulization msgs */
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
/* custom msgs of MRS group */
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>

#include <std_msgs/String.h>


/* custom helper functions from our library */
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <mrs_msgs/TrackerTrajectorySrvRequest.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
/* Planner services headers */

#include <balloon_filter/AddExclusionZone.h>
#include <balloon_filter/StartEstimation.h>
/* for operations with matrices */
#include <Eigen/Dense>
/* math  */
#include <math.h>
#include <cmath>
// | ------------------- transfroms include ------------------- |
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
// | ----------------------- pcl is SHIT ---------------------- |
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

//}
typedef pcl::PointCloud<pcl::PointXYZ> PC;
typedef Eigen::Vector3d                eigen_vect;

namespace balloon_circle_destroy
{

/* Forbidden struct //{ */

struct Forbidden_t
{
  eigen_vect vect_;
  double     r;
  ros::Time  start_time;
  double     lifetime;
};
//}

/* class BalloonCircleDestroy //{ */
class BalloonCircleDestroy : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

  std::string _uav_name_;

private:
  mrs_lib::Transformer transformer_;

private:
  /* flags */
  bool is_initialized_ = false;

  bool       is_idling_ = false;
  ros::Timer timer_idling_;
  /* ros parameters */
  double                                    _idle_time_;
  bool                                      _simulation_;
  float                                     _height_;
  float                                     _min_height_;
  float                                     _max_height_;
  float                                     _height_tol_;
  double                                    _vel_;
  double                                    _vel_attack_;
  double                                    _vel_arena_;
  double                                    _arena_corner_factor_;
  double                                    _dist_to_balloon_;
  double                                    _dist_acc_;
  double                                    _dist_to_overshoot_;
  double                                    _dist_kf_activation_;
  double                                    _traj_len_;
  double                                    _traj_time_;
  double                                    _dist_error_;
  double                                    _wait_for_ball_;
  int                                       _reset_tries_;
  int                                       _balloon_tries_;
  double                                    _forbidden_radius_;
  double                                    _height_offset_;
  double                                    _max_time_balloon_;
  double                                    _x_min_;
  double                                    _x_max_;
  double                                    _y_min_;
  double                                    _y_max_;
  double                                    _z_min_;
  double                                    _z_max_;
  double                                    _arena_center_x_;
  double                                    _arena_center_y_;
  double                                    _state_reset_time_;
  double                                    _overshoot_offset_;
  double                                    _dead_band_factor_;
  double                                    _time_to_emulate_;
  double                                    _balloon_activation_dist_;
  double                                    _fov_step_;
  std::string                               _sweep_constraints_;
  std::string                               _attack_constraints_;
  Eigen::Matrix<double, N_ARENAS, N_POINTS> _arenas_;

  // | ------------------------- state machine params ------------------------- |
  enum State
  {
    IDLE,
    GOING_AROUND,
    CHECKING_BALLOON,
    GOING_TO_BALLOON,
    AT_BALLOON,
    DESTROYING,
    DESTROY_OVERSHOOT,

  };
  State _state_ = IDLE;
  State _prev_state_;

  // Time thresholds for how long drone can be in a state
  double time_to_going_to;
  double time_to_destroy;
  double time_to_check_balloon;


  bool                     _is_state_machine_active_ = false;
  bool                     _is_destroy_enabled_      = true;
  bool                     _height_checking_         = false;
  double                   _closest_on_arena_        = 999.9;
  double                   _closest_angle_           = 0;
  int                      _reset_count_;
  double                   _time_to_land_;
  int                      _cur_arena_width_;
  int                      _cur_arena_length_;
  std::vector<Forbidden_t> _forb_vect_;
  eigen_vect               _estimate_vect_;
  eigen_vect               _cur_reference_;
  eigen_vect               _prev_closest_;
  eigen_vect               _last_goal_;
  bool                     _last_goal_reached_;
  int                      _balloon_try_count_;
  bool                     _is_going_around_;
  ros::Time                _last_time_balloon_seen_;
  double                   _arena_offset_;
  ros::Time                _time_destroy_overshoot_set_;
  void                     changeState(State state);
  ros::Time                time_state_set_;
  double                   cur_state_dur_;
  bool                     destroy_set = false;
  ros::Time                time_traj_sent_;


  // | ----------------------- transforms ----------------------- |


  std::string                                 world_frame_id_;
  std::string                                 untilted_frame_id_;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;


  bool transformPointToWorld(const geometry_msgs::Point& point, const std::string& from_frame, const ros::Time& stamp, geometry_msgs::Point& point_out);
  bool transformQuaternionToUntilted(const geometry_msgs::Quaternion& point, const std::string& to_frame, const ros::Time& stamp,
                                     geometry_msgs::Quaternion& point_out);
  bool getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::TransformStamped& transform_out);

  bool transformPclToWorld(const PC::Ptr& pcl, const std::string& from_frame, const ros::Time& stamp, PC& pcl_out);

  // | ---------------------- msg callbacks --------------------- |

  void                          callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
  ros::Subscriber               sub_odom_uav_;
  nav_msgs::Odometry            odom_uav_;
  geometry_msgs::Vector3Stamped uav_velocity_arena_frame_;
  eigen_vect                    odom_vector_;
  double                        odom_yaw_;
  bool                          got_odom_uav_ = false;
  std::mutex                    mutex_odom_uav_;
  ros::Time                     time_last_odom_uav_;

  void               callbackOdomGt(const nav_msgs::OdometryConstPtr& msg);
  ros::Subscriber    sub_odom_gt_;
  nav_msgs::Odometry odom_gt_;
  bool               got_odom_gt_ = false;
  std::mutex         mutex_odom_gt_;
  ros::Time          time_last_odom_gt_;

  void                                   callbackConstraintsDiag(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg);
  ros::Subscriber                        subscriber_constraints_diag_;
  bool                                   got_constraints_diag_ = false;
  std::string                            cur_constraints_;
  std::mutex                             mutex_constraints_;
  double                                 acceleration_;
  double                                 jerk_;
  mrs_msgs::ConstraintManagerDiagnostics constraints_msg_;
  ros::Time                              time_last_constraints_diagnostics_;


  void            callbackTrackerDiag(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg);
  ros::Subscriber sub_tracker_diag_;
  bool            got_tracker_diag_ = false;
  bool            is_tracking_      = false;
  std::mutex      mutex_is_tracking_;
  ros::Time       time_last_tracker_diagnostics_;


  void                                     callbackBalloonPoint(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  ros::Subscriber                          sub_balloon_point_;
  geometry_msgs::PoseWithCovarianceStamped balloon_point_;
  eigen_vect                               balloon_vector_;
  bool                                     got_balloon_point_  = false;
  bool                                     is_ballon_incoming_ = false;
  std::mutex                               mutex_is_balloon_incoming_;
  ros::Time                                time_last_balloon_point_;

  void                    callbackBalloonPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
  ros::Subscriber         sub_balloon_point_cloud_;
  std::vector<eigen_vect> balloon_pcl_processed_;
  eigen_vect              balloon_closest_vector_;
  bool                    got_balloon_point_cloud_  = false;
  bool                    is_ballon_cloud_incoming_ = false;
  std::mutex              mutex_is_balloon_cloud_incoming_;
  ros::Time               time_last_balloon_cloud_point_;


  // | --------------------- timer callbacks -------------------- |

  void callbackTimerIdling(const ros::TimerEvent& te);

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  void       callbackTimerCheckBalloonPoints(const ros::TimerEvent& te);
  ros::Timer timer_check_balloons_;
  int        _rate_timer_check_balloons_;

  void       callbackTimerStateMachine(const ros::TimerEvent& te);
  ros::Timer timer_state_machine_;
  int        _rate_timer_state_machine_;

  void       callbackTimerCheckStateMachine(const ros::TimerEvent& te);
  ros::Timer timer_check_state_machine_;
  int        _rate_timer_check_state_machine_;

  void       callbackTimerCheckForbidden(const ros::TimerEvent& te);
  ros::Timer timer_check_forbidden_;
  int        _rate_timer_check_forbidden_;


  void       callbackTimerCheckEmulation(const ros::TimerEvent& te);
  ros::Timer timer_check_emulation_;
  int        _rate_timer_check_emulation_;
  ros::Time  destroy_start_time_;
  bool       timer_set_ = false;

  // | ------------------ visulization markers ------------------ |
  void           callbackTimerPublishRviz(const ros::TimerEvent& te);
  ros::Timer     timer_publish_rviz_;
  ros::Publisher rviz_pub_;
  std::mutex     mutex_rviz_;
  int            _rate_time_publish_rviz_;
  // | ------------------ mrs status published ------------------ |
  void           callbackTimerPublishStatus(const ros::TimerEvent& te);
  ros::Timer     timer_publish_status_;
  ros::Publisher status_pub_;
  std::mutex     mutex_status_;
  int            _rate_time_publish_status_;

  // Publishing references from PCL and from planner
  ros::Publisher point_pub_;
  ros::Publisher balloon_pub_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_trajectory_;
  bool               _trajectory_published_;
  ros::Time          time_last_traj_published_;

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

  ros::ServiceClient srv_client_stop_;
  bool               _mpc_stop_ = false;

  ros::ServiceClient srv_set_constriants_;
  void               setConstraints(std::string desired_constraints);

  // | ------------------- Estimation services ------------------ |

  ros::ServiceClient srv_planner_reset_estimation_;
  bool               _planner_reset_;

  ros::ServiceClient srv_planner_start_estimation_;
  ros::ServiceClient srv_planner_stop_estimation_;
  bool               _planner_active_ = false;

  ros::ServiceClient srv_planner_add_zone_;
  bool               _planner_zone_added_;

  ros::ServiceClient srv_planner_reset_zones_;
  bool               _planner_reset_zones_;

  ros::Time time_last_planner_reset_;


  // | -------------------- Planner functions ------------------- |

  void plannerActivate(eigen_vect estimation, double radius_);
  void plannerStop();
  bool plannerReset();
  void addForbidden(eigen_vect forb, double radius);
  bool checkIfForbidden(eigen_vect forb, double raidus);


  // | ---------------- service server callbacks ---------------- |
  bool               callbackCircleAround(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_circle_around_;

  bool               callbackGoCloser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_go_closer_;

  bool               callbackStartStateMachine(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_state_machine_;


  bool               callbackStopStateMachine(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_stop_state_machine_;

  bool               callbackToggleDestroy(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_toggle_destroy_;

  bool               callbackResetZones(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_reset_zones_;


  bool               callbackAutoStart(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res);
  ros::ServiceServer srv_server_auto_start_;


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

  void        getCloseToBalloon(eigen_vect dest_, double dist, double speed_);
  double      getBalloonHeading(eigen_vect dest_);
  double      getArenaHeading(eigen_vect a_, eigen_vect b_);
  std::string getStateName();
  bool        pointInForbidden(eigen_vect vect_);
  void        checkForbidden();
  void        addToForbidden(eigen_vect dest_);
  bool        balloonOutdated();
  void        landAndEnd();
  eigen_vect  getClosestBalloon();
  bool        isBalloonVisible(eigen_vect balloon_);
  bool        droneStop();
  bool        isPointInArena(float x, float y, float z);
  bool        isPointInArena(eigen_vect p_);
  bool        isPointInArena(mrs_msgs::TrackerPoint p_);
  void        scanArena();
  void        goToPoint(eigen_vect p_, eigen_vect goal, double speed_, mrs_msgs::TrackerTrajectory& new_traj_, double yaw);
  void        goToHeight(double height_, double speed_);
  eigen_vect  deadBand(eigen_vect target_, eigen_vect reference_);
  bool        setArena(int i);
  double      getAngleBetween(double a, double b);
  //}
};
//}

}  // namespace balloon_circle_destroy
#endif
