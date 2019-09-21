#include "BalloonCircleDestroy.h"


#include <pluginlib/class_list_macros.h>

namespace balloon_circle_destroy
{
/* init //{ */


void BalloonCircleDestroy::onInit() {
  got_odom_uav_     = false;
  got_odom_gt_      = false;
  got_tracker_diag_ = false;

  is_tracking_        = false;
  _reset_count_       = 0;
  _balloon_try_count_ = 0;
  _forb_vect_         = std::vector<Forbidden_t>();

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();


  // | ------------------- load ros parameters ------------------ |
  /* params //{ */

  mrs_lib::ParamLoader param_loader(nh, "BalloonCircleDestroy");
  param_loader.load_param("arena_width", _arena_width_);
  param_loader.load_param("min_arena_width", _min_arena_width_);
  param_loader.load_param("arena_length", _arena_length_);
  param_loader.load_param("min_arena_length", _min_arena_length_);
  param_loader.load_param("arena_center_x", _arena_center_x_);
  param_loader.load_param("arena_center_y", _arena_center_y_);
  param_loader.load_param("arena_accuracy", _arena_accuracy_);
  param_loader.load_param("min_arena_accuracy", _min_arena_accuracy_);
  param_loader.load_param("elips_height", _height_);
  param_loader.load_param("height_min", _min_height_);
  param_loader.load_param("height_max", _max_height_);
  param_loader.load_param("height_tol", _height_tol_);
  param_loader.load_param("idle_time", _idle_time_);
  param_loader.load_param("circling_radius", _circle_radius_);
  param_loader.load_param("circle_accuracy", _circle_accuracy_);
  param_loader.load_param("traj_time", _traj_time_);
  param_loader.load_param("traj_len", _traj_len_);
  param_loader.load_param("dist_to_balloon", _dist_to_balloon_);
  param_loader.load_param("dist_to_overshoot", _dist_to_overshoot_);
  param_loader.load_param("vel", _vel_);
  param_loader.load_param("vel_attack", _vel_attack_);
  param_loader.load_param("vel_arena", _vel_arena_);
  param_loader.load_param("min_vel_arena", _vel_arena_min_);
  param_loader.load_param("dist_error", _dist_error_);
  param_loader.load_param("wait_for_ball", _wait_for_ball_);

  param_loader.load_param("simulation", _simulation_);

  param_loader.load_param("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.load_param("rate/check_balloons", _rate_timer_check_balloons_);
  param_loader.load_param("rate/state_machine", _rate_timer_state_machine_);
  param_loader.load_param("rate/pub_rviz", _rate_time_publish_rviz_);

  param_loader.load_param("world_frame_id", world_frame_id_);
  param_loader.load_param("reset_tries", _reset_tries_);
  param_loader.load_param("balloon_tries", _balloon_tries_);
  param_loader.load_param("time_to_land", _time_to_land_);
  param_loader.load_param("forbidden_radius", _forbidden_radius_);
  param_loader.load_param("height_offset", _height_offset_);

  ROS_INFO_STREAM_ONCE("[BalloonCircleDestroy]: params loaded");
  _cur_arena_width_  = _arena_width_;
  _cur_arena_length_ = _arena_length_;

  //}
  // | ------------------- Dynamic reconfigure ------------------ |

  /*  Dynamic reconfigure //{ */

  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&BalloonCircleDestroy::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);


  //}

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------
  /* subscribers //{ */

  if (_simulation_) {
    sub_odom_gt_ = nh.subscribe("odom_gt_in", 1, &BalloonCircleDestroy::callbackOdomGt, this, ros::TransportHints().tcpNoDelay());
  }
  sub_odom_uav_     = nh.subscribe("odom_uav_in", 1, &BalloonCircleDestroy::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());
  sub_tracker_diag_ = nh.subscribe("tracker_diagnostics_in", 1, &BalloonCircleDestroy::callbackTrackerDiag, this, ros::TransportHints().tcpNoDelay());


  sub_balloon_point_ = nh.subscribe("balloon_point_in", 1, &BalloonCircleDestroy::callbackBalloonPoint, this, ros::TransportHints().tcpNoDelay());
  sub_balloon_point_cloud_ =
      nh.subscribe("balloon_point_cloud_in", 1, &BalloonCircleDestroy::callbackBalloonPointCloud, this, ros::TransportHints().tcpNoDelay());


  //}

  // | ----------------------- Publishers ----------------------- |

  rviz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rviz_out", 1);

  // | --------------- initialize service servers --------------- |
  /*  server services //{ */

  srv_server_circle_around_       = nh.advertiseService("circle_around", &BalloonCircleDestroy::callbackCircleAround, this);
  srv_server_go_closer_           = nh.advertiseService("go_closer", &BalloonCircleDestroy::callbackGoCloser, this);
  srv_server_start_state_machine_ = nh.advertiseService("start_state_machine", &BalloonCircleDestroy::callbackStartStateMachine, this);
  srv_server_stop_state_machine_  = nh.advertiseService("stop_state_machine", &BalloonCircleDestroy::callbackStartStateMachine, this);
  srv_server_toggle_destroy_      = nh.advertiseService("toggle_destroy", &BalloonCircleDestroy::callbackToggleDestroy, this);


  //}

  /* Service clients //{ */


  // init service client for publishing trajectories for the drone
  srv_client_trajectory_        = nh.serviceClient<mrs_msgs::TrackerTrajectorySrv>("trajectory_srv");
  srv_planner_reset_estimation_ = nh.serviceClient<std_srvs::Trigger>("reset_estimation");
  srv_planner_start_estimation_ = nh.serviceClient<balloon_planner::StartEstimation>("start_estimation");
  srv_planner_stop_estimation_  = nh.serviceClient<std_srvs::Trigger>("stop_estimation");
  srv_planner_add_zone_         = nh.serviceClient<balloon_planner::AddExclusionZone>("add_zone");

  time_last_planner_reset_ = ros::Time::now();

  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("land_out");


  //}

  // | ---------- initialize dynamic reconfigure server --------- |
  /* dynamic server //{ */

  /* reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh)); */
  /* ReconfigureServer::CallbackType f = boost::bind(&BalloonCircleDestroy::callbackDynamicReconfigure, this, _1, _2); */
  /* reconfigure_server_->setCallback(f); */


  //}
  // | -------------------- initialize timers ------------------- |
  /* timers //{ */

  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &BalloonCircleDestroy::callbackTimerCheckSubscribers, this);
  // you can disable autostarting of the timer by the last argument
  timer_state_machine_  = nh.createTimer(ros::Rate(_rate_timer_state_machine_), &BalloonCircleDestroy::callbackTimerStateMachine, this, false, true);
  timer_check_balloons_ = nh.createTimer(ros::Rate(_rate_timer_check_balloons_), &BalloonCircleDestroy::callbackTimerCheckBalloonPoints, this, false, true);
  timer_publish_rviz_   = nh.createTimer(ros::Rate(_rate_time_publish_rviz_), &BalloonCircleDestroy::callbackTimerPublishRviz, this, false, true);


  //}
  //
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  ROS_INFO_ONCE("[BalloonCircleDestroy]: initialized");

  is_initialized_ = true;
}


//}

/* msg callbacks //{ */


void BalloonCircleDestroy::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }
  {
    std::scoped_lock lock(mutex_odom_uav_);
    odom_uav_    = *msg;
    odom_vector_ = Eigen::Vector3d(odom_uav_.pose.pose.position.x, odom_uav_.pose.pose.position.y, odom_uav_.pose.pose.position.z);
  }

  if (!got_odom_uav_) {
    got_odom_uav_ = true;
    ROS_INFO("[%s]: Got first odom", ros::this_node::getName().c_str());
  }

  time_last_odom_uav_ = ros::Time::now();
}

void BalloonCircleDestroy::callbackBalloonPoint(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  if (!is_initialized_) {
    ROS_INFO("[BalloonCircleDestroy]: not inited");
    return;
  }

  {
    std::scoped_lock lock(mutex_is_balloon_incoming_);

    balloon_point_  = *msg;
    balloon_vector_ = Eigen::Vector3d(balloon_point_.pose.pose.position.x, balloon_point_.pose.pose.position.y, balloon_point_.pose.pose.position.z);
  }

  time_last_balloon_point_ = ros::Time::now();
  if (!got_balloon_point_) {
    got_balloon_point_ = true;
    ROS_INFO("[%s]: got first balloon point", ros::this_node::getName().c_str());
  }
}
void BalloonCircleDestroy::callbackBalloonPointCloud(const sensor_msgs::PointCloudConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_is_balloon_cloud_incoming_);
    balloon_point_cloud_ = *msg;
  }


  if (balloon_point_cloud_.points.size() > 0) {
    if (!got_balloon_point_cloud_) {
      got_balloon_point_cloud_ = true;
      ROS_INFO("[%s]: got first balloon point cloud", ros::this_node::getName().c_str());
    }
    time_last_balloon_cloud_point_ = ros::Time::now();
  }
}


//}

/* callbackTrackerDiag //{ */

void BalloonCircleDestroy::callbackTrackerDiag(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }
  if (!got_tracker_diag_) {
    got_tracker_diag_ = true;
    ROS_INFO("[%s]: got first tracker diagnostics msg", ros::this_node::getName().c_str());
  }

  if (is_tracking_ && msg->tracking_trajectory && _state_ == GOING_AROUND) {

    Eigen::Vector3d ball_vect_;
    double          cur_dist_;

    {
      std::scoped_lock lock(mutex_is_balloon_cloud_incoming_);
      for (unsigned long i = 0; i < balloon_point_cloud_.points.size(); i++) {
        geometry_msgs::Point p_;

        bool ts_res = transformPointFromWorld(balloon_point_cloud_.points.at(i), balloon_point_cloud_.header.frame_id, balloon_point_cloud_.header.stamp, p_);
        if (!ts_res) {
          ROS_WARN_THROTTLE(1, "[BalloonCircleDestroy]: No transform,skipping");
          return;
        }
        // height check
        if (p_.z < _min_height_ || p_.z > _max_height_) {
          continue;
        }

        ball_vect_ = Eigen::Vector3d(p_.x, p_.y, p_.z);

        cur_dist_ = (odom_vector_ - ball_vect_).norm();
        if (pointInForbidden(ball_vect_)) {
          ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: POINT IN FORBIDDEN KURWA");
        }
        if (cur_dist_ < _closest_on_arena_ && !pointInForbidden(ball_vect_)) {
          _closest_on_arena_      = cur_dist_;
          _closest_angle_         = getArenaHeading();
          balloon_closest_vector_ = ball_vect_;
        }
      }
    }
  }

  if (is_tracking_ && !msg->tracking_trajectory) {
    ROS_INFO("[%s]: reached final point", ros::this_node::getName().c_str());
    std::scoped_lock lock(mutex_is_tracking_);
    is_tracking_ = false;
    is_idling_   = true;
    ros::NodeHandle nh("~");
    timer_idling_ = nh.createTimer(ros::Duration(_idle_time_), &BalloonCircleDestroy::callbackTimerIdling, this,
                                   true);  // the last boolean argument makes the timer run only once
    ROS_INFO("[BalloonCircleDestroy]: Idling for %2.2f seconds.", _idle_time_);
  }

  time_last_tracker_diagnostics_ = ros::Time::now();
}


//}

/* callbackTimerIdling //{ */

void BalloonCircleDestroy::callbackTimerIdling([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[%s]: Idling stopped", ros::this_node::getName().c_str());
  is_idling_ = false;
  if (_state_ == DESTROYING) {
    _state_ = IDLE;
    plannerStop();
    if (_cur_arena_width_ < _min_arena_width_) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: ARENA IS SMALLER (width)");
      _cur_arena_width_  = _arena_width_ / 2;
      _cur_arena_length_ = _arena_length_ / 2;
    } else if (_cur_arena_length_ < _min_arena_length_) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: ARENA IS SMALLER (length)");
      _cur_arena_length_ = _arena_length_ / 2;
      _cur_arena_width_  = _arena_width_ / 2;
    } else {
      _cur_arena_width_ -= _closest_on_arena_ / 2;
      _cur_arena_length_ -= _closest_on_arena_ / 2;
    }
    ROS_WARN_THROTTLE(0.5, "[StateMachine]: State changed to %s ", getStateName().c_str());
  }
}

//}

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

//* callbackTimerStateMachine //{ */

void BalloonCircleDestroy::callbackTimerStateMachine([[maybe_unused]] const ros::TimerEvent& te) {
  if (!_is_state_machine_active_) {
    return;
  }
  ROS_INFO_THROTTLE(0.5, "| ---------------- STATE MACHINE LOOP STATUS --------------- |");
  ROS_INFO_THROTTLE(0.5, "[State]: %s ", getStateName().c_str());
  ROS_INFO_THROTTLE(0.5, "[IsTracking]: %d", is_tracking_);
  ROS_INFO_THROTTLE(0.5, "[Planner Status]: %d", _planner_active_);
  ROS_INFO_THROTTLE(0.5, "[Destroy Status]: %d", _is_destroy_enabled_);
  ROS_INFO_THROTTLE(0.5, "[Current Dist To ball]: %f ", (odom_vector_ - balloon_vector_).norm());
  ROS_INFO_THROTTLE(0.5, "[Dist between KF and PCL vectors]: %f ", (balloon_vector_ - balloon_closest_vector_).norm());
  /* ROS_INFO_THROTTLE(0.5, "[Is closest in forbidden]: %d ", pointInForbidden(balloon_closest_vector_)); */
  ROS_INFO_THROTTLE(0.5, "[Closest ball (PointCloud)]: x %f ", balloon_closest_vector_(0, 0));
  ROS_INFO_THROTTLE(0.5, "[Closest ball (PointCloud)]: y %f ", balloon_closest_vector_(1, 0));
  ROS_INFO_THROTTLE(0.5, "[Closest ball (PointCloud)]: z %f ", balloon_closest_vector_(2, 0));
  /* ROS_INFO_THROTTLE(0.5, "[Prev ball (PointCloud)]: x %f ", _prev_closest_(0, 0)); */
  /* ROS_INFO_THROTTLE(0.5, "[Prev ball (PointCloud)]: y %f ", _prev_closest_(1, 0)); */
  /* ROS_INFO_THROTTLE(0.5, "[Prev ball (PointCloud)]: z %f ", _prev_closest_(2, 0)); */
  ROS_INFO_THROTTLE(0.5, "[Closest ball (KF)]: x %f ", balloon_vector_(0, 0));
  ROS_INFO_THROTTLE(0.5, "[Closest ball (KF)]: y %f ", balloon_vector_(1, 0));
  ROS_INFO_THROTTLE(0.5, "[Closest ball (KF)]: z %f ", balloon_vector_(2, 0));

  ROS_INFO_THROTTLE(0.5, "[KF reset tries]  %d ", _reset_count_);
  /* ROS_INFO_THROTTLE(0.5, "[Same balloon tries]  %d ", _balloon_try_count_); */
  /* ROS_INFO_THROTTLE(0.5, "[ARENA LENGTH]: %d ", _cur_arena_length_); */
  /* ROS_INFO_THROTTLE(0.5, "[ARENA WIDTH]: %d ", _cur_arena_width_); */
  /* ROS_INFO_THROTTLE(0.5, "[MIN ARENA LENGTH]: %d ", _min_arena_length_); */
  /* ROS_INFO_THROTTLE(0.5, "[MIN ARENA WIDTH]: %d ", _min_arena_width_); */
  /* ROS_INFO_THROTTLE(0.5, "[ARENA VELOCITY]: %f ", _vel_arena_); */
  /* ROS_INFO_THROTTLE(0.5, "[MIN HEIGHT]: %f ", _min_height_); */
  /* ROS_INFO_THROTTLE(0.5, "[MAX HEIGHT]: %f ", _max_height_); */
  ROS_INFO_THROTTLE(0.5, "[CUR HEIGHT]: %f ", odom_vector_(2, 0));
  ROS_INFO_THROTTLE(0.5, "[Cloud size]: %d ", int(balloon_point_cloud_.points.size()));

  ROS_INFO_THROTTLE(0.5, "| ----------------- STATE MACHINE LOOP END ----------------- |");

  if (_state_ == IDLE) {
    if (is_ballon_cloud_incoming_) {
      _state_ = CHECKING_BALLOON;
    } else {
      _state_ = GOING_AROUND;
      goAroundArena();
    }
    _closest_on_arena_ = 9999;
    _closest_angle_    = 0;
    _reset_count_      = 0;
    balloon_vector_    = Eigen::Vector3d();
    ROS_WARN_THROTTLE(0.5, "[StateMachine]: 374 STATE RESET TO %s", getStateName().c_str());
    plannerStop();
    return;
  } else if (_state_ == GOING_AROUND) {
    if (is_ballon_incoming_) {
      _state_ = CHECKING_BALLOON;
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 374 STATE RESET TO %s", getStateName().c_str());
    }

  } else if (_state_ == CHECKING_BALLOON) {

    balloon_closest_vector_ = getClosestBalloon();
    plannerActivate(balloon_closest_vector_, _dist_to_balloon_);
    _state_ = CHOOSING_BALLOON;
    ROS_WARN_THROTTLE(0.5, "[StateMachine]: 404 STATE RESET TO %s", getStateName().c_str());
  } else if (_state_ == CHOOSING_BALLOON) {
    if (is_tracking_ || is_idling_) {
      return;
    }
    if (is_ballon_incoming_) {
      _state_ = GOING_TO_BALLOON;
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 420 STATE RESET TO %s", getStateName().c_str());
    } else {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: dist between kf and balloon too large");
      if (plannerReset()) {
        _reset_count_++;
      }
    }
    if (_reset_tries_ == _reset_count_) {
      _state_ = IDLE;

      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 423 STATE RESET TO %s", getStateName().c_str());
    }
  } else if (_state_ == GOING_TO_BALLOON) {
    if (balloonOutdated()) {
      _state_ = CHOOSING_BALLOON;
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 435 STATE RESET TO %s", getStateName().c_str());
    }
    if (is_ballon_incoming_ && ((odom_vector_ - balloon_vector_).norm() - _dist_acc_ > _dist_acc_ + _dist_to_balloon_) &&
        ((balloon_vector_ - balloon_closest_vector_).norm() < _dist_acc_ + _dist_to_balloon_)) {
      getCloseToBalloon(balloon_vector_, _dist_to_balloon_, _vel_);
      return;
    } else if (!is_tracking_ && !is_idling_ && is_ballon_incoming_) {
      _state_ = AT_BALLOON;
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 441 STATE RESET TO %s", getStateName().c_str());
    }
  } else if (_state_ == AT_BALLOON) {
    if (!is_tracking_ && !is_idling_) {
      _state_      = CIRCLE_AROUND;
      is_tracking_ = true;
      circleAroundBalloon();
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 448 STATE RESET TO %s", getStateName().c_str());
    }
  } else if (_state_ == CIRCLE_AROUND) {
    if (!is_tracking_ && !is_idling_ && is_ballon_incoming_) {
      _state_ = READY_TO_DESTROY;
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 453 STATE RESET TO %s", getStateName().c_str());
    } else if (!is_tracking_ && !is_idling_ && !is_ballon_incoming_) {
      _state_ = IDLE;
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 462 STATE RESET TO %s", getStateName().c_str());
    }
  } else if (_state_ == READY_TO_DESTROY) {

    if (is_ballon_incoming_ && (odom_vector_ - balloon_vector_).norm() < _dist_error_ + _dist_to_balloon_) {
      _state_ = DESTROYING;
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: 460 STATE RESET TO %s", getStateName().c_str());
    }
  } else if (_state_ == DESTROYING) {

    if (!_is_destroy_enabled_) {
      return;
    }
    if (is_ballon_incoming_ && (balloon_closest_vector_ - balloon_vector_).norm()) {
      getCloseToBalloon(balloon_vector_, -_dist_to_overshoot_, _vel_attack_);
    } else {

      plannerStop();
      _is_state_machine_active_ = false;
      /* * if (balloonOutdated()) { *1/ */
      /*   _state_ = IDLE; */
      /*   ROS_WARN_THROTTLE(0.5, "[StateMachine]: 460 STATE RESET TO %s", getStateName().c_str()); */
      /* } */
    }
  }
}

//}

/* callbackTimerCheckSubcribers //{ */


void BalloonCircleDestroy::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }
  ros::Time time_now = ros::Time::now();
  if (!got_odom_uav_) {
    ROS_WARN_THROTTLE(0.5, "[%s]: No odom msg since node launch", ros::this_node::getName().c_str());

  } else {
    if ((time_now - time_last_odom_uav_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(0.5, "[%s]: didnt receive any odom msg for %f secs", ros::this_node::getName().c_str(), (time_now - time_last_odom_uav_).toSec());
    }
  }

  /* check whether tracker diagnostics msgs are coming */
  if (!got_tracker_diag_) {
    ROS_WARN_THROTTLE(0.5, "Not received tracker diagnostics msg since node launch.");
  } else {
    if ((time_now - time_last_tracker_diagnostics_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(0.5, "Not received uav odom msg for %f sec.", (time_now - time_last_tracker_diagnostics_).toSec());
    }
  }

  /* check whether ground truth pose msgs are coming */
  if (_simulation_) {
    if (!got_odom_gt_) {
      ROS_WARN_THROTTLE(0.5, "Not received ground truth odom msg since node launch.");
    } else {
      if ((time_now - time_last_odom_gt_).toSec() > 1.0) {
        ROS_WARN_THROTTLE(0.5, "Not received ground truth odom msg for %f sec.", (time_now - time_last_odom_gt_).toSec());
      }
    }
  }

  /* check whether balloons msgs are coming */
  if (_planner_active_) {
    if (!got_balloon_point_) {
      ROS_WARN_THROTTLE(0.5, "[%s]: haven't received balloon point since start of estimation", ros::this_node::getName().c_str());
    } else {
      if ((time_now - time_last_balloon_point_).toSec() > 1.0) {
        ROS_WARN_THROTTLE(0.5, "[%s]: haven't received any balloon points from KF for %f", ros::this_node::getName().c_str(),
                          (time_now - time_last_balloon_point_).toSec());
      }
    }
  }

  if (!got_balloon_point_cloud_) {
    ROS_WARN_THROTTLE(0.5, "[%s]: haven't received balloon point cloud since launch", ros::this_node::getName().c_str());
  } else {
    if ((time_now - time_last_balloon_cloud_point_).toSec() > 1) {
      ROS_WARN_THROTTLE(0.5, "[%s]: haven't received any balloon cloud points for %f", ros::this_node::getName().c_str(),
                        (time_now - time_last_balloon_cloud_point_).toSec());
      is_ballon_cloud_incoming_ = false;
    } else {
      if (balloon_point_cloud_.points.size() > 0) {
        is_ballon_cloud_incoming_ = true;
      }
    }
  }
}

//}

/* callbackTimerCheckBalloonPoints //{ */


void BalloonCircleDestroy::callbackTimerCheckBalloonPoints([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }
  if (_planner_active_) {
    if (ros::Time::now().toSec() - time_last_balloon_point_.toSec() < 1) {
      is_ballon_incoming_ = true;
    } else {
      is_ballon_incoming_ = false;
    }
  }

  if (_is_state_machine_active_) {
    if (got_balloon_point_cloud_) {
      if (ros::Time::now().toSec() - time_last_balloon_cloud_point_.toSec() > _time_to_land_) {
        ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: State machine finished, landing");
        /* landAndEnd(); */
      }
    }
  }
  /* ROS_INFO_THROTTLE(1.0, "[%s]: Got balloon point at x %f ", ros::this_node::getName().c_str(), balloon_point_.pose.pose.position.x); */
}

//}

/* callbackTimerPublishRviz //{ */

void BalloonCircleDestroy::callbackTimerPublishRviz([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: isn't initialized");
    return;
  }

  {
    std::scoped_lock lock(mutex_rviz_);

    visualization_msgs::MarkerArray msg_out;

    int id = 0;

    for (uint i = 0; i < _forb_vect_.size(); i++) {


      visualization_msgs::Marker marker;
      marker.header.frame_id    = "local_origin";
      marker.header.stamp       = ros::Time::now();
      marker.ns                 = "mtsp";
      marker.id                 = id++;
      marker.type               = visualization_msgs::Marker::SPHERE;
      marker.action             = visualization_msgs::Marker::ADD;
      marker.pose.position.x    = _forb_vect_[i].vect_(0, 0);
      marker.pose.position.y    = _forb_vect_[i].vect_(1, 0);
      marker.pose.position.z    = _forb_vect_[i].vect_(2, 0);
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x            = _forbidden_radius_;
      marker.scale.y            = _forbidden_radius_;
      marker.scale.z            = _forbidden_radius_;
      marker.color.a            = 0.3;
      marker.color.r            = 1.0;
      marker.color.g            = 0.0;
      marker.color.b            = 0.0;

      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: publishing forbidden zones");

      msg_out.markers.push_back(marker);
    }
    if (_planner_active_) {
      visualization_msgs::Marker marker;
      marker.header.frame_id    = "local_origin";
      marker.header.stamp       = ros::Time::now();
      marker.ns                 = "mtsp";
      marker.id                 = id++;
      marker.type               = visualization_msgs::Marker::SPHERE;
      marker.action             = visualization_msgs::Marker::ADD;
      marker.pose.position.x    = _estimate_vect_(0, 0);
      marker.pose.position.y    = _estimate_vect_(1, 0);
      marker.pose.position.z    = _estimate_vect_(2, 0);
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x            = 2.0 * 3;
      marker.scale.y            = 2.0 * 3;
      marker.scale.z            = 6;
      marker.color.a            = 0.3;
      marker.color.r            = 0.0;
      marker.color.g            = 1.0;
      marker.color.b            = 0.0;

      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

      msg_out.markers.push_back(marker);
    }
    rviz_pub_.publish(msg_out);
  }
}


//}

/* callbackCircleAround //{ */

bool BalloonCircleDestroy::callbackCircleAround([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {
    res.success = false;
    res.message = "Circle Destroyer isn't initialized";

    ROS_WARN("[BalloonCircleDestroy]: couldn't start circling, I am not initialized ");

    return true;
  }
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Start circling, circle radius %f", _circle_radius_);
  getCloseToBalloon(balloon_vector_, _dist_to_balloon_, _vel_);
  mrs_msgs::TrackerTrajectory new_trj_;
  new_trj_.header.frame_id = world_frame_id_;
  new_trj_.header.stamp    = ros::Time::now();
  new_trj_.fly_now         = true;
  new_trj_.loop            = false;
  new_trj_.use_yaw         = true;
  new_trj_.start_index     = 0;
  double iterat            = M_PI / (_circle_accuracy_ / 2);
  double angle             = 0;

  for (int i = 0; i < _circle_accuracy_; i++) {
    mrs_msgs::TrackerPoint point;
    point.x   = balloon_vector_(0, 0) + cos(angle) * _circle_radius_;
    point.y   = balloon_vector_(1, 0) + sin(angle) * _circle_radius_;
    point.z   = balloon_vector_(2, 0);
    point.yaw = angle + M_PI;
    angle += iterat;

    new_trj_.points.push_back(point);
  }


  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroyer]: publishing trajectory ");
  mrs_msgs::TrackerTrajectorySrv req_;
  req_.request.trajectory_msg = new_trj_;

  srv_client_trajectory_.call(req_);


  res.success = req_.response.success;
  res.message = req_.response.message;
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Result of circle service %d ", res.success);
  is_tracking_ = true;
  return res.success;
}

//}

/* callbackGoCloser //{ */

bool BalloonCircleDestroy::callbackGoCloser([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {

    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Could'nt call the service, node isn't initialized");
    res.message = "Node isn't initialized";
    res.success = false;
    return false;
  }
  plannerActivate(getClosestBalloon(), _dist_to_balloon_);
  while (got_balloon_point_ == false || _planner_active_ == false) {
    ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: waiting for estimation");
    ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: x %f y %f z %f", balloon_vector_(0, 0), balloon_vector_(1, 0), balloon_vector_(2, 0));
  }

  getCloseToBalloon(balloon_vector_, _dist_to_balloon_, _vel_);
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: I am at the balloon at %f", _dist_to_balloon_);
  res.message = "Getting close to the balloon";
  res.success = true;

  return true;
}

//}

/* callbackStartStateMachine //{ */


bool BalloonCircleDestroy::callbackStartStateMachine([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    res.success = false;
    res.message = "Can't trigger service, not initialized";
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Could'nt call the service, not inited yet");

    return false;
  }
  if (_is_state_machine_active_) {
    _is_state_machine_active_ = false;
    res.message               = "State machine disabled";

  } else {
    _is_state_machine_active_ = true;
    res.message               = "State machine activated";
  }

  _state_     = IDLE;
  res.success = true;
  return true;
}

//}

/* callbackToggleDestroy //{ */


bool BalloonCircleDestroy::callbackToggleDestroy([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    res.success = false;
    res.message = "Can't trigger service, not initialized";
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Could'nt call the service, not inited yet");

    return false;
  }
  if (_is_destroy_enabled_) {
    _is_destroy_enabled_ = false;
    res.message          = "Destroy disabled";

  } else {
    _is_destroy_enabled_ = true;
    res.message          = "Destroy activated";
  }

  res.success = true;
  return true;
}

//}

/* circleAroundBalloon //{ */

void BalloonCircleDestroy::circleAroundBalloon() {

  if (!is_initialized_) {
    ROS_WARN("[BalloonCircleDestroy]: couldn't start circling, I am not initialized ");
    return;
  }


  if (_state_ != CIRCLE_AROUND) {
    ROS_INFO_THROTTLE(0.5, "[StateMachine]: State is not CIRCLE_AROUND but is %s", getStateName().c_str());
    return;
  }
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Start circling, circle radius %f", _circle_radius_);
  mrs_msgs::TrackerTrajectory new_trj_;
  new_trj_.header.frame_id = world_frame_id_;
  new_trj_.header.stamp    = ros::Time::now();
  new_trj_.fly_now         = true;
  new_trj_.loop            = false;
  new_trj_.use_yaw         = true;
  new_trj_.start_index     = 0;
  double iterat            = M_PI / (_circle_accuracy_ / 2);
  double angle             = getBalloonHeading(balloon_vector_) + M_PI;

  for (int i = 0; i < _circle_accuracy_; i++) {
    mrs_msgs::TrackerPoint point;
    point.x   = balloon_vector_(0, 0) + cos(angle) * _circle_radius_;
    point.y   = balloon_vector_(1, 0) + sin(angle) * _circle_radius_;
    point.z   = balloon_vector_(2, 0);
    point.yaw = angle + M_PI;
    angle += iterat;

    new_trj_.points.push_back(point);
  }


  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroyer]: publishing trajectory ");
  mrs_msgs::TrackerTrajectorySrv req_;
  req_.request.trajectory_msg = new_trj_;

  srv_client_trajectory_.call(req_);


  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Result of circle service %s ", req_.response.message.c_str());
  is_tracking_ = true;
}

//}

/* getCloseToBalloon //{ */

void BalloonCircleDestroy::getCloseToBalloon(Eigen::Vector3d dest_, double close_dist_, double speed_) {

  double          sample_dist_ = speed_ * (_traj_time_ / _traj_len_);
  Eigen::Vector3d dir_vector_  = dest_ - odom_vector_;

  double          dist_   = dir_vector_.norm();
  double          normed_ = (dist_ - close_dist_) / dist_;
  Eigen::Vector3d goal_   = normed_ * dir_vector_ + odom_vector_;
  if (dest_(2, 0) < _min_height_) {
    goal_(2, 0) = _min_height_ + _height_offset_;
  }
  if (dest_(2, 0) > _max_height_) {
    goal_(2, 0) = _max_height_ + _height_offset_;
  }
  dir_vector_ = goal_ - odom_vector_;

  dist_       = dir_vector_.norm();
  dir_vector_ = (dir_vector_ / dist_) * sample_dist_;

  Eigen::Vector3d             cur_pos_ = odom_vector_;
  mrs_msgs::TrackerTrajectory new_traj_;
  Eigen::Vector3d             diff_vector_;
  double                      angle_ = getBalloonHeading(dest_);

  while (cur_pos_(0, 0) != goal_(0, 0) && cur_pos_(1, 0) != goal_(1, 0) && cur_pos_(2, 0) != goal_(2, 0)) {

    for (int i = 0; i < _traj_len_; i++) {
      mrs_msgs::TrackerPoint p;
      cur_pos_     = cur_pos_ + dir_vector_;
      diff_vector_ = cur_pos_ - odom_vector_;

      if (diff_vector_.norm() >= dist_) {
        cur_pos_ = goal_;
      }

      p.x   = cur_pos_(0, 0);
      p.y   = cur_pos_(1, 0);
      p.z   = cur_pos_(2, 0);
      p.yaw = angle_;


      new_traj_.points.push_back(p);
    }
  }
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Trajectory ready ");
  new_traj_.header.frame_id = world_frame_id_;
  new_traj_.header.stamp    = ros::Time::now();
  new_traj_.fly_now         = true;
  new_traj_.use_yaw         = true;
  new_traj_.loop            = false;
  new_traj_.start_index     = 0;
  mrs_msgs::TrackerTrajectorySrv req_;
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: calling service to get closer ");
  req_.request.trajectory_msg = new_traj_;
  srv_client_trajectory_.call(req_);
  is_tracking_ = true;
}

//}

/* getBalloonHeading //{ */

double BalloonCircleDestroy::getBalloonHeading(Eigen::Vector3d dest_) {

  Eigen::Vector3d angle_vector_ = dest_ - odom_vector_;

  return atan2(angle_vector_(1), angle_vector_(0));
}


//}

/* goAroundArena //{ */

void BalloonCircleDestroy::goAroundArena() {

  if (!is_initialized_) {
    ROS_WARN("[BalloonCircleDestroy]: couldn't start going around arena, I am not initialized ");

    return;
  }
  if (!_is_state_machine_active_) {
    return;
  }

  if (_state_ != State::GOING_AROUND) {

    if (_state_ != State::GOING_TO_ANGLE) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Can't circle around arena, drone state isn't GOING_AROUND or GOING_TO_ANGLE but is %s ",
                        getStateName().c_str());
      return;
    }
  }


  mrs_msgs::TrackerTrajectory new_trj_;
  new_trj_.header.frame_id = world_frame_id_;
  new_trj_.header.stamp    = ros::Time::now();
  new_trj_.fly_now         = true;
  new_trj_.loop            = false;
  new_trj_.use_yaw         = true;
  new_trj_.start_index     = 0;

  double mpc_speed_ = _traj_time_ / _traj_len_;

  double arena_accuracy_ = _cur_arena_length_ * _cur_arena_width_ * 2 * M_PI;
  arena_accuracy_        = arena_accuracy_ / _vel_arena_ * mpc_speed_;
  double iterat_         = M_PI / (arena_accuracy_ / 2);
  /* double iterat_            = M_PI / (_arena_accuracy_ / 2); */


  Eigen::Vector3d angle_vector_ = Eigen::Vector3d(_arena_center_x_, _arena_center_y_, _height_) - odom_vector_;
  double          angle         = atan2(angle_vector_(1), angle_vector_(0)) + M_PI;
  ROS_INFO("[]: angle %f accuracy %f", angle, arena_accuracy_);
  /* double angle = 0; */
  bool lower = false;
  if (angle > _closest_angle_) {
    lower = true;
  }

  for (int i = 0; i < arena_accuracy_; i++) {
    mrs_msgs::TrackerPoint point;
    point.x   = _arena_center_x_ + cos(angle) * _cur_arena_width_ / 2;
    point.y   = _arena_center_y_ + sin(angle) * _cur_arena_length_ / 2;
    point.z   = _height_;
    point.yaw = angle + M_PI;
    if (_state_ == State::GOING_TO_ANGLE && lower) {
      angle -= iterat_;
    } else {
      angle += iterat_;
    }
    new_trj_.points.push_back(point);
    // if state is going to angle  we should stop at the closest balloon found
    if (_state_ == State::GOING_TO_ANGLE) {
      if (lower && angle < _closest_angle_) {
        break;
      }
      if (angle > _closest_angle_ && !lower) {
        break;
      }
    }
  }


  ROS_WARN_THROTTLE(0.5, "[Traj]: %lu ", new_trj_.points.size());
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroyer]: publishing trajectory ");
  mrs_msgs::TrackerTrajectorySrv req_;
  req_.request.trajectory_msg = new_trj_;

  srv_client_trajectory_.call(req_);
  is_tracking_ = true;
}

//}

/* goToChosenBalloon //{ */


void BalloonCircleDestroy::goToChosenBalloon() {
  if (!is_initialized_) {
    return;
  }

  if (_state_ != State::GOING_TO_BALLOON) {
    ROS_INFO_THROTTLE(0.5, "[StateMachine]: State is not GOINT TO BALLOON, but is %s", getStateName().c_str());
  }

  double          sample_dist_ = _vel_ * (_traj_len_ / _traj_time_);
  Eigen::Vector3d dir_vector_  = balloon_vector_ - odom_vector_;

  double          dist_   = dir_vector_.norm();
  double          normed_ = (dist_ - _dist_to_balloon_) / dist_;
  Eigen::Vector3d goal_   = normed_ * dir_vector_ + odom_vector_;
  goal_(2, 0)             = balloon_vector_(2, 0);
  dir_vector_             = goal_ - odom_vector_;

  dist_       = dir_vector_.norm();
  dir_vector_ = (dir_vector_ / dist_) * sample_dist_;

  Eigen::Vector3d             cur_pos_ = odom_vector_;
  mrs_msgs::TrackerTrajectory new_traj_;
  Eigen::Vector3d             diff_vector_;
  double                      angle_ = getBalloonHeading(balloon_vector_) + M_PI;

  while (cur_pos_(0, 0) != goal_(0, 0) && cur_pos_(1, 0) != goal_(1, 0) && cur_pos_(2, 0) != goal_(2, 0)) {

    for (int i = 0; i < _traj_len_; i++) {
      mrs_msgs::TrackerPoint p;
      cur_pos_     = cur_pos_ + dir_vector_;
      diff_vector_ = cur_pos_ - odom_vector_;

      if (diff_vector_.norm() >= dist_) {
        cur_pos_ = goal_;
      }

      p.x   = cur_pos_(0, 0);
      p.y   = cur_pos_(1, 0);
      p.z   = cur_pos_(2, 0);
      p.yaw = angle_;

      new_traj_.points.push_back(p);
    }
  }
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Trajectory ready ");
  new_traj_.header.frame_id = world_frame_id_;
  new_traj_.header.stamp    = ros::Time::now();
  new_traj_.fly_now         = true;
  new_traj_.use_yaw         = true;
  new_traj_.loop            = false;
  new_traj_.start_index     = 0;
  mrs_msgs::TrackerTrajectorySrv req_;
  req_.request.trajectory_msg = new_traj_;
  srv_client_trajectory_.call(req_);
  is_tracking_ = true;
}

//}

/* getArenaHeading //{ */

double BalloonCircleDestroy::getArenaHeading() {
  Eigen::Vector3d angle_vector_ = Eigen::Vector3d(_arena_center_x_, _arena_center_y_, _height_) - odom_vector_;
  return atan2(angle_vector_(1), angle_vector_(0)) + M_PI;
}

//}

/* getStateName //{ */

std::string BalloonCircleDestroy::getStateName() {
  switch (_state_) {
    case IDLE:
      return "IDLE";
    case GOING_AROUND:
      return "GOING_AROUND";
    case GOING_TO_ANGLE:
      return "GOING_TO_ANGLE";
    case CHOOSING_BALLOON:
      return "CHOOSING_BALLOON";
    case GOING_TO_BALLOON:
      return "GOING_TO_BALLOON";
    case AT_BALLOON:
      return "AT_BALLOON";
    case CIRCLE_AROUND:
      return "CIRCLE_AROUND";
    case DESTROYING:
      return "DESTROYING";
    case CHECKING_BALLOON:
      return "CHECKING_BALLOON";
    case READY_TO_DESTROY:
      return "READY_TO_DESTROY";
  }
}
//}

// | ------------------- Estimation services ------------------ |

/* plannerActivate //{ */

void BalloonCircleDestroy::plannerActivate(Eigen::Vector3d estimation_, double radius_) {
  if (_planner_active_) {
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroyi]: planner is already active, I'll reset it ");
    /* plannerReset(); */
    return;
  }

  balloon_planner::StartEstimation        req_;
  balloon_planner::StartEstimationRequest rq_;

  geometry_msgs::Point p_;
  _estimate_vect_  = estimation_;
  p_.x             = estimation_(0, 0);
  p_.y             = estimation_(1, 0);
  p_.z             = estimation_(2, 0);
  rq_.inital_point = p_;
  rq_.radius       = radius_;
  req_.request     = rq_;
  if (srv_planner_start_estimation_.call(req_)) {
    if (req_.response.success) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner activated at point x %f. y %f. z %f", p_.x, p_.y, p_.z);
      _planner_active_ = true;
    } else {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner haven't been activated  at point x %f. y %f. z %f", p_.x, p_.y, p_.z);
    }

  } else {
    ROS_ERROR_THROTTLE(0.5, "[BalloonCircleDestroy]: Failed at calling planner activation service  ");
  }
}

//}

/* plannerStop //{ */

void BalloonCircleDestroy::plannerStop() {
  if (!_planner_active_) {
    ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner isn't active, can't stop");
    return;
  }

  std_srvs::Trigger req_;

  if (srv_planner_stop_estimation_.call(req_)) {
    if (req_.response.success) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner stopped");
      _planner_active_ = false;
    } else {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner haven't been stopped");
    }

  } else {
    ROS_ERROR_THROTTLE(0.5, "[BalloonCircleDestroy]: Failed at calling planner stopping service");
  }
}


//}

/* plannerReset //{ */

bool BalloonCircleDestroy::plannerReset() {
  if (!_planner_active_) {
    ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: couldn't reset, planner isn't active");
  }

  if (_state_ == CHOOSING_BALLOON) {
    if (ros::Time::now().toSec() - time_last_planner_reset_.toSec() < _wait_for_ball_) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: can't reset, too soon");
      return false;
    }
  }
  std_srvs::Trigger req_;

  if (srv_planner_reset_estimation_.call(req_)) {
    if (req_.response.success) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner reset");
      time_last_planner_reset_ = ros::Time::now();
      return true;

    } else {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner couldn't been reset, msg: %s", req_.response.message.c_str());
      return false;
    }

  } else {
    ROS_ERROR_THROTTLE(0.5, "[BalloonCircleDestroy]: Failed at calling planner reset service");
    return false;
  }
}

//}

/* addForbidden //{ */

void BalloonCircleDestroy::addForbidden(Eigen::Vector3d forb_, double radius_) {

  bool is_in_rad_ = false;
  for (int i = 0; i < int(_forb_vect_.size()); i++) {
    if ((_forb_vect_.at(i).vect_ - forb_).norm() < radius_) {
      is_in_rad_ = true;
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Couldn't add forbidden zone, already in the zone of another ");
    }
  }

  Forbidden_t forb_t_;
  forb_t_.r     = radius_;
  forb_t_.vect_ = forb_;
  _forb_vect_.push_back(forb_t_);
  balloon_planner::AddExclusionZone        req_;
  balloon_planner::AddExclusionZoneRequest rq_;

  geometry_msgs::Point p_;
  p_.x            = forb_(0, 0);
  p_.y            = forb_(1, 0);
  p_.z            = forb_(2, 0);
  rq_.zone_center = p_;
  rq_.zone_radius = radius_;

  if (srv_planner_add_zone_.call(req_)) {
    if (req_.response.success) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner added exclusion zone at point x %f. y %f. z %f with radius %f", p_.x, p_.y, p_.z, radius_);
      _planner_active_ = true;
    } else {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Planner couldn't add exclusion zone  at point x %f, y %f, z %f with radius %f", p_.x, p_.y, p_.z,
                        radius_);
    }

  } else {
    ROS_ERROR_THROTTLE(0.5, "[BalloonCircleDestroy]: Failed at calling planner add exculsive zone service  ");
  }
}

//}

/* pointInForbidden //{ */

bool BalloonCircleDestroy::pointInForbidden(Eigen::Vector3d vect_) {
  for (uint i = 0; i < _forb_vect_.size(); i++) {
    if ((vect_ - _forb_vect_[i].vect_).norm() < _forb_vect_[i].r) {
      return true;
    }
  }
  return false;
}
//}

/* checkForbidden //{ */

void BalloonCircleDestroy::checkForbidden() {
  if ((_prev_closest_ - balloon_closest_vector_).norm() < _dist_error_) {
    if (_balloon_tries_ == _balloon_try_count_) {
      addToForbidden(balloon_closest_vector_);
      _balloon_try_count_ = 0;
    } else {
      _balloon_try_count_++;
    }
  } else {
    _balloon_try_count_ = 0;
    _prev_closest_      = balloon_closest_vector_;
  }
}

//}

/* addToForbidden //{ */

void BalloonCircleDestroy::addToForbidden(Eigen::Vector3d dest_) {
  Forbidden_t forb_;
  forb_.vect_ = dest_;
  forb_.r     = _forbidden_radius_;
  _forb_vect_.push_back(forb_);
}

//}

/* balloonOutdated //{ */

bool BalloonCircleDestroy::balloonOutdated() {
  if (ros::Time::now().toSec() - time_last_planner_reset_.toSec() > _wait_for_ball_) {
    return ros::Time::now().toSec() - balloon_point_.header.stamp.toSec() > _wait_for_ball_;
  }
  return false;
}

//}

/* landAndEnd //{ */

void BalloonCircleDestroy::landAndEnd() {
  _is_state_machine_active_ = false;
  std_srvs::Trigger srv_land_call;
  srv_client_land_.call(srv_land_call);
}

//}

/* getClosestBalloon //{ */

Eigen::Vector3d BalloonCircleDestroy::getClosestBalloon() {

  double          dist_;
  double          best_dist_ = 999;
  Eigen::Vector3d ball_vect_;
  Eigen::Vector3d ball_vect_best_;
  {
    std::scoped_lock lock_uav(mutex_odom_uav_);
    std::scoped_lock lock_balloon(mutex_is_balloon_cloud_incoming_);

    for (uint8_t i = 0; i < balloon_point_cloud_.points.size(); i++) {

      geometry_msgs::Point p_;

      ROS_INFO_THROTTLE(0.5, "[ishiiit]: ");
      bool ts_res = transformPointFromWorld(balloon_point_cloud_.points.at(i), balloon_point_cloud_.header.frame_id, balloon_point_cloud_.header.stamp, p_);
      if (!ts_res) {
        ROS_WARN_THROTTLE(1, "[BalloonCircleDestroy]: No transform,skipping");
        break;
      }
      if (p_.z > _max_height_ || p_.z < _min_height_) {
        continue;
      }
      ball_vect_(0, 0) = p_.x;
      ball_vect_(1, 0) = p_.y;
      ball_vect_(2, 0) = p_.z;

      ROS_INFO_THROTTLE(1.0, "[StateMachine]: closest ball %f, %f, %f ", ball_vect_(0, 0), ball_vect_(1, 0), ball_vect_(2, 0));
      dist_ = (odom_vector_ - ball_vect_).norm();
      if (dist_ < best_dist_) {

        ball_vect_best_ = ball_vect_;
      }
    }
  }
  ROS_INFO_THROTTLE(1.0, "[StateMachine]: closest ball %f, %f, %f ", ball_vect_best_(0, 0), ball_vect_best_(1, 0), ball_vect_best_(2, 0));
  return ball_vect_best_;
}

//}

/* callbackDynamicReconfigure //{ */

void BalloonCircleDestroy::callbackDynamicReconfigure([[maybe_unused]] Config& config, uint32_t level) {
  {
    std::scoped_lock lock(mutex_dynamic_reconfigure_);
    _height_offset_ = config.height_offset;
  }
}

//}

// | --------------------- transformations -------------------- |
/* getTransform() method //{ */
bool BalloonCircleDestroy::getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& stamp,
                                        geometry_msgs::TransformStamped& transform_out) {
  try {
    transform_out = tf_buffer_.lookupTransform(to_frame, from_frame, stamp);
    return true;
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", from_frame.c_str(), to_frame.c_str(),
                      ex.what());
    return false;
  }
}
//}

/* transformPointFromWorld() method //{ */
bool BalloonCircleDestroy::transformPointFromWorld(const geometry_msgs::Point32& point, const std::string& to_frame, const ros::Time& stamp,
                                                   geometry_msgs::Point& point_out) {
  geometry_msgs::Point p_;
  p_.x = point.x;
  p_.y = point.y;
  p_.z = point.z;
  geometry_msgs::TransformStamped transform;
  if (!getTransform(to_frame, world_frame_id_, stamp, transform))
    return false;


  tf2::doTransform(p_, point_out, transform);
  return true;
}
//}
//
}  // namespace balloon_circle_destroy

PLUGINLIB_EXPORT_CLASS(balloon_circle_destroy::BalloonCircleDestroy, nodelet::Nodelet);
