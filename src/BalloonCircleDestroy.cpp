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

  mrs_lib::ParamLoader param_loader(nh, "BalloonCircleDestroy");

  // | ----------------------- transformer ---------------------- |
  param_loader.load_param("uav_name", _uav_name_);
  transformer_ = mrs_lib::Transformer("BalloonCircleDestroy", _uav_name_);

  // | ------------------- load ros parameters ------------------ |
  /* params //{ */

  param_loader.load_param("elips_height", _height_);
  param_loader.load_param("height_tol", _height_tol_);
  param_loader.load_param("idle_time", _idle_time_);
  param_loader.load_param("traj_time", _traj_time_);
  param_loader.load_param("traj_len", _traj_len_);
  param_loader.load_param("dist_to_balloon", _dist_to_balloon_);
  param_loader.load_param("dist_to_overshoot", _dist_to_overshoot_);
  param_loader.load_param("dist_kf_activation", _dist_kf_activation_);
  param_loader.load_param("overshoot_offset", _overshoot_offset_);
  param_loader.load_param("vel", _vel_);
  param_loader.load_param("vel_attack", _vel_attack_);
  param_loader.load_param("vel_arena", _vel_arena_);
  param_loader.load_param("dist_error", _dist_error_);
  param_loader.load_param("dist_acc", _dist_acc_);
  param_loader.load_param("wait_for_ball", _wait_for_ball_);
  param_loader.load_param("time_to_emulate", _time_to_emulate_);

  param_loader.load_param("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.load_param("rate/check_state_machine", _rate_timer_check_state_machine_);
  param_loader.load_param("rate/check_balloons", _rate_timer_check_balloons_);
  param_loader.load_param("rate/state_machine", _rate_timer_state_machine_);
  param_loader.load_param("rate/pub_rviz", _rate_time_publish_rviz_);
  param_loader.load_param("rate/pub_status", _rate_time_publish_status_);
  param_loader.load_param("time_thresholds/going_to", time_to_going_to);
  param_loader.load_param("time_thresholds/destroy", time_to_destroy);
  param_loader.load_param("time_thresholds/checking_balloon", time_to_check_balloon);

  param_loader.load_param("world_frame_id", world_frame_id_);
  param_loader.load_param("safety_area_frame", safety_area_frame_);
  param_loader.load_param("untilted_frame_id", untilted_frame_id_);
  param_loader.load_param("reset_tries", _reset_tries_);
  param_loader.load_param("balloon_tries", _balloon_tries_);
  param_loader.load_param("time_to_land", _time_to_land_);
  param_loader.load_param("forbidden_radius", _forbidden_radius_);
  param_loader.load_param("height_offset", _height_offset_);
  param_loader.load_param("max_time_balloon", _max_time_balloon_);
  param_loader.load_param("area/x_min", _x_min_);
  param_loader.load_param("area/x_max", _x_max_);
  param_loader.load_param("area/y_min", _y_min_);
  param_loader.load_param("area/y_max", _y_max_);
  param_loader.load_param("area/z_min", _z_min_);
  param_loader.load_param("area/z_max", _z_max_);
  param_loader.load_param("area/offset", _arena_offset_);
  param_loader.load_param("dead_band_factor", _dead_band_factor_);
  param_loader.load_param("balloon_activation_dist", _balloon_activation_dist_);
  param_loader.load_param("arena_scan_step", _fov_step_);
  param_loader.load_param("arena_corner_factor", _arena_corner_factor_);
  param_loader.load_matrix_static("arena", _arenas_);
  param_loader.load_matrix_dynamic("safe1", safety_1, -1, 2);
  param_loader.load_matrix_dynamic("safe2", safety_2, -1, 2);
  param_loader.load_matrix_dynamic("safe_big", safety_, -1, 2);
  param_loader.load_param("constraints/sweeping", _sweep_constraints_);
  param_loader.load_param("constraints/going", _attack_constraints_);
  param_loader.load_param("arena_swap_time", _arena_time_);
  param_loader.load_param("is_arena_swap_enabled", _do_swap_);
  param_loader.load_param("cheating_mode", _cheating_mode_);
  param_loader.load_param("cheating_height", _cheating_height_);
  param_loader.load_param("max_silence_time", _max_silence_time_);

  /* safety_polygon_ = std::make_shared<mrs_lib::Polygon>(safety_); */

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[BalloonCircleDestroy]: Couldn't load all params, shutdown");
    ros::shutdown();
  }


  ROS_INFO_STREAM_ONCE("[BalloonCircleDestroy]: params loaded");
  if (_x_min_ > _x_max_ || _y_min_ > _y_max_) {
    ROS_ERROR("[BalloonCircleDestroy]: Arena params are wrong, please check them once again, shutdown");
    ros::shutdown();
  }


  _cur_arena_width_ = std::abs(_x_max_ - _x_min_) - _arena_offset_;
  /* _cur_arena_length_ = std::abs(_x_max_ - _x_min_); */
  _cur_arena_length_ = std::abs(_y_max_ - _y_min_) - _arena_offset_;
  _arena_center_x_   = (_x_min_ + _x_max_) / 2;
  _arena_center_y_   = (_y_min_ + _y_max_) / 2;


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

  sub_odom_uav_     = nh.subscribe("odom_uav_in", 1, &BalloonCircleDestroy::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());
  sub_tracker_diag_ = nh.subscribe("tracker_diagnostics_in", 1, &BalloonCircleDestroy::callbackTrackerDiag, this, ros::TransportHints().tcpNoDelay());
  subscriber_constraints_diag_ =
      nh.subscribe("constraints_diagnostics_in", 1, &BalloonCircleDestroy::callbackConstraintsDiag, this, ros::TransportHints().tcpNoDelay());
  sub_balloon_point_ = nh.subscribe("balloon_point_in", 1, &BalloonCircleDestroy::callbackBalloonPoint, this, ros::TransportHints().tcpNoDelay());
  sub_balloon_point_cloud_ =
      nh.subscribe("balloon_point_cloud_in", 1, &BalloonCircleDestroy::callbackBalloonPointCloud, this, ros::TransportHints().tcpNoDelay());
  sub_realsense = nh.subscribe("realsense_camera_info", 1, &BalloonCircleDestroy::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());

  sub_arena_ = nh.subscribe("arena_info", 1, &BalloonCircleDestroy::callbackArenaInfo, this, ros::TransportHints().tcpNoDelay());

  sub_comrade_tracker_diag_ =
      nh.subscribe("comrade_diagnostics", 1, &BalloonCircleDestroy::callbackComradeTrackerDiag, this, ros::TransportHints().tcpNoDelay());
  //}

  // | ----------------------- Publishers ----------------------- |

  rviz_pub_   = nh.advertise<visualization_msgs::MarkerArray>("rviz_out", 1);
  status_pub_ = nh.advertise<std_msgs::String>("status_out", 1);

  point_pub_   = nh.advertise<geometry_msgs::PointStamped>("ref_out", 1);
  balloon_pub_ = nh.advertise<geometry_msgs::PointStamped>("ball_out", 1);

  // | --------------- initialize service servers --------------- |
  /*  server services //{ */

  srv_server_start_state_machine_ = nh.advertiseService("start_state_machine", &BalloonCircleDestroy::callbackStartStateMachine, this);
  srv_server_stop_state_machine_  = nh.advertiseService("stop_state_machine", &BalloonCircleDestroy::callbackStopStateMachine, this);
  srv_server_toggle_destroy_      = nh.advertiseService("toggle_destroy", &BalloonCircleDestroy::callbackToggleDestroy, this);
  srv_server_reset_zones_         = nh.advertiseService("reset_forbidden_zones", &BalloonCircleDestroy::callbackResetZones, this);
  srv_server_auto_start_          = nh.advertiseService("auto_start", &BalloonCircleDestroy::callbackAutoStart, this);


  //}

  /* Service clients //{ */


  // init service client for publishing trajectories for the drone
  srv_client_trajectory_        = nh.serviceClient<mrs_msgs::TrackerTrajectorySrv>("trajectory_srv");
  srv_planner_reset_estimation_ = nh.serviceClient<std_srvs::Trigger>("reset_estimation");
  srv_client_stop_              = nh.serviceClient<std_srvs::Trigger>("drone_stop");
  srv_set_constriants_          = nh.serviceClient<mrs_msgs::String>("set_constraints_out");
  srv_eland_                    = nh.serviceClient<std_srvs::Trigger>("eland");
  srv_planner_start_estimation_ = nh.serviceClient<balloon_filter::StartEstimation>("start_estimation");
  srv_planner_stop_estimation_  = nh.serviceClient<std_srvs::Trigger>("stop_estimation");
  srv_planner_add_zone_         = nh.serviceClient<balloon_filter::AddExclusionZone>("add_zone");
  srv_planner_reset_zones_      = nh.serviceClient<std_srvs::Trigger>("reset_zones");


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
  timer_state_machine_     = nh.createTimer(ros::Rate(_rate_timer_state_machine_), &BalloonCircleDestroy::callbackTimerStateMachine, this, false, true);
  timer_check_state_machine_ =
      nh.createTimer(ros::Rate(_rate_timer_check_state_machine_), &BalloonCircleDestroy::callbackTimerCheckStateMachine, this, false, true);
  timer_check_balloons_ = nh.createTimer(ros::Rate(_rate_timer_check_balloons_), &BalloonCircleDestroy::callbackTimerCheckBalloonPoints, this, false, true);
  timer_publish_rviz_   = nh.createTimer(ros::Rate(_rate_time_publish_rviz_), &BalloonCircleDestroy::callbackTimerPublishRviz, this, false, true);
  timer_publish_status_ = nh.createTimer(ros::Rate(_rate_time_publish_status_), &BalloonCircleDestroy::callbackTimerPublishStatus, this, false, true);

  // you can disable autostarting of the timer by the last argument

  //}
  //
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  ROS_INFO_ONCE("[BalloonCircleDestroy]: initialized");

  safety_area_init_timer_ = nh.createTimer(ros::Duration(1.0), &BalloonCircleDestroy::callbackTimerInitSafetyArea, this);
}


//}

/* callbackInitSafetyArea //{ */

void BalloonCircleDestroy::callbackTimerInitSafetyArea(const ros::TimerEvent& te) {
  ROS_INFO("[ArenaInit]: called");
  geometry_msgs::TransformStamped trans_out;

  const auto tf_opt = transformer_.getTransform(safety_area_frame_, world_frame_id_);

  /* bool       trans  = getTransform(arena_params_.header.frame_id, world_frame_id_, arena_params_.header.stamp, trans_out); */

  if (!tf_opt.has_value()) {
    ROS_ERROR("Safety area could not be transformed!");
    return;
  }
  const auto tf = tf_opt.value();

  /* transform safety_1 //{ */

  {
    for (int it = 0; it < safety_1.rows(); it++) {
      Eigen::Vector2d      vec = safety_1.row(it);
      geometry_msgs::Point pt;
      pt.x     = vec.x();
      pt.y     = vec.y();
      pt.z     = 0.0;
      auto tfd = transformer_.transformHeaderless(tf, pt);
      if (!tfd.has_value()) {
        ROS_ERROR("Safety area 1 could not be transformed!");
        return;
      } else {
        safety_1.row(it).x() = tfd.value().x;
        safety_1.row(it).y() = tfd.value().y;
      }
    }
  }

  //}

  ///* transform safety_2 //{ */

  {
    for (int it = 0; it < safety_2.rows(); it++) {
      Eigen::Vector2d      vec = safety_2.row(it);
      geometry_msgs::Point pt;
      pt.x     = vec.x();
      pt.y     = vec.y();
      pt.z     = 0.0;
      auto tfd = transformer_.transformHeaderless(tf, pt);
      if (!tfd.has_value()) {
        ROS_ERROR("Safety area 2  could not be transformed!");
        return;
      } else {
        safety_2.row(it).x() = tfd.value().x;
        safety_2.row(it).y() = tfd.value().y;
      }
    }
  }

  //}

  ///* transform safety_big //{ */

  {
    for (int it = 0; it < safety_.rows(); it++) {
      Eigen::Vector2d      vec = safety_.row(it);
      geometry_msgs::Point pt;
      pt.x     = vec.x();
      pt.y     = vec.y();
      pt.z     = 0.0;
      auto tfd = transformer_.transformHeaderless(tf, pt);
      if (!tfd.has_value()) {
        ROS_ERROR("Big Safety area could not be transformed!");
        return;
      } else {
        safety_.row(it).x() = tfd.value().x;
        safety_.row(it).y() = tfd.value().y;
      }
    }
  }

  //}

  safety_polygon_  = std::make_shared<mrs_lib::Polygon>(safety_);
  safety_polygon_1 = std::make_shared<mrs_lib::Polygon>(safety_1);
  safety_polygon_2 = std::make_shared<mrs_lib::Polygon>(safety_2);
  safety_area_init_timer_.stop();
  is_initialized_ = true;
}

//}

/* callbackOdomUav //{ */

void BalloonCircleDestroy::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }
  {
    std::scoped_lock lock(mutex_odom_uav_);
    odom_uav_ = *msg;
    geometry_msgs::Point      p_;
    geometry_msgs::Quaternion q_untilted;
    //  sqrt(x**2+y**2+z**2)
    /* _speed_ = std::sqrt(odom_uav_.twist.twist.linear.x * odom_uav_.twist.twist.linear.x + odom_uav_.twist.twist.linear.y * odom_uav_.twist.twist.linear.y +
     */
    /*                     odom_uav_.twist.twist.linear.z * odom_uav_.twist.twist.linear.z); */

    try {

      if (transformPointToWorld(odom_uav_.pose.pose.position, msg->header.frame_id, msg->header.stamp, p_)) {

        odom_vector_ = eigen_vect(p_.x, p_.y, p_.z);
      }

      if (transformQuaternionToUntilted(odom_uav_.pose.pose.orientation, odom_uav_.header.frame_id, msg->header.stamp, q_untilted)) {
        tf2::Quaternion q(q_untilted.x, q_untilted.y, q_untilted.z, q_untilted.w);
        tf2::Matrix3x3  m(q);
        double          roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        odom_yaw_ = yaw;
      }
    }
    catch (tf2::ExtrapolationException e) {
      return;
    }
  }

  // transform the uav_odom to arena frame
  {
    geometry_msgs::Vector3Stamped uav_velocity_control_frame_;
    uav_velocity_control_frame_.header   = msg->header;
    uav_velocity_control_frame_.vector.x = msg->twist.twist.linear.x;
    uav_velocity_control_frame_.vector.y = msg->twist.twist.linear.y;
    uav_velocity_control_frame_.vector.z = msg->twist.twist.linear.z;

    auto res = transformer_.transformSingle(world_frame_id_, uav_velocity_control_frame_);

    if (res) {

      std::scoped_lock lock(mutex_odom_uav_);

      uav_velocity_arena_frame_ = res.value();

    } else {

      ROS_WARN_THROTTLE(1.0, "[transformer]: could not transform cmd_odom to the arena frame");
    }
  }


  if (!got_odom_uav_) {
    got_odom_uav_ = true;
    ROS_INFO("[%s]: Got first odom", ros::this_node::getName().c_str());
  }

  time_last_odom_uav_ = ros::Time::now();
}

//}

/* callbackBalloonPoint //{ */

void BalloonCircleDestroy::callbackBalloonPoint(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  if (!is_initialized_) {
    ROS_INFO("[BalloonCircleDestroy]: not inited");
    return;
  }

  {
    std::scoped_lock lock(mutex_is_balloon_incoming_);

    balloon_point_ = *msg;
    geometry_msgs::Point p_;
    /* const std::string& to_frame, const ros::Time& stamp, */
    /*                                               geometry_msgs::Point& point_out */
    try {


      if (transformPointToWorld(balloon_point_.pose.pose.position, msg->header.frame_id, msg->header.stamp, p_)) {
        balloon_vector_ = eigen_vect(p_.x, p_.y, p_.z);
      }
    }
    catch (tf2::ExtrapolationException e) {
      return;
    }
  }

  time_last_balloon_point_ = ros::Time::now();
  if (!got_balloon_point_) {
    got_balloon_point_ = true;
    ROS_INFO("[%s]: got first balloon point", ros::this_node::getName().c_str());
  }
}

//}

/* callbackBalloonPointCloud //{ */

void BalloonCircleDestroy::callbackBalloonPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }
  if (!_is_state_machine_active_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_is_balloon_cloud_incoming_);
    balloon_pcl_processed_.clear();

    sensor_msgs::PointCloud2 pcl2_ = *msg;
    PC                       cloud_out;
    PC::Ptr                  cloud_in(new PC());
    pcl::fromROSMsg(*msg, *cloud_in);
    bool ts_res = transformPclToWorld(cloud_in, msg->header.frame_id, msg->header.stamp, cloud_out);
    if (!ts_res) {
      ROS_WARN("[BalloonPclCallback]: skipping pcl, no tf");
      return;
    }

    for (unsigned i = 0; i < cloud_out.points.size(); i++) {
      const auto x_ = cloud_out.points.at(i).x;
      const auto y_ = cloud_out.points.at(i).y;
      const auto z_ = cloud_out.points.at(i).z;
      if (pointInSafety(x_, y_, z_) && !pointInForbidden(eigen_vect(x_, y_, z_))) {
        balloon_pcl_processed_.push_back(eigen_vect(x_, y_, z_));
      }
      /* } else { */
      /*   ROS_INFO("[]: out of arena: [%f, %f, %f]", x_, y_, z_); */
      /* } */
    }


    if (balloon_pcl_processed_.size() > 0) {
      if (!got_balloon_point_cloud_) {
        got_balloon_point_cloud_ = true;
        ROS_INFO("[%s]: got first balloon point cloud", ros::this_node::getName().c_str());
      }
      time_last_balloon_cloud_point_ = ros::Time::now();
    }
  }
}  // namespace balloon_circle_destroy


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


  if (_state_ == DESTROY_OVERSHOOT) {
    if ((odom_vector_ - _last_goal_).norm() < _dist_acc_) {
      _last_goal_reached_ = true;
    }
  }

  if (is_tracking_ && !msg->tracking_trajectory) {
    ROS_INFO("[%s]: reached final point", ros::this_node::getName().c_str());
    {
      std::scoped_lock lock(mutex_is_tracking_);
      is_tracking_ = false;
      is_idling_   = true;
      ros::NodeHandle nh("~");
      timer_idling_ = nh.createTimer(ros::Duration(_idle_time_), &BalloonCircleDestroy::callbackTimerIdling, this,
                                     true);  // the last boolean argument makes the timer run only once
      ROS_INFO("[BalloonCircleDestroy]: Idling for %2.2f seconds.", _idle_time_);
    }
  }

  time_last_tracker_diagnostics_ = ros::Time::now();
}


//}

/* callbackComradeTrackerDiagnostics //{ */

void BalloonCircleDestroy::callbackComradeTrackerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {
  {
    if (!is_initialized_) {
      return;
    }
    if (revenge_mode) {
      return;
    }
    ROS_INFO_THROTTLE(1.0, "[Comrade]: Ya topolya, do you copy? ");
    std::scoped_lock lock(is_comrade_tracking_);
    comrade_diag_ = *msg;

    if (comrade_diag_.flying_normally) {
      if (!got_comrade_tracker_diag_) {
        got_comrade_tracker_diag_ = true;
        ROS_WARN_THROTTLE(0.5, "[]: Got first message from comrad, he is flying normally");
        ROS_WARN("[Comrade]: Paehali !");
      }
      time_last_comrade_tracker_diagnostics_ = ros::Time::now();
    } else {
      if (got_comrade_tracker_diag_) {
        ROS_WARN_THROTTLE(1.0, "[]: Comrade is not flying, set arena to 2");
        setArena(2);
        revenge_mode = true;
      }
    }
  }
}

//}

/* callbackCameraInfo //{ */

void BalloonCircleDestroy::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }

  if (!got_realsense_) {
    ROS_INFO_THROTTLE(0.5, "[RealSense]: Got first realsense camera info msg");
    got_realsense_ = true;
  }

  time_last_realsense_ = ros::Time::now();
}

//}

/* callbackConstraintsDiag //{ */

void BalloonCircleDestroy::callbackConstraintsDiag(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }
  if (!got_constraints_diag_) {
    got_constraints_diag_ = true;
    ROS_INFO("[%s]: got first constraints diagnostics msg", ros::this_node::getName().c_str());
  }


  {
    std::scoped_lock lock(mutex_constraints_);
    constraints_msg_ = *msg;
    cur_constraints_ = constraints_msg_.current_name;
    /* acceleration_ = constraints_msg_.horizontal_acceleration; */
    /* jerk_ = constraints_msg_.horizontal_jerk; */
  }

  time_last_tracker_diagnostics_ = ros::Time::now();
}


//}

/* callbackTimerIdling //{ */

void BalloonCircleDestroy::callbackTimerIdling([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }

  ROS_INFO("[%s]: Idling stopped", ros::this_node::getName().c_str());
  is_idling_ = false;
  if (_state_ == DESTROY_OVERSHOOT) {
    changeState(IDLE);
  }
}

//}

/* callbackTimeStateMachine //{ */

void BalloonCircleDestroy::callbackTimerStateMachine([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }

  if (!_is_state_machine_active_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_is_balloon_cloud_incoming_);

    /*  verbose output //{ */

    ROS_INFO_THROTTLE(0.5, "| ---------------- STATE MACHINE LOOP STATUS --------------- |");
    ROS_INFO_THROTTLE(0.5, "[State]: %s ", getStateName().c_str());
    ROS_INFO_THROTTLE(0.5, "[StateDuration]: %f ", cur_state_dur_);
    ROS_INFO_THROTTLE(0.5, "[CurrentConstraints]: %s ", cur_constraints_.c_str());
    std::string tracker_status_ = is_tracking_ ? "active" : "not active";
    ROS_INFO_THROTTLE(0.5, "[IsTracking]: %s", tracker_status_.c_str());
    std::string planner_status_ = _planner_active_ ? "active" : "not active";
    ROS_INFO_THROTTLE(0.5, "[Planner Status]: %s", planner_status_.c_str());
    std::string balloon_status_    = is_ballon_cloud_incoming_ ? "incoming" : "not incoming";
    std::string balloon_kf_status_ = is_ballon_incoming_ ? "incoming" : "not incoming";
    ROS_INFO_THROTTLE(0.5, "[Balloon cloud incoming]: %s", balloon_status_.c_str());
    ROS_INFO_THROTTLE(0.5, "[Balloon KF incoming]: %s", balloon_kf_status_.c_str());
    ROS_INFO_THROTTLE(0.5, "[Current Dist To ball]: %f ", (odom_vector_ - balloon_vector_).norm());
    ROS_INFO_THROTTLE(0.5, "[Closest ball (PointCloud)]: x %f y %f z %f", balloon_closest_vector_(0, 0), balloon_closest_vector_(1, 0),
                      balloon_closest_vector_(2, 0));
    ROS_INFO_THROTTLE(0.5, "[Closest ball (KF)]: x %f  y %f z %f", balloon_vector_(0, 0), balloon_vector_(1, 0), balloon_vector_(2, 0));
    ROS_INFO_THROTTLE(0.5, "[Dist between KF and PCL vectors]: %f ", (balloon_vector_ - balloon_closest_vector_).norm());

    ROS_INFO_THROTTLE(0.5, "[KF reset tries]  %d ", _reset_count_);
    ROS_INFO_THROTTLE(0.5, "[Popped count]  %d ", (int)_forb_vect_.size());
    ROS_INFO_THROTTLE(0.5, "[CUR HEIGHT]: %f ", odom_vector_(2, 0));
    ROS_INFO_THROTTLE(0.5, "[Cloud size]: %d ", int(balloon_pcl_processed_.size()));

    ROS_INFO_THROTTLE(0.5, "| ----------------- STATE MACHINE LOOP END ----------------- |");


    //}
    switch (_state_) {
      case State::IDLE:
        /*  IDLE state //{ */

        {
          destroy_set = false;
          if (_mpc_stop_ == false) {
            droneStop();
            return;
          }
          if (is_ballon_cloud_incoming_) {
            changeState(CHECKING_BALLOON);
          } else {
            changeState(GOING_AROUND);
            _mpc_stop_ = false;
          }
          plannerStop();
          balloon_vector_ = eigen_vect();
          _reset_count_   = 0;
          return;
        }


        //}
        break;
      case State::GOING_AROUND:
        /* GOING_AROUND state //{ */
        {
          if (cur_constraints_ != _sweep_constraints_) {
            setConstraints(_sweep_constraints_);
          }
          if (is_ballon_cloud_incoming_) {
            if (_mpc_stop_ == false) {
              droneStop();
              return;
            }
            is_idling_ = true;

            changeState(CHECKING_BALLOON);

            ros::NodeHandle nh("~");
            timer_idling_ = nh.createTimer(ros::Duration(_idle_time_), &BalloonCircleDestroy::callbackTimerIdling, this,
                                           true);  // the last boolean argument makes the timer run only once
          } else {

            if (!is_tracking_ && !is_idling_) {
              if (odom_vector_(2, 0) < _height_ - _height_tol_) {
                ROS_INFO("[]: Height is %f  compared to %f", odom_vector_(2, 0), _height_ - _height_tol_);
                ROS_WARN_THROTTLE(0.5, "[StateMachine]: height is not correct, ascend");
                goToHeight(_height_, _vel_);
              } else {
                scanArena();
              }
            }
          }
          return;
        }

        //}
        break;
      case State::CHECKING_BALLOON:
        /* CHECKING_BALLOON state //{ */

        {
          /* if (_mpc_stop_ == false) { */
          /*   droneStop(); */
          /*   return; */
          /* } */

          if (!is_idling_) {
            if (odom_vector_(2, 0) < _height_ - _height_tol_) {
              ROS_INFO("[]: Height is %f  compared to %f", odom_vector_(2, 0), _height_ - _height_tol_);
              ROS_WARN_THROTTLE(0.5, "[StateMachine]: height is not correct, ascend");
              goToHeight(_height_, _vel_);
              break;
            }
          } else {
            break;
          }
          if (!is_ballon_cloud_incoming_) {
            ROS_INFO("[]: PCL is not incoming");
            changeState(IDLE);
          }
          balloon_closest_vector_ = getClosestBalloon();
          if (!isBalloonVisible(balloon_closest_vector_)) {
            ROS_WARN_THROTTLE(1.0, "[]: balloon is not visible, search again");
            changeState(IDLE);
            return;
          } else {
            changeState(GOING_TO_BALLOON);
            plannerActivate(balloon_closest_vector_, _dist_error_);
          }
        }


        //}
        break;
      case State::GOING_TO_BALLOON:
        /* GOING_TO_BALLOON state //{ */

        {
          destroy_set = false;
          if (is_idling_) {
            return;
          }
          if (!is_ballon_cloud_incoming_) {
            changeState(IDLE);
          }
          if (!_planner_active_ || !is_ballon_incoming_) {
            plannerActivate(balloon_closest_vector_, _dist_error_);

          } else if ((odom_vector_ - balloon_closest_vector_).norm() < _dist_kf_activation_ && isBalloonVisible(balloon_vector_) &&
                     pointInSafety(balloon_vector_)) {

            ROS_WARN_THROTTLE(0.5, "[StateMachine]: KF close ");
            changeState(DESTROYING);
            return;
          }

          if ((odom_vector_ - balloon_closest_vector_).norm() > _dist_to_balloon_ - _dist_acc_ - _dead_band_factor_) {

            if (isBalloonVisible(balloon_closest_vector_)) {
              if (!balloon_closest_vector_.isZero()) {
                getCloseToBalloon(balloon_closest_vector_, _dist_to_balloon_, _vel_);
              }
            } else {
              changeState(IDLE);
              ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: baloon is not visible, stop");
              is_idling_ = true;
              ros::NodeHandle nh("~");
              timer_idling_ = nh.createTimer(ros::Duration(_idle_time_), &BalloonCircleDestroy::callbackTimerIdling, this,
                                             true);  // the last boolean argument makes the timer run only once
            }
          } else if (isBalloonVisible(balloon_vector_) && pointInSafety(balloon_vector_)) {
            ROS_WARN_THROTTLE(0.5, "[StateMachine]: dist nice");
            changeState(DESTROYING);

          } else if (!isBalloonVisible(balloon_vector_)) {
            changeState(CHECKING_BALLOON);
          }
        }

        //}
        break;
        /*   /1* AT_BALLOON state //{ *1/ */
        /* case State::AT_BALLOON: */

        /*   { */
        /*     if (isBalloonVisible(balloon_vector_) && isPointInArena(balloon_vector_)) { */
        /*       /1* _state_ = DESTROYING; *1/ */
        /*     } */
        /*     /1* double dist_diff = (balloon_closest_vector_ - balloon_vector_).norm(); *1/ */
        /*   } */


        /* break; */
        /* //} */
      case State::DESTROYING:
        /* DESTROYING state //{ */

        {
          /* if (odom_vector_(2, 0) - _height_tol_ > balloon_vector_(2, 0)) { */
          /*   ROS_WARN_THROTTLE(0.5, "[StateMachine]: height is not the same with the balloon, reset"); */
          /*   changeState(GOING_TO_BALLOON); */
          /*   break; */
          /* } */

          if (balloonOutdated()) {
            if (ros::Time::now().toSec() - time_last_planner_reset_.toSec() < _wait_for_ball_) {
              return;
            }
            if (_reset_tries_ > _reset_count_ && ros::Time::now().toSec() - time_last_planner_reset_.toSec() > _wait_for_ball_) {
              plannerActivate(balloon_closest_vector_, _dist_error_);
              ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: reseting kf");
              _reset_count_++;
            } else {
              ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: balloon kf is outdated");
              changeState(IDLE);
              _reset_count_ = 0;
            }
          }
          if (isBalloonVisible(balloon_vector_)) {
            if (!balloon_closest_vector_.isZero()) {
              getCloseToBalloon(balloon_vector_, -_dist_to_overshoot_, _vel_attack_);
              time_traj_sent_ = ros::Time::now();
              if (!destroy_set) {
                destroy_set = true;
              }
            }
            return;
          } else {
            if (ros::Time::now().toSec() - time_last_balloon_point_.toSec() > _wait_for_ball_) {
              ROS_WARN_THROTTLE(1.0, "[StateMachine]: balloon is not visible within time, reset ");
              changeState(IDLE);
              if (_mpc_stop_ == false) {
                droneStop();
                return;
              }
            }
            /* else { */
            /*   _state_                      = DESTROY_OVERSHOOT; */
            /*   _time_destroy_overshoot_set_ = ros::Time::now(); */
            /*   ROS_WARN_THROTTLE(0.5, "[StateMachine]: STATE RESET TO %s", getStateName().c_str()); */
            /*   return; */
            /* } */
          }
        }


        //}
        break;
      case State::DESTROY_OVERSHOOT:
        /* DESTROY_OVERSHOOT state //{ */

        {

          if (isBalloonVisible(balloon_vector_)) {
            changeState(DESTROYING);
          }
          if (!is_tracking_ && !is_idling_ && _last_goal_reached_) {
            ROS_WARN_THROTTLE(0.5, "[StateMachine]:destroying ended");
            changeState(IDLE);
          }
          if (ros::Time::now().toSec() - _time_destroy_overshoot_set_.toSec() > _state_reset_time_) {
            ROS_WARN_THROTTLE(1.0, "[StateMachine]: DESTROY OVERSHOOT TIMER, SETTING IDLE ");
            changeState(IDLE);
          }
        }

        //}
        break;
      default:
        /* default case //{ */
        ROS_INFO_THROTTLE(0.5, "[StateMachine]: Received unexpected state");
        break;
        //}
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
    if ((time_now - time_last_balloon_cloud_point_).toSec() > _wait_for_ball_) {
      ROS_WARN_THROTTLE(0.5, "[%s]: haven't received any balloon cloud points for %f", ros::this_node::getName().c_str(),
                        (time_now - time_last_balloon_cloud_point_).toSec());
      is_ballon_cloud_incoming_ = false;
    } else {
      if (balloon_pcl_processed_.size() > 0) {
        is_ballon_cloud_incoming_ = true;
      }
    }
  }

  if (!got_constraints_diag_) {
    ROS_WARN_THROTTLE(0.5, "[%s]: haven't received constraints manager diagnostics since launch", ros::this_node::getName().c_str());
  } else {
    if ((time_now - time_last_constraints_diagnostics_).toSec() > 4.0) {
      ROS_WARN_THROTTLE(0.5, "[%s]: haven't received any constraints manager diagnostics for %f", ros::this_node::getName().c_str(),
                        (time_now - time_last_constraints_diagnostics_).toSec());
    }
  }


  if (_is_state_machine_active_) {
    if (!got_realsense_) {
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: Didn't get realsense since start of the state machine");
      abortEland();
    } else {
      if ((time_now - time_last_realsense_).toSec() > 5.0) {
        ROS_WARN_THROTTLE(0.5, "[%s]: haven't received any realsense data %f, ABORT, ALARM KURWA", ros::this_node::getName().c_str(),
                          (time_now - time_last_realsense_).toSec());
        abortEland();
      }
    }

    if(got_comrade_tracker_diag_) {
      if(ros::Time::now().toSec() - time_last_comrade_tracker_diagnostics_.toSec() > _max_silence_time_) {
        ROS_WARN("[ComradeWatcher] Haven't seen comrade for more then %f, setting revenge mode ON!!!",_max_silence_time_ );
        revenge_mode = true;
        setArena(2);
      
      }
    }
  }
  
}

//}

/* callbackTimerCheckEmulation //{ */

void BalloonCircleDestroy::callbackTimerCheckEmulation([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }
  if (_state_ == DESTROYING) {
    ROS_INFO("[EmulationTimer]: This balloon was shot, adding to forbidden list");
    addForbidden(balloon_vector_, _forbidden_radius_);
    changeState(IDLE);
    timer_set_ = false;
  } else {
    ROS_INFO("[EmulationTimer]: Not in destroy state, wasn't popped");
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
      /* if (ros::Time::now().toSec() - time_last_balloon_cloud_point_.toSec() > _time_to_land_) { */
      /*   ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: State machine finished, landing"); */
      /*   /1* landAndEnd(); *1/ */
      /* } */
    }
  }
  /* ROS_INFO_THROTTLE(1.0, "[%s]: Got balloon point at x %f ", ros::this_node::getName().c_str(), balloon_point_.pose.pose.position.x); */
}

//}

/* callbackTimerCheckForbidden //{ */

void BalloonCircleDestroy::callbackTimerCheckForbidden([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }

  if (!_is_state_machine_active_) {
    return;
  }
  std::vector<Forbidden_t>::iterator it = _forb_vect_.begin();
  for (; it != _forb_vect_.end();) {
    if (ros::Time::now().toSec() - it->start_time.toSec() > it->lifetime) {
      ROS_INFO("[StateMachine]: cleared forbidden area due to timer");
      it = _forb_vect_.erase(it);
    } else {
      ++it;
    }
  }
}

//}

/* callbackTimerCheckStateMachine //{ */

void BalloonCircleDestroy::callbackTimerCheckStateMachine([[maybe_unused]] const ros::TimerEvent& te) {
  if (!is_initialized_) {
    return;
  }

  if (!_is_state_machine_active_) {
    return;
  }

  cur_state_dur_ = ros::Time::now().toSec() - time_state_set_.toSec();
  switch (_state_) {

    case CHECKING_BALLOON:
      if (cur_state_dur_ > time_to_check_balloon) {
        ROS_INFO("[StateMachine]: too long check balloon");
        addForbidden(balloon_closest_vector_, _forbidden_radius_);
        changeState(IDLE);
      }
      break;
    case GOING_TO_BALLOON:
      if (cur_state_dur_ > time_to_going_to) {
        ROS_INFO("[StateMachine]: too long going to balloon");
        addForbidden(balloon_closest_vector_, _forbidden_radius_);
        changeState(IDLE);
      }
      break;
    case DESTROYING:
      if (cur_state_dur_ > time_to_destroy) {
        ROS_INFO("[StateMachine]: too long destroy");
        addForbidden(balloon_closest_vector_, _forbidden_radius_);
        changeState(IDLE);
      }
  }
  // if swapping of the arenas are enabled and the time is bigger from last swap, we swap the arenas 1->0, and 0->1;

  if (ros::Time::now().toSec() - current_arena_time_.toSec() > _arena_time_ && _do_swap_) {
    if (_arena_type_ == 0) {
      setArena(1);
    } else if (_arena_type_ == 1) {
      setArena(0);
    }
    changeState(IDLE);
  }
}

//}

/* callbackTimerPublishStatus //{ */


void BalloonCircleDestroy::callbackTimerPublishStatus([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  std_msgs::String status_;

  if (!_is_state_machine_active_) {

    status_.data = "Not activated";
    status_pub_.publish(status_);
    return;
  }

  status_.data = getStateName().c_str();
  status_pub_.publish(status_);
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
      marker.header.frame_id    = world_frame_id_;
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
    visualization_msgs::Marker line_pcl;
    line_pcl.header.frame_id    = world_frame_id_;
    line_pcl.header.stamp       = ros::Time::now();
    line_pcl.ns                 = "mtsp";
    line_pcl.id                 = id++;
    line_pcl.type               = visualization_msgs::Marker::LINE_STRIP;
    line_pcl.action             = visualization_msgs::Marker::ADD;
    line_pcl.pose.position.x    = 0;
    line_pcl.pose.position.y    = 0;
    line_pcl.pose.position.z    = 0;
    line_pcl.pose.orientation.x = 0.0;
    line_pcl.pose.orientation.y = 0.0;
    line_pcl.pose.orientation.z = 0.0;
    line_pcl.pose.orientation.w = 1.0;
    line_pcl.scale.x            = 0.15;
    line_pcl.scale.y            = 0.15;
    line_pcl.scale.z            = 0.15;
    line_pcl.color.a            = 0.3;
    line_pcl.color.r            = 0.0;
    line_pcl.color.g            = 1.0;
    line_pcl.color.b            = 0.0;

    geometry_msgs::Point odom_point_;
    odom_point_.x = odom_vector_(0, 0);
    odom_point_.y = odom_vector_(1, 0);
    odom_point_.z = odom_vector_(2, 0);


    geometry_msgs::Point balloon_viz_point_;
    balloon_viz_point_.x = balloon_closest_vector_(0, 0);
    balloon_viz_point_.y = balloon_closest_vector_(1, 0);
    balloon_viz_point_.z = balloon_closest_vector_(2, 0);

    if (_state_ != GOING_AROUND) {

      line_pcl.points.push_back(odom_point_);
      line_pcl.points.push_back(balloon_viz_point_);
    }

    msg_out.markers.push_back(line_pcl);

    if (_planner_active_) {
      visualization_msgs::Marker marker;
      marker.header.frame_id    = world_frame_id_;
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
      marker.scale.x            = _dist_error_;
      marker.scale.y            = _dist_error_;
      marker.scale.z            = _dist_error_;
      marker.color.a            = 0.3;
      marker.color.r            = 0.0;
      marker.color.g            = 1.0;
      marker.color.b            = 0.0;

      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

      msg_out.markers.push_back(marker);


      visualization_msgs::Marker line_kf;
      line_kf.header.frame_id    = world_frame_id_;
      line_kf.header.stamp       = ros::Time::now();
      line_kf.ns                 = "mtsp";
      line_kf.id                 = id++;
      line_kf.type               = visualization_msgs::Marker::LINE_STRIP;
      line_kf.action             = visualization_msgs::Marker::ADD;
      line_kf.pose.position.x    = 0;
      line_kf.pose.position.y    = 0;
      line_kf.pose.position.z    = 0;
      line_kf.pose.orientation.x = 0.0;
      line_kf.pose.orientation.y = 0.0;
      line_kf.pose.orientation.z = 0.0;
      line_kf.pose.orientation.w = 1.0;
      line_kf.scale.x            = 0.15;
      line_kf.scale.y            = 0.15;
      line_kf.scale.z            = 0.15;
      line_kf.color.a            = 0.3;
      line_kf.color.r            = 1.0;
      line_kf.color.g            = 0.0;
      line_kf.color.b            = 0.0;


      geometry_msgs::Point balloon_kf_;
      balloon_kf_.x = balloon_vector_(0, 0);
      balloon_kf_.y = balloon_vector_(1, 0);
      balloon_kf_.z = balloon_vector_(2, 0);

      line_kf.points.push_back(odom_point_);
      line_kf.points.push_back(balloon_kf_);
      msg_out.markers.push_back(line_kf);
    }

    visualization_msgs::Marker arena_bound_;
    arena_bound_.header.frame_id    = world_frame_id_;
    arena_bound_.header.stamp       = ros::Time::now();
    arena_bound_.ns                 = "mtsp";
    arena_bound_.id                 = id++;
    arena_bound_.type               = visualization_msgs::Marker::LINE_STRIP;
    arena_bound_.action             = visualization_msgs::Marker::ADD;
    arena_bound_.pose.position.x    = 0;
    arena_bound_.pose.position.y    = 0;
    arena_bound_.pose.position.z    = 0;
    arena_bound_.pose.orientation.x = 0.0;
    arena_bound_.pose.orientation.y = 0.0;
    arena_bound_.pose.orientation.z = 0.0;
    arena_bound_.pose.orientation.w = 1.0;
    arena_bound_.scale.x            = 0.35;
    arena_bound_.scale.y            = 0.35;
    arena_bound_.scale.z            = 0.35;
    arena_bound_.color.a            = 0.3;
    arena_bound_.color.r            = 0.0;
    arena_bound_.color.g            = 1.0;
    arena_bound_.color.b            = 0.0;


    geometry_msgs::Point mark_point_;
    // x min y min z min
    mark_point_.x = _x_min_;
    mark_point_.y = _y_min_;
    mark_point_.z = _z_min_;
    arena_bound_.points.push_back(mark_point_);
    mark_point_.x = _x_min_;
    mark_point_.y = _y_max_;
    mark_point_.z = _z_min_;
    arena_bound_.points.push_back(mark_point_);

    mark_point_.x = _x_max_;
    mark_point_.y = _y_max_;
    mark_point_.z = _z_min_;
    arena_bound_.points.push_back(mark_point_);

    mark_point_.x = _x_max_;
    mark_point_.y = _y_min_;
    mark_point_.z = _z_min_;
    arena_bound_.points.push_back(mark_point_);

    mark_point_.x = _x_min_;
    mark_point_.y = _y_min_;
    mark_point_.z = _z_min_;
    arena_bound_.points.push_back(mark_point_);


    msg_out.markers.push_back(arena_bound_);

    visualization_msgs::Marker arena_bound_top_;
    arena_bound_top_.header.frame_id    = world_frame_id_;
    arena_bound_top_.header.stamp       = ros::Time::now();
    arena_bound_top_.ns                 = "mtsp";
    arena_bound_top_.id                 = id++;
    arena_bound_top_.type               = visualization_msgs::Marker::LINE_STRIP;
    arena_bound_top_.action             = visualization_msgs::Marker::ADD;
    arena_bound_top_.pose.position.x    = 0;
    arena_bound_top_.pose.position.y    = 0;
    arena_bound_top_.pose.position.z    = 0;
    arena_bound_top_.pose.orientation.x = 0.0;
    arena_bound_top_.pose.orientation.y = 0.0;
    arena_bound_top_.pose.orientation.z = 0.0;
    arena_bound_top_.pose.orientation.w = 1.0;
    arena_bound_top_.scale.x            = 0.35;
    arena_bound_top_.scale.y            = 0.35;
    arena_bound_top_.scale.z            = 0.35;
    arena_bound_top_.color.a            = 0.3;
    arena_bound_top_.color.r            = 0.0;
    arena_bound_top_.color.g            = 1.0;
    arena_bound_top_.color.b            = 0.0;


    geometry_msgs::Point mark_point_top_;
    // x min y min z min
    mark_point_top_.x = _x_min_;
    mark_point_top_.y = _y_min_;
    mark_point_top_.z = _z_max_;
    arena_bound_top_.points.push_back(mark_point_top_);
    mark_point_top_.x = _x_min_;
    mark_point_top_.y = _y_max_;
    mark_point_top_.z = _z_max_;
    arena_bound_top_.points.push_back(mark_point_top_);

    mark_point_top_.x = _x_max_;
    mark_point_top_.y = _y_max_;
    mark_point_top_.z = _z_max_;
    arena_bound_top_.points.push_back(mark_point_top_);

    mark_point_top_.x = _x_max_;
    mark_point_top_.y = _y_min_;
    mark_point_top_.z = _z_max_;
    arena_bound_top_.points.push_back(mark_point_top_);

    mark_point_top_.x = _x_min_;
    mark_point_top_.y = _y_min_;
    mark_point_top_.z = _z_max_;
    arena_bound_top_.points.push_back(mark_point_top_);

    msg_out.markers.push_back(arena_bound_top_);

    visualization_msgs::Marker arena_pole_1;
    arena_pole_1.header.frame_id    = world_frame_id_;
    arena_pole_1.header.stamp       = ros::Time::now();
    arena_pole_1.ns                 = "mtsp";
    arena_pole_1.id                 = id++;
    arena_pole_1.type               = visualization_msgs::Marker::CYLINDER;
    arena_pole_1.action             = visualization_msgs::Marker::ADD;
    arena_pole_1.pose.position.x    = _x_min_;
    arena_pole_1.pose.position.y    = _y_min_;
    arena_pole_1.pose.position.z    = _z_min_ + 0.43 * _z_max_;
    arena_pole_1.pose.orientation.x = 0.0;
    arena_pole_1.pose.orientation.y = 0.0;
    arena_pole_1.pose.orientation.z = 0.0;
    arena_pole_1.pose.orientation.w = 1.0;
    arena_pole_1.scale.x            = 0.5;
    arena_pole_1.scale.y            = 0.5;
    arena_pole_1.scale.z            = _z_max_ - 1;
    arena_pole_1.color.a            = 0.3;
    arena_pole_1.color.r            = 1.0;
    arena_pole_1.color.g            = 0.0;
    arena_pole_1.color.b            = 0.0;

    msg_out.markers.push_back(arena_pole_1);
    visualization_msgs::Marker arena_pole_2;
    arena_pole_2.header.frame_id    = world_frame_id_;
    arena_pole_2.header.stamp       = ros::Time::now();
    arena_pole_2.ns                 = "mtsp";
    arena_pole_2.id                 = id++;
    arena_pole_2.type               = visualization_msgs::Marker::CYLINDER;
    arena_pole_2.action             = visualization_msgs::Marker::ADD;
    arena_pole_2.pose.position.x    = _x_min_;
    arena_pole_2.pose.position.y    = _y_max_;
    arena_pole_2.pose.position.z    = _z_min_ + 0.43 * _z_max_;
    arena_pole_2.pose.orientation.x = 0.0;
    arena_pole_2.pose.orientation.y = 0.0;
    arena_pole_2.pose.orientation.z = 0.0;
    arena_pole_2.pose.orientation.w = 1.0;
    arena_pole_2.scale.x            = 0.5;
    arena_pole_2.scale.y            = 0.5;
    arena_pole_2.scale.z            = _z_max_ - 1;
    arena_pole_2.color.a            = 0.3;
    arena_pole_2.color.r            = 1.0;
    arena_pole_2.color.g            = 0.0;
    arena_pole_2.color.b            = 0.0;

    msg_out.markers.push_back(arena_pole_2);
    visualization_msgs::Marker arena_pole_3;
    arena_pole_3.header.frame_id    = world_frame_id_;
    arena_pole_3.header.stamp       = ros::Time::now();
    arena_pole_3.ns                 = "mtsp";
    arena_pole_3.id                 = id++;
    arena_pole_3.type               = visualization_msgs::Marker::CYLINDER;
    arena_pole_3.action             = visualization_msgs::Marker::ADD;
    arena_pole_3.pose.position.x    = _x_max_;
    arena_pole_3.pose.position.y    = _y_max_;
    arena_pole_3.pose.position.z    = _z_min_ + 0.43 * _z_max_;
    arena_pole_3.pose.orientation.x = 0.0;
    arena_pole_3.pose.orientation.y = 0.0;
    arena_pole_3.pose.orientation.z = 0.0;
    arena_pole_3.pose.orientation.w = 1.0;
    arena_pole_3.scale.x            = 0.5;
    arena_pole_3.scale.y            = 0.5;
    arena_pole_3.scale.z            = _z_max_ - 1;
    arena_pole_3.color.a            = 0.3;
    arena_pole_3.color.r            = 0.0;
    arena_pole_3.color.g            = 1.0;
    arena_pole_3.color.b            = 0.0;


    msg_out.markers.push_back(arena_pole_3);
    visualization_msgs::Marker arena_pole_4;
    arena_pole_4.header.frame_id    = world_frame_id_;
    arena_pole_4.header.stamp       = ros::Time::now();
    arena_pole_4.ns                 = "mtsp";
    arena_pole_4.id                 = id++;
    arena_pole_4.type               = visualization_msgs::Marker::CYLINDER;
    arena_pole_4.action             = visualization_msgs::Marker::ADD;
    arena_pole_4.pose.position.x    = _x_max_;
    arena_pole_4.pose.position.y    = _y_min_;
    arena_pole_4.pose.position.z    = _z_min_ + 0.43 * _z_max_;
    arena_pole_4.pose.orientation.x = 0.0;
    arena_pole_4.pose.orientation.y = 0.0;
    arena_pole_4.pose.orientation.z = 0.0;
    arena_pole_4.pose.orientation.w = 1.0;
    arena_pole_4.scale.x            = 0.5;
    arena_pole_4.scale.y            = 0.5;
    arena_pole_4.scale.z            = _z_max_ - 1;
    arena_pole_4.color.a            = 0.3;
    arena_pole_4.color.r            = 0.0;
    arena_pole_4.color.g            = 1.0;
    arena_pole_4.color.b            = 0.0;


    msg_out.markers.push_back(arena_pole_4);
    // | ---------------------- arena markers --------------------- |
    if (_is_state_machine_active_) {
      visualization_msgs::Marker marker;
      marker.header.frame_id    = world_frame_id_;
      marker.header.stamp       = ros::Time();
      marker.ns                 = "mtsp";
      marker.id                 = id++;
      marker.type               = visualization_msgs::Marker::SPHERE;
      marker.action             = visualization_msgs::Marker::ADD;
      marker.pose.position.x    = middle_one_(0, 0);
      marker.pose.position.y    = middle_one_(1, 0);
      marker.pose.position.z    = 1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x            = 1;
      marker.scale.y            = 1;
      marker.scale.z            = 1;
      marker.color.a            = 1.0;  // Don't forget to set the alpha!
      marker.color.r            = 0.0;
      marker.color.g            = 1.0;
      marker.color.b            = 0.0;
      // only if using a MESH_RESOURCE marker type:
      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

      msg_out.markers.push_back(marker);

      visualization_msgs::Marker marker2;
      marker2.header.frame_id    = world_frame_id_;
      marker2.header.stamp       = ros::Time();
      marker2.ns                 = "mtsp";
      marker2.id                 = id++;
      marker2.type               = visualization_msgs::Marker::SPHERE;
      marker2.action             = visualization_msgs::Marker::ADD;
      marker2.pose.position.x    = middle_two_(0, 0);
      marker2.pose.position.y    = middle_two_(1, 0);
      marker2.pose.position.z    = 1;
      marker2.pose.orientation.x = 0.0;
      marker2.pose.orientation.y = 0.0;
      marker2.pose.orientation.z = 0.0;
      marker2.pose.orientation.w = 1.0;
      marker2.scale.x            = 1;
      marker2.scale.y            = 1;
      marker2.scale.z            = 1;
      marker2.color.a            = 1.0;  // Don't forget to set the alpha!
      marker2.color.r            = 0.0;
      marker2.color.g            = 1.0;
      marker2.color.b            = 0.0;
      // only if using a MESH_RESOURCE marker2 type:
      marker2.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";


      msg_out.markers.push_back(marker2);

      // | ---------- current safety area for object detect --------- |

      // safety area marker
      visualization_msgs::Marker safety_marker;

      safety_marker.header.frame_id = world_frame_id_;
      safety_marker.type            = visualization_msgs::Marker::LINE_LIST;
      safety_marker.ns              = "safety_area";
      safety_marker.id              = id++;
      safety_marker.color.a         = 0.5;
      safety_marker.scale.x         = 0.2;
      safety_marker.color.r         = 1;
      safety_marker.color.g         = 1;
      safety_marker.color.b         = 0;

      std::vector<geometry_msgs::Point> safety_area_points;
      /* ROS_INFO_STREAM("hello bithc " << cur_safety_); */
      safety_area_points = cur_safety_->getPointMessageVector(0);
      for (size_t i = 0; i < safety_area_points.size(); i++) {
        safety_marker.points.push_back(safety_area_points[i]);
        safety_marker.points.push_back(safety_area_points[(i + 1) % safety_area_points.size()]);
      }

      msg_out.markers.push_back(safety_marker);
    }

    rviz_pub_.publish(msg_out);
  }
}


//}

/* callbackArenaInfo //{ */

void BalloonCircleDestroy::callbackArenaInfo(const mbzirc_msgs::MbzircArenaParametersConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_arena_);

    arena_params_ = *msg;
    gotArena      = true;

    std::string arena_frame = arena_params_.header.frame_id;
    ROS_ERROR("[MbzircArena]:Arena  frame id %s", arena_frame.c_str());
  }
}

//}

/* configurateArena //{ */

bool BalloonCircleDestroy::configurateArena() {
  ROS_WARN("[]: Called configuring the arena");

  {
    std::scoped_lock lock(mutex_arena_);

    std::vector<geometry_msgs::PointStamped> corners_;
    for (auto p_ : arena_params_.arena_corners) {
      geometry_msgs::PointStamped corner_a;
      corner_a.header  = arena_params_.header;
      corner_a.point.x = p_.x;
      corner_a.point.y = p_.y;
      corner_a.point.z = p_.z;
      auto res         = transformer_.transformSingle(world_frame_id_, corner_a);

      if (res) {
        corners_.push_back(res.value());

      } else {
        ROS_WARN("[ArenaTransformer]: could not transform from %s to  to the %s", corner_a.header.frame_id.c_str(), world_frame_id_.c_str());
        return false;
      }
    }

    Eigen::MatrixXd safety_matrix_(8, 2);


    for (uint i = 0; i < arena_params_.safety_area.size(); i++) {
      geometry_msgs::PointStamped safety_p_;
      safety_p_.header  = arena_params_.header;
      safety_p_.point.x = arena_params_.safety_area[i].x;
      safety_p_.point.y = arena_params_.safety_area[i].y;
      safety_p_.point.z = arena_params_.safety_area[i].z;
      auto res          = transformer_.transformSingle(world_frame_id_, safety_p_);

      if (res) {
        safety_matrix_(i, 0) = res.value().point.x;
        safety_matrix_(i, 1) = res.value().point.y;

      } else {

        ROS_WARN("[ArenaTransformer]: could not transform from %s to  to the %s", safety_p_.header.frame_id.c_str(), world_frame_id_.c_str());
        return false;
      }
    }

    middle_one_(0, 0) = (corners_[0].point.x + corners_[1].point.x) / 2;
    middle_one_(1, 0) = (corners_[0].point.y + corners_[1].point.y) / 2;
    middle_two_(0, 0) = (corners_[2].point.x + corners_[3].point.x) / 2;
    middle_two_(1, 0) = (corners_[2].point.y + corners_[3].point.y) / 2;

    safety_1       = Eigen::MatrixXd(6, 2);
    safety_1(0, 0) = middle_two_(0, 0);
    safety_1(0, 1) = middle_two_(1, 0);

    safety_1(1, 0) = safety_matrix_(5, 0);
    safety_1(1, 1) = safety_matrix_(5, 1);

    safety_1(2, 0) = safety_matrix_(6, 0);
    safety_1(2, 1) = safety_matrix_(6, 1);

    safety_1(3, 0) = safety_matrix_(7, 0);
    safety_1(3, 1) = safety_matrix_(7, 1);

    safety_1(4, 0) = safety_matrix_(0, 0);
    safety_1(4, 1) = safety_matrix_(0, 1);

    safety_1(5, 0) = middle_one_(0, 0);
    safety_1(5, 1) = middle_one_(1, 0);

    // choosing the shortest parts of the polygon
    if ((safety_1.row(0) - safety_1.row(1)).norm() > (safety_1.row(4) - safety_1.row(5)).norm()) {
      safe_length_1 = (safety_1.row(4) - safety_1.row(5)).norm();
    } else {
      safe_length_1 = (safety_1.row(0) - safety_1.row(1)).norm();
    }
    ROS_INFO_STREAM("safety matrix 1 " << safety_1);

    ROS_ERROR("[]: height first %f second %f", (safety_1.row(7) - safety_1.row(6)).norm(), (safety_1.row(2) - safety_1.row(3)).norm());
    if ((safety_1.row(5) - safety_1.row(0)).norm() > (safety_1.row(2) - safety_1.row(3)).norm()) {
      safe_height_1 = (safety_1.row(2) - safety_1.row(3)).norm();
    } else {
      safe_height_1 = (safety_1.row(0) - safety_1.row(5)).norm();
    }

    ROS_ERROR("[]:  arena 1 length %f and height %f ", safe_length_1, safe_height_1);

    /* safety_polygon_1 = std::make_shared<mrs_lib::Polygon>(safety_1); */


    safety_2 = Eigen::MatrixXd(6, 2);

    safety_2(0, 0) = middle_one_(0, 0);
    safety_2(0, 1) = middle_one_(1, 0);

    safety_2(1, 0) = safety_matrix_(1, 0);
    safety_2(1, 1) = safety_matrix_(1, 1);

    safety_2(2, 0) = safety_matrix_(2, 0);
    safety_2(2, 1) = safety_matrix_(2, 1);

    safety_2(3, 0) = safety_matrix_(3, 0);
    safety_2(3, 1) = safety_matrix_(3, 1);

    safety_2(4, 0) = safety_matrix_(4, 0);
    safety_2(4, 1) = safety_matrix_(4, 1);


    safety_2(5, 0) = middle_two_(0, 0);
    safety_2(5, 1) = middle_two_(1, 0);


    // choosing the shortest parts of the polygon
    if ((safety_2.row(0) - safety_2.row(1)).norm() > (safety_2.row(4) - safety_2.row(5)).norm()) {
      safe_length_2 = (safety_2.row(4) - safety_2.row(5)).norm();
    } else {
      safe_length_2 = (safety_2.row(0) - safety_2.row(1)).norm();
    }

    if ((safety_2.row(5) - safety_2.row(0)).norm() > (safety_2.row(2) - safety_2.row(3)).norm()) {
      safe_height_2 = (safety_2.row(2) - safety_2.row(3)).norm();
    } else {
      safe_height_2 = (safety_2.row(5) - safety_2.row(0)).norm();
    }
    ROS_ERROR("[]:  arena 2 length %f and height %f ", safe_length_2, safe_height_2);

    ROS_INFO_STREAM("safety matrix 2 " << safety_2);
    /* safety_polygon_2 = std::make_shared<mrs_lib::Polygon>(safety_2); */

    /* safety_polygon_ = std::make_shared<mrs_lib::Polygon>(safety_matrix_); */

    // choosing the shortest parts of the polygon
    if ((safety_matrix_.row(0) - safety_matrix_.row(1)).norm() > (safety_matrix_.row(4) - safety_matrix_.row(5)).norm()) {
      safe_length_ = (safety_matrix_.row(4) - safety_matrix_.row(5)).norm();
    } else {
      safe_length_ = (safety_matrix_.row(0) - safety_matrix_.row(1)).norm();
    }

    if ((safety_matrix_.row(7) - safety_matrix_.row(6)).norm() > (safety_matrix_.row(2) - safety_matrix_.row(3)).norm()) {
      safe_height_ = (safety_matrix_.row(2) - safety_matrix_.row(3)).norm();
    } else {
      safe_height_ = (safety_matrix_.row(7) - safety_matrix_.row(6)).norm();
    }
    arenaConfigured = true;
    ROS_INFO("[]: configured");
    return true;
  }
}

//}

/* callbackStartStateMachine //{ */


bool BalloonCircleDestroy::callbackStartStateMachine([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    res.success = false;
    res.message = "Can't trigger service, not initialized";
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Couldn't call the service, not inited yet");

    return false;
  }
  if (_is_state_machine_active_) {
    res.message = "State machine was already active";
    res.success = true;
    return true;
  } else {
    _is_state_machine_active_ = true;
    res.message               = "State machine activated";
    _state_                   = IDLE;
    res.success               = true;
    return true;
  }
}

//}

/* callbackAutoStart //{ */

bool BalloonCircleDestroy::callbackAutoStart(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res) {

  switch (req.value) {

    case 0:
      if (!setArena(0)) {
        res.message = "[AutoStart]: Arena 0 couldn't be  set";
        res.success = false;
        return true;
      }
      res.message               = "[AutoStart]: Arena 0 is set";
      _is_state_machine_active_ = true;
      res.success               = true;
      return true;

    case 1:
      if (!setArena(1)) {
        res.message = "[AutoStart]: Arena 1 couldn't be  set";
        res.success = false;
        return true;
      }
      ROS_INFO("[AutoStart]: Arena 1 is set");
      res.message               = "[AutoStart]: Arena 1 is set";
      _is_state_machine_active_ = true;
      res.success               = true;
      return true;
    case 2:
      if (!setArena(2)) {
        res.message = "[AutoStart]: Arena 2 couldn't be  set";
        res.success = false;
        return true;
      }
      ROS_INFO("[AutoStart]: Arena 2 is set");
      res.message               = "[AutoStart]: Arena 2 is set";
      _is_state_machine_active_ = true;
      res.success               = true;
      return true;
    default:
      if (!setArena(2)) {
        res.message = "[AutoStart]: Arena 2 couldn't be  set";
        res.success = false;
        return true;
      }
      ROS_INFO("[AutoStart]: Default case is triggered ( RC signal is bad ) Arena 2 is set");
      res.message               = "[AutoStart]: Default case is triggered ( RC signal is bad ) Arena 2 is set";
      _is_state_machine_active_ = true;
      res.success               = true;
      return true;
  }
  ROS_WARN("[AutoStart]: Unexpected value from automatic start request");
  return true;
}

//}

/* callbackStopStateMachine //{ */


bool BalloonCircleDestroy::callbackStopStateMachine([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    res.success = false;
    res.message = "Can't trigger service, not initialized";
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Couldn't call the service, not inited yet");

    return false;
  }
  if (_is_state_machine_active_) {
    _is_state_machine_active_ = false;
    res.message               = "State machine disabled";

  } else {
    res.message = "State machine isn't working";
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
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Couldn't call the service, not inited yet");

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

/* callbackResetZones //{ */


bool BalloonCircleDestroy::callbackResetZones([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    res.success = false;
    res.message = "Can't trigger service, not initialized";
    ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Couldn't call the service, not inited yet");

    return false;
  }
  std_srvs::Trigger srv_;


  if (srv_planner_reset_zones_.call(srv_)) {
    if (srv_.response.success) {
      _forb_vect_.clear();
      res.success = true;
      res.message = "Zones are reset for planner and for the destroyer, free to continue";
      ROS_INFO_THROTTLE(0.5, "[ResetZonesSrv]: zones have benn reset, number of forb zones %d, free to proceed ", (int)_forb_vect_.size());
      return true;
    } else {
      ROS_INFO_THROTTLE(0.5, "[ResetZonesSrv]: zones couldn't be reset for planner, don't turn me into a dummy, I need the planner buddy ");
    }

  } else {
    ROS_ERROR_THROTTLE(0.5, "[ResetZonesSrv]: Failed at calling planner reset exculsive zones service  ");
  }
  res.success = false;
  return false;
}

//}

/* getCloseToBalloon //{ */

void BalloonCircleDestroy::getCloseToBalloon(eigen_vect dest_, double close_dist_, double speed_) {
  if (!is_initialized_) {
    return;
  }

  auto odom_uav = mrs_lib::get_mutexed(mutex_odom_uav_, odom_uav_);

  // getting new reference
  eigen_vect dir_vector_ = dest_ - odom_vector_;
  double     dist_       = dir_vector_.norm();
  double     normed_     = (dist_ - close_dist_) / dist_;
  eigen_vect goal_       = normed_ * dir_vector_ + odom_vector_;

  goal_(2, 0) = dest_(2, 0);
  goal_       = deadBand(dest_, goal_);
  dir_vector_ = goal_ - odom_vector_;

  dist_       = dir_vector_.norm();
  dir_vector_ = (dir_vector_ / dist_);

  eigen_vect                  cur_pos_ = odom_vector_;
  mrs_msgs::TrackerTrajectory new_traj_;
  eigen_vect                  diff_vector_;
  double                      angle_ = getBalloonHeading(dest_);
  if (_state_ == DESTROYING) {
    goal_(2, 0) += _height_offset_;
    if (getAngleBetween(angle_, odom_yaw_) > M_PI / 6) {
      ROS_INFO("[BalloonCircleDestroy]: Angle between drone and balloon is too big, abort");
      return;
    }   
  }

  mrs_msgs::TrackerPoint p;
  p.x   = cur_pos_(0, 0);
  p.y   = cur_pos_(1, 0);
  p.z   = cur_pos_(2, 0);
  p.yaw = angle_;
  new_traj_.points.push_back(p);

  geometry_msgs::PointStamped ref_;
  ref_.header.frame_id = world_frame_id_;
  ref_.point.x         = goal_(0, 0);
  ref_.point.y         = goal_(1, 0);
  ref_.point.z         = dest_(2, 0);
  point_pub_.publish(ref_);
  double     sample_dist_;
  double     acceleration = 2;
  eigen_vect cur_dir;

  // | --- calculate the speed in the direction to the target --- |
  double cur_speed = 0;
  {

    Eigen::Vector2d dir_vector(dir_vector_(0), dir_vector_(1));
    Eigen::Vector2d curr_pos(cur_pos_(0, 0), cur_pos_(1, 0));
    Eigen::Vector2d curr_vel(uav_velocity_arena_frame_.vector.x, uav_velocity_arena_frame_.vector.y);

    Eigen::MatrixXd projector = dir_vector * dir_vector.transpose();

    Eigen::Vector2d projection = projector * curr_vel;

    cur_speed = projection.norm();

    if (cur_speed < 0) {
      cur_speed = 0;
    }
    ROS_INFO_THROTTLE(0.1, "[Planner]: current speed: %.2f", cur_speed);
  }

  /* ROS_INFO("[]: cur_pos_ 0 x %f y %f z %f", cur_pos_(0, 0), cur_pos_(1, 0), cur_pos_(2, 0)); */
  /* ROS_INFO("[]: goal_ 0 x %f y %f z %f", goal_(0, 0), goal_(1, 0), goal_(2, 0)); */
  while (cur_pos_(0, 0) != goal_(0, 0) && cur_pos_(1, 0) != goal_(1, 0) && cur_pos_(2, 0) != goal_(2, 0)) {

    for (int i = 0; i < _traj_len_; i++) {
      if (cur_speed < speed_) {
        /* if (destroy_set) { */
        /*   double dt = ros::Time::now().toSec() - time_traj_sent_.toSec(); */
        /*   ROS_INFO("[]: dt is %f", dt); */
        /*   cur_speed += dt * acceleration; */
        /* } else { */
        cur_speed += 0.2 * acceleration;
        /* } */
      } else {
        cur_speed = speed_;
      }

      sample_dist_ = cur_speed * (_traj_time_ / _traj_len_);
      cur_dir      = dir_vector_ * sample_dist_;
      mrs_msgs::TrackerPoint p;
      cur_pos_     = cur_pos_ + cur_dir;
      diff_vector_ = cur_pos_ - odom_vector_;

      if (diff_vector_.norm() >= dist_) {
        cur_pos_ = goal_;
        break;
      }

      p.x = cur_pos_(0, 0);
      p.y = cur_pos_(1, 0);

      if (dist_ > _balloon_activation_dist_) {
        p.z = odom_vector_(2, 0);
      } else {

        p.z = dest_(2, 0);
      }
      if (_state_ == DESTROYING) {
        if(_cheating_mode_) {
          p.z = _cheating_height_;
        }
        p.z += _height_offset_;
      }
      ROS_INFO_THROTTLE(0.1, "[BallonCircleDestroy]: desired_height in traj: %.2f", p.z);

      p.yaw = angle_;

      /* ROS_INFO("[]: cur_pos_ 0 x %f y %f z %f", cur_pos_(0, 0), cur_pos_(1, 0), cur_pos_(2, 0)); */
      new_traj_.points.push_back(p);
    }
  }

  if (_state_ == DESTROYING) {
    new_traj_.points[new_traj_.points.size() - 1].z += _overshoot_offset_;
  }

  _last_goal_         = cur_pos_;
  _last_goal_reached_ = false;
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Trajectory ready ");
  new_traj_.header.frame_id = world_frame_id_;
  new_traj_.header.stamp    = ros::Time::now();
  new_traj_.fly_now         = true;
  new_traj_.use_yaw         = true;
  new_traj_.loop            = false;
  mrs_msgs::TrackerTrajectorySrv req_;
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: calling service to get closer ");
  req_.request.trajectory_msg = new_traj_;
  srv_client_trajectory_.call(req_);
  is_tracking_ = true;
}

//}

/* getBalloonHeading //{ */

double BalloonCircleDestroy::getBalloonHeading(eigen_vect dest_) {

  eigen_vect angle_vector_ = dest_ - odom_vector_;

  return atan2(angle_vector_(1), angle_vector_(0));
}


//}

/* getArenaHeading //{ */


double BalloonCircleDestroy::getArenaHeading(eigen_vect a_, eigen_vect b_) {

  eigen_vect angle_vector_ = a_ - b_;
  double     theta_        = atan2(angle_vector_(1), angle_vector_(0)) + M_PI;
  return theta_;
}

//}

/* getStateName //{ */

std::string BalloonCircleDestroy::getStateName() {
  switch (_state_) {
    case IDLE:
      return "IDLE";
    case GOING_AROUND:
      return "GOING_AROUND";
    case GOING_TO_BALLOON:
      return "GOING_TO_BALLOON";
    case AT_BALLOON:
      return "AT_BALLOON";
    case DESTROYING:
      return "DESTROYING";
    case CHECKING_BALLOON:
      return "CHECKING_BALLOON";
    case DESTROY_OVERSHOOT:
      return "DESTROY_OVERSHOOT";
  }
}
//}

/* plannerActivate //{ */

void BalloonCircleDestroy::plannerActivate(eigen_vect estimation_, double radius_) {
  if (ros::Time::now().toSec() - time_last_planner_reset_.toSec() < _wait_for_ball_) {
    ROS_INFO_THROTTLE(0.5, "[BalloonFilterPlanner]: Wait, activated a moment ago");
    return;
  }

  balloon_filter::StartEstimation        req_;
  balloon_filter::StartEstimationRequest rq_;
  rq_.header.frame_id = world_frame_id_;

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
      _planner_active_         = true;
      time_last_planner_reset_ = ros::Time::now();
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
      _planner_active_    = false;
      is_ballon_incoming_ = false;

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

  if (ros::Time::now().toSec() - time_last_planner_reset_.toSec() < _wait_for_ball_) {
    return false;
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

void BalloonCircleDestroy::addForbidden(eigen_vect forb_, double radius_) {
  ROS_WARN("[]: called add to forbidden");

  bool is_in_rad_ = false;
  for (int i = 0; i < int(_forb_vect_.size()); i++) {
    if ((_forb_vect_.at(i).vect_ - forb_).norm() < radius_) {
      is_in_rad_ = true;
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Couldn't add forbidden zone, already in the zone of another ");
    }
  }
  if (is_in_rad_) {
    return;
  }


  Forbidden_t forb_t_;
  forb_t_.r          = radius_;
  forb_t_.vect_      = forb_;
  forb_t_.lifetime   = 30;
  forb_t_.start_time = ros::Time::now();
  ROS_INFO("[StateMachine]: Added forbidden zone  ");

  _forb_vect_.push_back(forb_t_);
  balloon_filter::AddExclusionZone        req_;
  balloon_filter::AddExclusionZoneRequest rq_;

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

bool BalloonCircleDestroy::pointInForbidden(eigen_vect vect_) {
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
      /* addToForbidden(balloon_closest_vector_); */
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

eigen_vect BalloonCircleDestroy::getClosestBalloon() {

  double     dist_;
  double     best_dist_ = 999;
  eigen_vect ball_vect_;
  eigen_vect ball_vect_best_;
  {
    std::scoped_lock lock_uav(mutex_odom_uav_);
    /* std::scoped_lock lock_balloon(mutex_is_balloon_cloud_incoming_); */

    for (uint8_t i = 0; i < balloon_pcl_processed_.size(); i++) {

      ball_vect_ = balloon_pcl_processed_.at(i);

      dist_ = (odom_vector_ - ball_vect_).norm();
      if (dist_ < best_dist_) {

        ball_vect_best_ = ball_vect_;
        best_dist_      = dist_;
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
    _height_offset_           = config.height_offset;
    _dist_error_              = config.dist_error;
    _dist_to_balloon_         = config.dist_to_balloon;
    _wait_for_ball_           = config.wait_for_ball;
    _vel_                     = config.vel;
    _vel_arena_               = config.arena_vel;
    _overshoot_offset_        = config.overshoot_offset;
    _dead_band_factor_        = config.dead_band_factor;
    _balloon_activation_dist_ = config.balloon_activation_dist;
    _dist_kf_activation_      = config.dist_kf_activation;
  }
}

//}

/* droneStop //{ */

bool BalloonCircleDestroy::droneStop() {
  if (cur_constraints_ != _attack_constraints_) {
    setConstraints(_attack_constraints_);
  }
  std_srvs::Trigger srv_stop_call_;
  srv_client_stop_.call(srv_stop_call_);
  if (srv_client_stop_.call(srv_stop_call_)) {
    if (srv_stop_call_.response.success) {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Drone stopped");
      is_tracking_ = false;
      _mpc_stop_   = true;
    } else {
      ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Drone haven't been stopped");
    }

  } else {
    ROS_ERROR_THROTTLE(0.5, "[BalloonCircleDestroy]: Failed at calling drone stopping service");
  }

  return srv_stop_call_.response.success;
}

//}

/* isBalloonVisible //{ */

bool BalloonCircleDestroy::isBalloonVisible(eigen_vect balloon_) {

  double     dist_;
  eigen_vect ball_vect_;
  bool       res_ = false;
  {
    std::scoped_lock lock_uav(mutex_odom_uav_);
    /* std::scoped_lock lock_balloon(mutex_is_balloon_cloud_incoming_); */

    for (uint8_t i = 0; i < balloon_pcl_processed_.size(); i++) {


      ball_vect_ = balloon_pcl_processed_.at(i);

      dist_ = (balloon_ - ball_vect_).norm();
      if (dist_ < _dist_error_ + _dist_acc_) {
        res_                     = true;
        _last_time_balloon_seen_ = ros::Time::now();
        if (_state_ == GOING_TO_BALLOON) {
          if ((balloon_closest_vector_ - ball_vect_).norm() < _dist_error_) {
            balloon_closest_vector_ = ball_vect_;
            /* ROS_INFO("[]:  changed "); */
            geometry_msgs::PointStamped p_;
            p_.header.frame_id = world_frame_id_;
            p_.point.x         = balloon_closest_vector_(0, 0);
            p_.point.y         = balloon_closest_vector_(1, 0);
            p_.point.z         = balloon_closest_vector_(2, 0);
            balloon_pub_.publish(p_);
          }
        }
      }
    }
    std::string res_s = res_ ? "visible" : "not visible";
    ROS_INFO_THROTTLE(1.0, "[]: balloon at x %f y %f z %f is %s", balloon_(0, 0), balloon_(1, 0), balloon_(2, 0), res_s.c_str());
    if (_state_ == GOING_TO_BALLOON && res_ == false && ros::Time::now().toSec() - _last_time_balloon_seen_.toSec() < _max_time_balloon_) {
      res_ = true;
    }
  }
  return res_;
}

//}

/* isPointInArena //{ */

bool BalloonCircleDestroy::isPointInArena(float x, float y, float z) {
  bool is_x_ = x > _x_min_ && x < _x_max_;
  bool is_y_ = y > _y_min_ && y < _y_max_;
  bool is_z_ = z > _z_min_ && z < _z_max_;

  return is_x_ && is_y_ && is_z_;
}
bool BalloonCircleDestroy::isPointInArena(mrs_msgs::TrackerPoint p_) {
  bool is_x_ = p_.x > _x_min_ + _arena_offset_ && p_.x < _x_max_ - _arena_offset_;
  bool is_y_ = p_.y > _y_min_ + _arena_offset_ && p_.y < _y_max_ - _arena_offset_;
  bool is_z_ = p_.z > _z_min_ && p_.z < _z_max_;

  return is_x_ && is_y_ && is_z_;
}

bool BalloonCircleDestroy::isPointInArena(eigen_vect p_) {
  bool is_x_ = p_(0, 0) > _x_min_ + _arena_offset_ && p_(0, 0) < _x_max_ - _arena_offset_;
  bool is_y_ = p_(1, 0) > _y_min_ + _arena_offset_ && p_(1, 0) < _y_max_ - _arena_offset_;
  bool is_z_ = p_(2, 0) > _z_min_ && p_(2, 0) < _z_max_;

  return is_x_ && is_y_ && is_z_;
}

bool BalloonCircleDestroy::pointInSafety(float x, float y, float z) {
  bool in_polygon = cur_safety_->isPointInside(x, y);
  bool is_z_      = z > _z_min_ && z < _z_max_;
  return in_polygon && is_z_;
}

bool BalloonCircleDestroy::pointInSafety(eigen_vect p_) {
  bool in_polygon = cur_safety_->isPointInside(p_(0, 0), p_(1, 0));
  bool is_z_      = p_(2, 0) > _z_min_ && p_(2, 0) < _z_max_;
  return in_polygon && is_z_;
}

//}

/* scanArena //{ */

void BalloonCircleDestroy::scanArena() {
  mrs_msgs::TrackerTrajectory new_traj_;
  new_traj_.header.frame_id = world_frame_id_;
  new_traj_.header.stamp    = ros::Time::now();
  new_traj_.fly_now         = true;
  new_traj_.use_yaw         = true;
  new_traj_.loop            = false;
  int    step               = (_x_max_ - _x_min_ - _arena_offset_) / _fov_step_;
  double left               = _x_max_ - _arena_offset_ - _dist_acc_;
  double right              = _x_min_ + _arena_offset_ + _dist_acc_;
  ROS_INFO("[]: left %f right %f ", left, right);
  double top = _y_max_ - _arena_offset_ - _dist_acc_;
  double bot = _y_min_ + _arena_offset_ + _dist_acc_;
  ROS_INFO("[]: step size %d", step);
  eigen_vect cur_odom_ = odom_vector_;
  cur_odom_(2, 0)      = _height_;
  eigen_vect nxt;
  ROS_INFO("[]: to left %f to right %f bool %d", cur_odom_(0, 0) - left, cur_odom_(0, 0) - right,
           std::abs(cur_odom_(0, 0) - left) > std::abs(cur_odom_(0, 0) - right));

  if (_x_min_ > _x_max_ || _y_min_ > _y_max_) {
    ROS_ERROR("[StateMachine]: ERROR min max swapped");
  }
  bool dir = std::abs(cur_odom_(0, 0) - left) > std::abs(cur_odom_(0, 0) - right);
  if (!isPointInArena(cur_odom_(0, 0), cur_odom_(1, 0), cur_odom_(2, 0))) {
    if (cur_odom_(0, 0) > left && cur_odom_(0, 0) < right) {
      ROS_INFO("[]: is outside from the left side x %f y %f z %f", cur_odom_(0, 0), cur_odom_(1, 0), cur_odom_(2, 0));
    } else if (cur_odom_(0, 0) > right && cur_odom_(0, 0) < left) {
      ROS_INFO("[]: is outside from the right side x %f y %f z %f", cur_odom_(0, 0), cur_odom_(1, 0), cur_odom_(2, 0));
    }
  }


  for (int i = 0; i < step; i++) {

    if (dir) {
      // going from left to right
      nxt = eigen_vect(cur_odom_(0, 0), top, _height_);

      goToPoint(cur_odom_, nxt, _vel_arena_, new_traj_, getArenaHeading(cur_odom_, nxt));
      cur_odom_(1, 0) = top;
      nxt(0, 0) += _fov_step_;
      goToPoint(cur_odom_, nxt, _vel_arena_ / _arena_corner_factor_, new_traj_, getArenaHeading(cur_odom_, eigen_vect(nxt(0, 0), bot, 0)));
      cur_odom_(0, 0) += _fov_step_;
      nxt(1, 0) = bot;
      goToPoint(cur_odom_, nxt, _vel_arena_, new_traj_, getArenaHeading(cur_odom_, nxt));
      cur_odom_(1, 0) = bot;
      nxt(0, 0) += _fov_step_;
      goToPoint(cur_odom_, nxt, _vel_arena_ / _arena_corner_factor_, new_traj_, getArenaHeading(cur_odom_, eigen_vect(nxt(0, 0), top, 0)));
      cur_odom_(0, 0) += _fov_step_;


    } else {
      // going from right to left
      nxt = eigen_vect(cur_odom_(0, 0), top, _height_);
      goToPoint(cur_odom_, nxt, _vel_arena_, new_traj_, getArenaHeading(cur_odom_, nxt));
      cur_odom_(1, 0) = top;
      nxt(0, 0) -= _fov_step_;
      goToPoint(cur_odom_, nxt, _vel_arena_ / _arena_corner_factor_, new_traj_, getArenaHeading(cur_odom_, eigen_vect(nxt(0, 0), bot, 0)));
      cur_odom_(0, 0) -= _fov_step_;
      nxt(1, 0) = bot;
      goToPoint(cur_odom_, nxt, _vel_arena_, new_traj_, getArenaHeading(cur_odom_, nxt));
      cur_odom_(1, 0) = bot;
      nxt(0, 0) -= _fov_step_;
      goToPoint(cur_odom_, nxt, _vel_arena_ / _arena_corner_factor_, new_traj_, getArenaHeading(cur_odom_, eigen_vect(nxt(0, 0), top, 0)));
      cur_odom_(0, 0) -= _fov_step_;
    }
  }

  /* if (!isPointInArena(odom_vector_)) { */
  /*       ROS_INFO("[Sweeper] Outside of the arena, setting the start point 5 times"); */
  /*       for (int i = 0; i < 5; i++) { */
  /*         ROS_INFO("[Sweeper]: "); */
  /*         new_traj_.points.insert(new_traj_.points.begin(), new_traj_.points.at(0)); */
  /*       } */
  /*       for (int i = 0; i < 5; i++) { */
  /*         ROS_INFO_STREAM("point at 0 " <<new_traj_.points.at(i)); */

  /*       } */
  /*     } */
  mrs_msgs::TrackerTrajectorySrv req_;
  ROS_INFO("[BalloonCircleDestroy]: calling service to scan arena %d ", (int)new_traj_.points.size());
  req_.request.trajectory_msg = new_traj_;
  srv_client_trajectory_.call(req_);
  is_tracking_ = true;
}

//}

/* goToPoint //{ */

void BalloonCircleDestroy::goToPoint(eigen_vect p_, eigen_vect goal, double speed_, mrs_msgs::TrackerTrajectory& new_traj_, double yaw) {
  double     sample_dist_ = speed_ * (_traj_time_ / _traj_len_);
  eigen_vect dir_vector_  = goal - p_;

  double dist_   = dir_vector_.norm();
  double normed_ = (dist_) / dist_;
  /* eigen_vect goal_   = normed_ * dir_vector_ + goal; */
  /* /1* goal_(2, 0)             = p_(2, 0); *1/ */

  /* dir_vector_ = p_ - goal; */

  /* dist_       = dir_vector_.norm(); */
  dir_vector_ = (dir_vector_ / dist_) * sample_dist_;

  eigen_vect             cur_pos_ = p_;
  eigen_vect             diff_vector_;
  mrs_msgs::TrackerPoint p;
  p.x   = cur_pos_(0, 0);
  p.y   = cur_pos_(1, 0);
  p.z   = cur_pos_(2, 0);
  p.yaw = yaw;
  if (isPointInArena(p)) {
    new_traj_.points.push_back(p);
  }
  while (cur_pos_(0, 0) != goal(0, 0) || cur_pos_(1, 0) != goal(1, 0)) {


    for (int i = 0; i < _traj_len_; i++) {
      mrs_msgs::TrackerPoint p;
      cur_pos_ = cur_pos_ + dir_vector_;

      diff_vector_ = cur_pos_ - goal;

      if ((int)diff_vector_.norm() <= 0) {
        /* ROS_INFO("[BalloonCircleDestroy]: goToPoint returned %d ", (int)new_traj_.points.size()); */
        cur_pos_ = goal;
        p.x      = cur_pos_(0, 0);
        p.y      = cur_pos_(1, 0);
        p.z      = cur_pos_(2, 0);
        p.yaw    = yaw;
        if (!isPointInArena(p)) {
          return;
        }


        new_traj_.points.push_back(p);
        break;
      }


      p.x   = cur_pos_(0, 0);
      p.y   = cur_pos_(1, 0);
      p.z   = cur_pos_(2, 0);
      p.yaw = yaw;
      if (!isPointInArena(p)) {
        return;
      }


      new_traj_.points.push_back(p);
    }
  }
  ROS_INFO("[BalloonCircleDestroy]: goToPoint returned %d ", (int)new_traj_.points.size());
}

//}

/* goToHeight //{ */

void BalloonCircleDestroy::goToHeight(double height_, double speed_) {
  double     sample_dist_ = speed_ * (_traj_time_ / _traj_len_);
  eigen_vect goal         = odom_vector_;
  goal(2, 0)              = height_;

  /* ROS_INFO("[]: height 0 x %f y %f z %f", odom_vector_(0, 0), odom_vector_(1, 0), odom_vector_(2, 0)); */
  /* ROS_INFO("[]: height goal 0 x %f y %f z %f", goal(0, 0), goal(1, 0), goal(2, 0)); */
  eigen_vect dir_vector_ = goal - odom_vector_;

  double dist_      = dir_vector_.norm();
  dir_vector_       = (dir_vector_) / dist_;
  dir_vector_(2, 0) = 0.1;

  eigen_vect                  cur_pos_ = odom_vector_;
  eigen_vect                  diff_vector_;
  mrs_msgs::TrackerTrajectory new_traj_;

  mrs_msgs::TrackerPoint p;
  p.x = cur_pos_(0, 0);
  p.y = cur_pos_(1, 0);
  p.z = cur_pos_(2, 0);
  new_traj_.points.push_back(p);
  bool end_ = false;
  while (cur_pos_(2, 0) != goal(2, 0)) {

    if (end_) {
      break;
    }

    for (int i = 0; i < _traj_len_; i++) {
      mrs_msgs::TrackerPoint p;
      cur_pos_ = cur_pos_ + dir_vector_;

      diff_vector_ = cur_pos_ - goal;

      if (diff_vector_.norm() >= dist_) {
        cur_pos_ = goal;
        break;
      }

      p.x = cur_pos_(0, 0);
      p.y = cur_pos_(1, 0);
      p.z = cur_pos_(2, 0);
      /* if (!isPointInArena(p)) { */
      /*   break; */
      /* } */


      new_traj_.points.push_back(p);
      if (cur_pos_(2, 0) > height_) {
        end_ = true;
        break;
      }
    }
  }
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: Ascend  trajectory ready size: %d ", (int)new_traj_.points.size());
  new_traj_.header.frame_id = world_frame_id_;
  new_traj_.header.stamp    = ros::Time::now();
  new_traj_.fly_now         = true;
  new_traj_.use_yaw         = false;
  new_traj_.loop            = false;
  mrs_msgs::TrackerTrajectorySrv req_;
  ROS_INFO_THROTTLE(0.5, "[BalloonCircleDestroy]: calling service to get higher");
  req_.request.trajectory_msg = new_traj_;
  srv_client_trajectory_.call(req_);
  is_tracking_ = true;
}

//}

/* deadBand //{ */

eigen_vect BalloonCircleDestroy::deadBand(eigen_vect target_, eigen_vect reference_) {
  eigen_vect dir_vector_ = target_ - odom_vector_;
  dir_vector_            = dir_vector_ / dir_vector_.norm();

  // project the reference onto the plane between the target and the drone
  Eigen::Matrix3Xd projector  = dir_vector_ * dir_vector_.transpose();
  eigen_vect       projection = projector * (reference_ - odom_vector_);

  double factor_ = projection.norm();
  /* ROS_INFO("[]:  factor of dead band is %f ref factor is %f",_dead_band_factor_, factor_ ); */
  /* ROS_INFO("[]: odom  x %f y %f z %f", odom_vector_(0, 0), odom_vector_(1, 0), odom_vector_(2, 0)); */
  /* ROS_INFO("[]: reference  x %f y %f z %f", reference_(0, 0), reference_(1, 0), reference_(2, 0)); */

  if (std::abs(factor_) < _dead_band_factor_) {
    eigen_vect       old_reference_ = reference_;
    Eigen::Matrix3Xd eye;
    eye.setIdentity(3, 3);
    Eigen::Matrix3Xd plane_projector = eye - projector;
    reference_                       = plane_projector * (reference_ - odom_vector_) + odom_vector_;
    reference_(2, 0)                 = old_reference_(2, 0);
    /* ROS_INFO("[]: DEAD BAND"); */
  }
  return reference_;
}

//}

/* setArena //{ */

bool BalloonCircleDestroy::setArena(int i) {
  if (i > _arenas_.rows()) {
    ROS_INFO("[AutoStart]: Arena couldn't set to type %d is out of matrix range, redefine your arenas", i);
    return false;
  }
  _x_min_ = _arenas_(i, 0);
  _x_max_ = _arenas_(i, 1);

  _y_min_ = _arenas_(i, 2);
  _y_max_ = _arenas_(i, 3);

  _z_min_        = _arenas_(i, 4);
  _z_max_        = _arenas_(i, 5);
  _arena_offset_ = _arenas_(i, 6);

  switch (i) {
    case 0:
      _arena_type_ = 0;
      cur_safety_  = safety_polygon_1;
      break;
    case 1:
      _arena_type_ = 1;
      cur_safety_  = safety_polygon_2;
      break;

    case 2:
      _arena_type_ = 2;
      cur_safety_  = safety_polygon_;
      break;

    default:
      _arena_type_ = 2;
      cur_safety_  = safety_polygon_;
      break;
  }


  _arena_set_         = true;
  current_arena_time_ = ros::Time::now();
  ROS_INFO("[AutoStart]: Arena is set to type %d", i);
  ROS_INFO("[]: X min %f max %f Y min %f max %f", _x_min_, _x_max_, _y_min_, _y_max_);
  return true;
}

//}

/* changeState //{ */

void BalloonCircleDestroy::changeState(State state) {
  _prev_state_    = _state_;
  _state_         = state;
  time_state_set_ = ros::Time::now();
  ROS_WARN_THROTTLE(0.5, "[StateMachine]: State changed to %s time was set ", getStateName().c_str());
}

//}

/* getAngleBetween //{ */

double BalloonCircleDestroy::getAngleBetween(double a, double b) {
  double temp = a - b;

  return atan2(sin(temp), cos(temp));
}

//}

/* setConstraints() //{ */

void BalloonCircleDestroy::setConstraints(std::string desired_constraints) {

  mrs_msgs::String srv;
  srv.request.value = desired_constraints;

  ROS_INFO("[BalloonCircleDestroy]: setting constraints to \"%s\"", desired_constraints.c_str());

  bool res = srv_set_constriants_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[BalloonCircleDestroy]: service call for setConstraints() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[BalloonCircleDestroy]: service call for setConstraints() failed!");
  }
}

//}

/* abortEland //{ */

void BalloonCircleDestroy::abortEland() {

  std_srvs::Trigger srv_;
  bool              res = srv_eland_.call(srv_);
  if (res) {
    if (!srv_.response.success) {
      ROS_WARN_THROTTLE(0.5, "[StateMachine]: Service call for eland returned false");
    } else {
      ROS_WARN("[StateMachine]:  realsense is not going, called eland, turning off");
    }
  } else {
    ROS_WARN("[StateMachine]: Couldn't call eland");
  }

  _is_state_machine_active_ = false;
}

//}

// | --------------------- transformations -------------------- |
/* getTransform() method //{ */
bool BalloonCircleDestroy::getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& stamp,
                                        geometry_msgs::TransformStamped& transform_out) {
  try {
    transform_out = tf_buffer_.lookupTransform(to_frame, from_frame, stamp, ros::Duration(0.05));
    return true;
  }
  catch (tf2::TransformException& ex) {
    try {
      transform_out = tf_buffer_.lookupTransform(to_frame, from_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(0.5, "[BalloonCircleDestroy]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", from_frame.c_str(),
                        to_frame.c_str(), ex.what());
    }


    return false;
  }
}
//}

/* transformPointToWorld() method //{ */
bool BalloonCircleDestroy::transformPointToWorld(const geometry_msgs::Point& point, const std::string& from_frame, const ros::Time& stamp,
                                                 geometry_msgs::Point& point_out) {
  geometry_msgs::Point p_;
  p_.x = point.x;
  p_.y = point.y;
  p_.z = point.z;
  geometry_msgs::TransformStamped transform;
  if (!getTransform(from_frame, world_frame_id_, stamp - ros::Duration(0.2), transform))
    return false;


  tf2::doTransform(p_, point_out, transform);
  return true;
}
//}

/* transformQuaternion() method //{ */
bool BalloonCircleDestroy::transformQuaternionToUntilted(const geometry_msgs::Quaternion& point, const std::string& from_frame, const ros::Time& stamp,
                                                         geometry_msgs::Quaternion& point_out) {
  geometry_msgs::TransformStamped transform;
  if (!getTransform(from_frame, world_frame_id_, stamp, transform))
    return false;


  tf2::doTransform(point, point_out, transform);
  return true;
}
//}

/* transform pcl from World //{ */

bool BalloonCircleDestroy::transformPclToWorld(const PC::Ptr& pcl, const std::string& from_frame, const ros::Time& stamp, PC& pcl_out) {
  geometry_msgs::TransformStamped transform;

  if (!getTransform(from_frame, world_frame_id_, stamp, transform))
    return false;

  Eigen::Affine3d msg2odom_eigen_transform;
  msg2odom_eigen_transform = tf2::transformToEigen(transform);
  pcl::transformPointCloud(*pcl, pcl_out, msg2odom_eigen_transform);

  return true;
}


//}

}  // namespace balloon_circle_destroy

PLUGINLIB_EXPORT_CLASS(balloon_circle_destroy::BalloonCircleDestroy, nodelet::Nodelet);
