#include "BalloonCircleDestroy.h"


#include <pluginlib/class_list_macros.h>

namespace balloon_circle_destroy {
/* init //{ */


  void BalloonCircleDestroy::onInit() {
    got_odom_uav_     = false;
    got_odom_gt_      = false;
    got_tracker_diag_ = false;

    is_tracking_      = false;

    ros::NodeHandle nh("~");

    ros::Time::waitForValid();

    
  // | ------------------- load ros parameters ------------------ |
/* params //{ */

    mrs_lib::ParamLoader param_loader(nh, "BalloonCircleDestroy");
    param_loader.load_param("arena_width", _arena_width_);
    param_loader.load_param("arena_length", _arena_length_);
    param_loader.load_param("arena_center_x", _arena_center_x_);
    param_loader.load_param("arena_center_y", _arena_center_y_);
    param_loader.load_param("arena_accuracy", _arena_accuracy_);
    param_loader.load_param("height", _height_);
    param_loader.load_param("idle_time", _idle_time_);
    param_loader.load_param("circling_radius", _circle_radius_);
    param_loader.load_param("circle_accuracy", _circle_accuracy_);
    param_loader.load_param("traj_time", _traj_time_);
    param_loader.load_param("traj_len", _traj_len_);
    param_loader.load_param("dist_to_balloon", _dist_to_balloon_);
    param_loader.load_param("vel", _vel_);
    param_loader.load_param("dist_error", _dist_error_);
    param_loader.load_param("wait_for_ball", _wait_for_ball_);

    param_loader.load_param("simulation", _simulation_);
    
    param_loader.load_param("rate/check_subscribers", _rate_timer_check_subscribers_);
    param_loader.load_param("rate/check_balloons", _rate_timer_check_balloons_);
    param_loader.load_param("rate/state_machine", _rate_timer_state_machine_);
    param_loader.load_param("rate/publish_goto", _rate_timer_publish_goto_);

    param_loader.load_param("world_frame_id", world_frame_id_);
    param_loader.load_param("world_point/x", world_point_x_);
    param_loader.load_param("world_point/y", world_point_y_);
    param_loader.load_param("world_point/z", world_point_z_);

    ROS_INFO_STREAM_ONCE("[BalloonCircleDestroy]: params loaded");

//}


    /* subscribe ground truth only in simulation, where it is available */
    // --------------------------------------------------------------
    // |                         subscribers                        |
    // --------------------------------------------------------------
    /* subscribers //{ */
    
    if (_simulation_) {
      sub_odom_gt_ = nh.subscribe("odom_gt_in", 1, &BalloonCircleDestroy::callbackOdomGt, this, ros::TransportHints().tcpNoDelay());
    }
    sub_odom_uav_     = nh.subscribe("odom_uav_in", 1, &BalloonCircleDestroy::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());
    sub_tracker_diag_ = nh.subscribe("tracker_diagnostics_in", 1, &BalloonCircleDestroy::callbackTrackerDiag, this, ros::TransportHints().tcpNoDelay());


    sub_balloon_point_ = nh.subscribe("balloon_point_in",1,&BalloonCircleDestroy::callbackBalloonPoint,this, ros::TransportHints().tcpNoDelay());
    sub_balloon_point_cloud_ = nh.subscribe("balloon_point_cloud_in",1,&BalloonCircleDestroy::callbackBalloonPointCloud,this, ros::TransportHints().tcpNoDelay());


    
    //}

    // | --------------- initialize service servers --------------- |
    //
/*  server services //{ */

  srv_server_circle_around_           = nh.advertiseService("circle_around", &BalloonCircleDestroy::callbackCircleAround, this);
  srv_server_go_closer_               = nh.advertiseService("go_closer", &BalloonCircleDestroy::callbackGoCloser, this);
  srv_server_start_state_machine_     = nh.advertiseService("start_state_machine", &BalloonCircleDestroy::callbackStartStateMachine, this);
  srv_server_stop_state_machine_     = nh.advertiseService("stop_state_machine", &BalloonCircleDestroy::callbackStartStateMachine, this);


    // init service client for publishing trajectories for the drone
    srv_client_trajectory_ = nh.serviceClient<mrs_msgs::TrackerTrajectorySrv>("trajectory_srv");
    srv_planner_reset_ = nh.serviceClient<std_srvs::Empty>("balloon_reset");


//}
    // | ---------- initialize dynamic reconfigure server --------- |
/* dynamic server //{ */

  /* reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh)); */
  /* ReconfigureServer::CallbackType f = boost::bind(&BalloonCircleDestroy::callbackDynamicReconfigure, this, _1, _2); */
  /* reconfigure_server_->setCallback(f); */



//} 
     // | -------------------- initialize timers ------------------- |
/* timers //{ */

  timer_check_subscribers_        = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &BalloonCircleDestroy::callbackTimerCheckSubscribers, this);
  // you can disable autostarting of the timer by the last argument
  timer_state_machine_ = nh.createTimer(ros::Rate(_rate_timer_state_machine_), &BalloonCircleDestroy::callbackTimerStateMachine, this, false, true);
  timer_check_balloons_ = nh.createTimer(ros::Rate(_rate_timer_check_balloons_), &BalloonCircleDestroy::callbackTimerCheckBalloonPoints, this, false, true);




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
    odom_uav_ = *msg;
    odom_vector_ = Eigen::Vector3d(odom_uav_.pose.pose.position.x,odom_uav_.pose.pose.position.y,odom_uav_.pose.pose.position.z );
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
    balloon_point_  = *msg;
    balloon_vector_ = Eigen::Vector3d(balloon_point_.pose.pose.position.x,balloon_point_.pose.pose.position.y,balloon_point_.pose.pose.position.z);
   }

   if(!got_balloon_point_) {
    got_balloon_point_ = true;
    ROS_INFO("[%s]: got first balloon point", ros::this_node::getName().c_str());
   }

   time_last_balloon_point_ = ros::Time::now();

 }
void BalloonCircleDestroy::callbackBalloonPointCloud(const sensor_msgs::PointCloudConstPtr& msg) {
   if (!is_initialized_) {
    return;
    }
  
   {
    std::scoped_lock lock(mutex_is_balloon_cloud_incoming_);
     balloon_point_cloud_  = *msg;
     
   }

   if(!got_balloon_point_cloud_) {
    got_balloon_point_cloud_ = true;
    ROS_INFO("[%s]: got first balloon point cloud", ros::this_node::getName().c_str());
   }

   time_last_balloon_cloud_point_ = ros::Time::now();

 }


//}

 void BalloonCircleDestroy::callbackTrackerDiag(const mrs_msgs::TrackerDiagnosticsConstPtr& msg) {
  if(!is_initialized_) {
    return;
  }
  if (!got_tracker_diag_) {
    got_tracker_diag_ = true;
    ROS_INFO("[%s]: got first tracker diagnostics msg", ros::this_node::getName().c_str());
  }

  if(is_tracking_ && msg->tracking_trajectory && _state_ == GOING_AROUND) {

    Eigen::Vector3d ball_vect;
    double cur_dist_;
    


    for (unsigned long i = 0; i < balloon_point_cloud_.points.size() ; i++) {
        geometry_msgs::Point p_ ;

        bool ts_res = transformPointFromWorld(balloon_point_cloud_.points.at(i), balloon_point_cloud_.header.frame_id,balloon_point_cloud_.header.stamp,p_);
        if (!ts_res) {
          ROS_WARN_THROTTLE(1,"[BalloonCircleDestroy]: No transform,skipping");
          return;
        }  
                
        ball_vect = Eigen::Vector3d(p_.x,p_.y,p_.z);
        cur_dist_ =  (ball_vect - odom_vector_).norm();
        if (cur_dist_ < _closest_on_arena_) {
          _closest_on_arena_ = cur_dist_;
          _closest_angle_ = getArenaHeading();
        }
        
    }
  
  } 

  if (is_tracking_ && !msg->tracking_trajectory) {
    ROS_INFO("[%s]: reached final point", ros::this_node::getName().c_str());
    std::scoped_lock lock(mutex_is_tracking_);
    is_tracking_ = false;
    is_idling_ =  true;
    ros::NodeHandle nh("~");
    timer_idling_ = nh.createTimer(ros::Duration(_idle_time_), &BalloonCircleDestroy::callbackTimerIdling, this, true);  // the last boolean argument makes the timer run only once
    ROS_INFO("[BalloonCircleDestroy]: Idling for %2.2f seconds.", _idle_time_);
  }

  time_last_tracker_diagnostics_ = ros::Time::now();

 }
void BalloonCircleDestroy::callbackTimerIdling([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[%s]: Idling stopped", ros::this_node::getName().c_str());
  if (_state_ == GOING_TO_ANGLE) {
    ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: Finished observing arena");
  
  }
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

void BalloonCircleDestroy::callbackTimerStateMachine([[maybe_unused]] const ros::TimerEvent& te) {
  if (!_is_state_machine_active_) {
    return;

  }
  ROS_INFO_THROTTLE(0.5, "// | ---------------- STATE MACHINE LOOP STATUS --------------- |");
  ROS_INFO_THROTTLE(0.5, "[State]: %s ", getStateName().c_str());
  ROS_INFO_THROTTLE(0.5,"[ClosestAngle]: %f ", _closest_angle_);
  ROS_INFO_THROTTLE(0.5,"[CurrentAngle]: %f ", getArenaHeading());
  ROS_INFO_THROTTLE(0.5,"[ClosestDist]: %f ", _closest_on_arena_);
  ROS_INFO_THROTTLE(0.5,"[Current Dist To ball]: %f ", (odom_vector_ - balloon_vector_).norm());

  ROS_INFO_THROTTLE(0.5, "// | ----------------- STATE MACHINE LOOP END ----------------- |");

  if (_state_ == IDLE) {
    _state_ = GOING_AROUND;
    _closest_on_arena_ = 9999;
    ROS_WARN_THROTTLE(1.0, "[]: STATE RESET TO %s",getStateName().c_str() );
    goAroundArena();
    return;

  } else if (_state_ == GOING_AROUND) {
    if (!is_tracking_) {
      _state_  = GOING_TO_ANGLE;
      ROS_WARN_THROTTLE(1.0, "[]: STATE RESET TO %s",getStateName().c_str() );
      goAroundArena();
      return;}
    } else if (_state_ == GOING_TO_ANGLE) {
      if(!is_tracking_) {
          _state_ = CHOOSING_BALLOON;
          ROS_WARN_THROTTLE(1.0, "[]: STATE RESET TO %s",getStateName().c_str() );
          return;
      
      }
    } else if (_state_ == CHOOSING_BALLOON) {
      if( (odom_vector_ - balloon_vector_).norm() < _closest_on_arena_+_dist_error_) {

        if (time_last_balloon_point_.toSec() < _wait_for_ball_) {
            std_srvs::Empty req;
            srv_planner_reset_.call(req);
            ROS_WARN_THROTTLE(1.0,"[StateMachine]: req %s ", req.response);
        }
        ROS_INFO_THROTTLE(1.0, "[StateMachine]: Ball found, point is in the right distance");
      }
    }
  }


/* callbackTimerCheckSubcribers //{ */


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
 if (!got_balloon_point_cloud_) {
  ROS_WARN_THROTTLE(1.0, "[%s]: haven't received balloon point cloud since launch", ros::this_node::getName().c_str());
 } else {
  if ((time_now - time_last_balloon_cloud_point_).toSec() > 1.0) {
    ROS_WARN_THROTTLE(1.0, "[%s]: haven't received any balloon cloud points for %f", ros::this_node::getName().c_str(), (time_now - time_last_balloon_cloud_point_).toSec());
  }
 }
}

//}


/* callbackTimerCheckBalloonPoints //{ */

void BalloonCircleDestroy::callbackTimerCheckBalloonPoints([[maybe_unused]] const ros::TimerEvent& te) {
  
  if(!is_initialized_) {
    return;
  }
 /* ROS_INFO_THROTTLE(1.0, "[%s]: Got balloon point at x %f ", ros::this_node::getName().c_str(), balloon_point_.pose.pose.position.x); */ 
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
    ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: Start circling, circle radius %f",_circle_radius_ );
    getCloseToBalloon();
    mrs_msgs::TrackerTrajectory new_trj_;
    new_trj_.header.frame_id = "local_origin";
    new_trj_.header.stamp    = ros::Time::now();
    new_trj_.fly_now = true;
    new_trj_.loop = false;
    new_trj_.use_yaw = true;
    new_trj_.start_index = 0;
    double iterat = M_PI/(_circle_accuracy_/2);
    double angle = 0;

    for (int i = 0; i <_circle_accuracy_; i++) {
      mrs_msgs::TrackerPoint point;
      point.x = balloon_point_.pose.pose.position.x + cos(angle)*_circle_radius_;
      point.y = balloon_point_.pose.pose.position.y + sin(angle)*_circle_radius_;
      point.z = balloon_point_.pose.pose.position.z;
      point.yaw = angle + M_PI;
      angle +=iterat;
      
      new_trj_.points.push_back(point);
    }
  

    ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroyer]: publishing trajectory ");
    mrs_msgs::TrackerTrajectorySrv req_;
    req_.request.trajectory_msg = new_trj_;
    
    srv_client_trajectory_.call(req_);
    

    res.success = req_.response.success;
    res.message = req_.response.message;
    ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: Result of circle service %d ", res.success);
    is_tracking_ = true;
    return res.success;

}

//}



/* callbackCircleAround //{ */

bool BalloonCircleDestroy::callbackGoCloser([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {
  
    ROS_WARN_THROTTLE(1.0, "[BalloonCircleDestroy]: Could'nt call the service, node isn't initialized");
    res.message = "Node isn't initialized";
    res.success = false;
    return false;

  }

  getCloseToBalloon();
  ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: I am at the balloon at %f",_dist_to_balloon_);
  res.message = "Getting close to the balloon";
  res.success = true;

  return true;

}

//}

/* callbackStartStateMachine //{ */


bool BalloonCircleDestroy::callbackStartStateMachine([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

    if(!is_initialized_) {
      res.success = false;
      res.message = "Can't trigger service, not initialized";
      ROS_WARN_THROTTLE(1.0, "[BalloonCircleDestroy]: Could'nt call the service, not inited yet");

      return false;
    }
    if (_is_state_machine_active_) {
      _is_state_machine_active_ = false;
      res.message = "State machine disabled";
      
    } else {
      _is_state_machine_active_ = true;
      res.message = "State machine activated";
    }
 
    _state_ = IDLE;
    res.success = true;
    return true;

}

//}

/* getCloseToBalloon //{ */

void BalloonCircleDestroy::getCloseToBalloon() {
   
  double sample_dist_ = _vel_* (_traj_len_/_traj_time_);
  Eigen::Vector3d dir_vector_ = balloon_vector_ - odom_vector_;

  double dist_ = dir_vector_.norm();
  double normed_ = (dist_ - _dist_to_balloon_)/dist_;
  Eigen::Vector3d goal_ = normed_*dir_vector_ + odom_vector_;
  goal_(2,0) = balloon_vector_(2,0);
  dir_vector_ = goal_ - odom_vector_; 

  dist_ = dir_vector_.norm();
  dir_vector_ = (dir_vector_/dist_) * sample_dist_;

  Eigen::Vector3d cur_pos_ = odom_vector_;
  mrs_msgs::TrackerTrajectory new_traj_;
  Eigen::Vector3d diff_vector_;
  double angle_ = getBalloonHeading() + M_PI;

  while (cur_pos_(0,0) != goal_(0,0) && cur_pos_(1,0)!=goal_(1,0) && cur_pos_(2,0) != goal_(2,0)) {
  
    for (int i = 0; i < _traj_len_; i++) {
      mrs_msgs::TrackerPoint p;
      cur_pos_ = cur_pos_ + dir_vector_;
      diff_vector_ = cur_pos_ - odom_vector_;

      if (diff_vector_.norm() >= dist_) {
        cur_pos_ = goal_;
      }

      p.x = cur_pos_(0,0);
      p.y = cur_pos_(1,0);
      p.z = cur_pos_(2,0);
      p.yaw = angle_;


      new_traj_.points.push_back(p);
      
    }
  
    
  }
  ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: Trajectory ready ");
    new_traj_.header.frame_id = "local_origin";
    new_traj_.header.stamp    = ros::Time::now();
    new_traj_.fly_now         = true;
    new_traj_.use_yaw         = true;
    new_traj_.loop            = false;
    new_traj_.start_index     = 0;
    mrs_msgs::TrackerTrajectorySrv req_;
    ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: calling service to get closer ");
    req_.request.trajectory_msg = new_traj_;
    srv_client_trajectory_.call(req_);
    is_tracking_ = true;


 }

//}

/* getBalloonHeading //{ */

double BalloonCircleDestroy::getBalloonHeading() {
  
    Eigen::Vector3d angle_vector_ = balloon_vector_  - odom_vector_;
    
    return atan2(angle_vector_(1),angle_vector_(0));
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

    if(_state_ !=State::GOING_TO_ANGLE) {
      ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: Can't circle around arena, drone state isn't GOING_AROUND or GOING_TO_ANGLE but is %s ", getStateName().c_str());
      return;
    }
  } 


    mrs_msgs::TrackerTrajectory new_trj_;
    new_trj_.header.frame_id = "local_origin";
    new_trj_.header.stamp    = ros::Time::now();
    new_trj_.fly_now = true;
    new_trj_.loop = false;
    new_trj_.use_yaw = true;
    new_trj_.start_index = 0;
    double iterat = M_PI/(_arena_accuracy_/2);
    

    Eigen::Vector3d angle_vector_ = Eigen::Vector3d(_arena_center_x_,_arena_center_y_,_height_) - odom_vector_; 
    double angle = atan2(angle_vector_(1), angle_vector_(0)) + M_PI;
    ROS_INFO("[]: angle %f",angle );
    /* double angle = 0; */
    bool lower = false;
    if (angle > _closest_angle_) {
      lower = true;
    }

    for (int i = 0; i <_arena_accuracy_; i++) {
      mrs_msgs::TrackerPoint point;
      point.x = _arena_center_x_ + cos(angle)*_arena_width_/2;
      point.y = _arena_center_y_ + sin(angle)*_arena_length_/2;
      point.z = _height_;
      point.yaw = angle + M_PI;
      if (_state_ == State::GOING_TO_ANGLE && lower) {
        angle -= iterat;
      }else {
        angle +=iterat;
      }
      new_trj_.points.push_back(point);
      // if state is going to angle  we should stop at the closest balloon found
      if( _state_ == State::GOING_TO_ANGLE) {
        if (lower && angle < _closest_angle_) {
            break;
        }
          if (angle > _closest_angle_ && !lower) {
            break;
          }
      } 
    }
  

    ROS_WARN_THROTTLE(1.0, "[Traj]: %d ", new_trj_.points.size());
    ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroyer]: publishing trajectory ");
    mrs_msgs::TrackerTrajectorySrv req_;
    req_.request.trajectory_msg = new_trj_;
    
    srv_client_trajectory_.call(req_);
    is_tracking_ = true;
   

}

//}


/* goToChosenBalloon //{ */


void BalloonCircleDestroy::goToChosenBalloon() {
  if(!is_initialized_) {
    return;
  }

  if(_state_ != State::GOING_TO_BALLOON) {
    ROS_INFO_THROTTLE(1.0, "[StateMachine]: State is not GOINT TO BALLOON, but is %s",getStateName().c_str() );
  }

  double sample_dist_ = _vel_* (_traj_len_/_traj_time_);
  Eigen::Vector3d dir_vector_ = balloon_vector_ - odom_vector_;

  double dist_ = dir_vector_.norm();
  double normed_ = (dist_ - _dist_to_balloon_)/dist_;
  Eigen::Vector3d goal_ = normed_*dir_vector_ + odom_vector_;
  goal_(2,0) = balloon_vector_(2,0);
  dir_vector_ = goal_ - odom_vector_; 

  dist_ = dir_vector_.norm();
  dir_vector_ = (dir_vector_/dist_) * sample_dist_;

  Eigen::Vector3d cur_pos_ = odom_vector_;
  mrs_msgs::TrackerTrajectory new_traj_;
  Eigen::Vector3d diff_vector_;
  double angle_ = getBalloonHeading() + M_PI;

  while (cur_pos_(0,0) != goal_(0,0) && cur_pos_(1,0)!=goal_(1,0) && cur_pos_(2,0) != goal_(2,0)) {
  
    for (int i = 0; i < _traj_len_; i++) {
      mrs_msgs::TrackerPoint p;
      cur_pos_ = cur_pos_ + dir_vector_;
      diff_vector_ = cur_pos_ - odom_vector_;

      if (diff_vector_.norm() >= dist_) {
        cur_pos_ = goal_;
      }

      p.x = cur_pos_(0,0);
      p.y = cur_pos_(1,0);
      p.z = cur_pos_(2,0);
      p.yaw = angle_;


      new_traj_.points.push_back(p);
      
    }
  
    
  }
  ROS_INFO_THROTTLE(1.0, "[BalloonCircleDestroy]: Trajectory ready ");
    new_traj_.header.frame_id = "local_origin";
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
    Eigen::Vector3d angle_vector_ = Eigen::Vector3d(_arena_center_x_,_arena_center_y_,_height_) - odom_vector_; 
    return atan2(angle_vector_(1), angle_vector_(0)) + M_PI;

}

//}

/* getStateName //{ */

std::string BalloonCircleDestroy::getStateName() {
    switch(_state_) {
      case IDLE: return "IDLE";
      case GOING_AROUND: return "GOING_AROUND";
      case GOING_TO_ANGLE: return "GOING_TO_ANGLE";
      case CHOOSING_BALLOON: return "CHOOSING_BALLOON";
      case GOING_TO_BALLOON: return "GOING_TO_BALLOON";
      case AT_BALLOON: return "AT_BALLOON";
      case CIRCLE_AROUND: return "CIRCLE_AROUND";
      case DESTROYING: return "DESTROYING";
    }
  
}
//}


// | --------------------- transformations -------------------- |
/* getTransform() method //{ */
bool BalloonCircleDestroy::getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::TransformStamped& transform_out)
{
  try
  {
    transform_out = tf_buffer_.lookupTransform(to_frame, from_frame, stamp);
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "[BalloonCircleDestroy]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", from_frame.c_str(), to_frame.c_str(), ex.what());
    return false;
  }
}
//}

/* transformPointFromWorld() method //{ */
bool BalloonCircleDestroy::transformPointFromWorld(const geometry_msgs::Point32& point, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::Point& point_out)
{
  geometry_msgs::Point p_;
  p_.x = point.x;
  p_.y = point.y;
  p_.z = point.z;
  geometry_msgs::TransformStamped transform;
  if (!getTransform(to_frame,world_frame_id_, stamp, transform))
    return false;
  

  tf2::doTransform(p_, point_out, transform);
  return true;
}
//}


} // namespace balloon_circle_destroy

PLUGINLIB_EXPORT_CLASS(balloon_circle_destroy::BalloonCircleDestroy, nodelet::Nodelet);
