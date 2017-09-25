/*********************************************************************
*
* Software License Agreement (GPLv3)
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*********************************************************************/

// There is a really dumb compile error on Linux: "Eigen/src/Core/util/Constants.h:369:2:
// error: #error The preprocessor symbol 'Success' is defined, possibly by the X11 header file X.h",
// so this undef is necessary until we can figure out a better solution.
#if defined(Success)
#undef Success
#endif

#ifndef CODRHA_PLANNER_ROS_H_
#define CODRHA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <codrha_local_planner/CODRHAPlannerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/local_planner_util.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

// #include <base_local_planner/local_planner_util.h>
// #include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/trajectory.h>
//
// #include <base_local_planner/map_grid_cost_function.h>
// #include <base_local_planner/obstacle_cost_function.h>
// #include <base_local_planner/oscillation_cost_function.h>
// #include <base_local_planner/simple_scored_sampling_planner.h>
// #include <base_local_planner/odometry_helper_ros.h>

// #include <angles/angles.h>

// #include <nav_msgs/Odometry.h>

#include <codrha_local_planner/codrha_planner.h>
#include <codrha_local_planner/codrha_comm_ros.h>
// #include <unordered_set>

namespace codrha_local_planner {
/**
 * @class CODRHAPlannerROS
 * @brief ROS Wrapper for the CODRHAPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class CODRHAPlannerROS : public nav_core::BaseLocalPlanner {

public:
  /**
   * @brief  Constructor for CODRHAPlannerROS wrapper
   */
  CODRHAPlannerROS();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf::TransformListener *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  /**
   * @brief  Destructor for the wrapper
   */
  ~CODRHAPlannerROS();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base, using cotinuous optimal
   * decentralized dynamic window approach and a NCGPC-M control
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool codrhaComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose,
                                     geometry_msgs::Twist &cmd_vel);

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  bool isInitialized() { return initialized_; }

private:
  CODRHACommROS *comm_link_;
  /**
   * @brief Callback to update the local planner's parameters based on dynamic
   * reconfigure
   */
  void reconfigureCB(CODRHAPlannerConfig &config, uint32_t level);

  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);

  base_local_planner::Trajectory pathFromParamTrajectory(const Trajectory<double, MF::flatDim, MF::derivOrdForAccel+1> & traj);


  tf::TransformListener *tf_; ///< @brief Used for transforming point clouds

  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_;

  base_local_planner::LocalPlannerUtil planner_util_;

  boost::shared_ptr<CODRHAPlanner>
      codrha_planner_; ///< @brief The trajectory controller

  costmap_2d::Costmap2DROS *costmap_ros_;

  dynamic_reconfigure::Server<CODRHAPlannerConfig> *dsrv_;
  codrha_local_planner::CODRHAPlannerConfig default_config_;
  bool setup_;
  tf::Stamped<tf::Pose> current_pose_;
  // geometry_msgs / PoseWithCovariance
  //
  // std::vector<double> dy_mod_;
  std::unordered_set<std::string> comm_list_;

  base_local_planner::LatchedStopRotateController latchedStopRotateController_;

  bool initialized_;

  base_local_planner::OdometryHelperRos odom_helper_;
  std::string odom_topic_;
};
};
#endif
