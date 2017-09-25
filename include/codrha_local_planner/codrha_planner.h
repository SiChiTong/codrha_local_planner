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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*********************************************************************/

#ifndef CODRHA_PLANNER_H_
#define CODRHA_PLANNER_H_

#include <base_local_planner/local_planner_util.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <codrha_local_planner/codrha_controller.h>
#include <codrha_local_planner/CODRHAPlannerConfig.h>
#include <codrha_local_planner/monocycle_flatoutput.h>
#include <codrha_local_planner/codrha_comm.h>
#include <costmap_2d/costmap_2d.h>
#include <Eigen/Core>
#include <fstream>
#include <vector>
#include <limits>
#include <iostream>
#include <nlopt/nlopt.h>
#include <nav_msgs/Path.h>
#include <unordered_set>

#define OPT_DEBUG_ENABLED (1)

namespace codrha_local_planner
{

typedef std::map< std::string, Eigen::Matrix < double,  MF::positionDim, Eigen::Dynamic > > DictSharedPath;


/**
 * @class CODRHAPlanner
 * @brief A class implementing a local planner using the Continous Optimal
 *        Decentralized Receding Horizon Approach
 */
class CODRHAPlanner
{

public:

/**
 * @brief  Constructor for the planner
 * @param name The name of the planner
 * @param costmap_2d The local 2d costmap to be used by the planner
 */
CODRHAPlanner(std::string name, costmap_2d::Costmap2D* costmap_ros, CODRHAComm* comm_link, const long unsigned sec, const long unsigned nsec);

/**
 * @brief  Destructor for the planner
 */
~CODRHAPlanner();

/**
 * @brief Reconfigures the motion planner
 * @param cfg_plan_time                         Planning horizon
 * @param cfg_plan_granularity                  Sampling period of trajectory for constraints evaluation
 * @param cfg_comp_time                         Implementation/Computation horizon
 * @param cfg_traj_spl_n_ctrl_pts               Number of control points for b-spline representation of a trajectory
 * @param cfg_opt_method                        Name of the optimization method
 * @param cfg_num_dif_eps                       Epsilon value (h) used for differentiation (gradient computation)
 * @param cfg_opt_objective_func_abs_tol        The optimization stops if |&Delta;f|/|f| is less than cfg_opt_objective_func_rel_tol, or |&Delta;f| is less than cfg_opt_objective_func_abs_tol
 * @param cfg_opt_objective_func_rel_tol        The optimization stops if |&Delta;f|/|f| is less than cfg_opt_objective_func_rel_tol, or |&Delta;f| is less than cfg_opt_objective_func_abs_tol
 * @param cfg_opt_param_abs_tol                 The optimization stops if |&Delta;x|/|x| is less than cfg_opt_param_func_rel_tol, or |&Delta;x| is less than cfg_opt_param_func_abs_tol
 * @param cfg_opt_param_rel_tol                 The optimization stops if |&Delta;x|/|x| is less than cfg_opt_param_func_rel_tol, or |&Delta;x| is less than cfg_opt_param_func_abs_tol
 * @param cfg_opt_equetions_abs_tol             The optimization stops if |&Delta;e|/|e| is less than cfg_opt_equations_func_rel_tol, or |&Delta;e| is less than cfg_opt_equations_func_abs_tol
 * @param cfg_opt_inequetions_abs_tol           The optimization stops if |&Delta;i|/|i| is less than cfg_opt_inequations_func_rel_tol, or |&Delta;i| is less than cfg_opt_inequations_func_abs_tol
 * @param cfg_max_vel_x                         Max linear velocity
 * @param cfg_max_vel_theta                     Max angular velocity
 * @param cfg_acc_lim_x                         Max linear acceleration
 * @param cfg_acc_lim_theta                     Min linear acceleration
 * @param cfg_acc_sup_x_init                    Max linear acceleration used for first guess before optimization
 * @param cfg_acc_inf_x_init                    Min linear acceleration used for first guess before optimization
 * @param cfg_timeout_for_computing_first_plan  Timeout for computing first plan
 * @param cfg_robot_obst_safety_dist            Safety distance between robot and obstacles
 * @param cfg_radius                            Robot footprint
 * @param cfg_last_step_min_dist                Distance added in the distance for going to the last step
 * @param cfg_first_guess_tweak                 Defines the shape of the first guess trajectories
 * @param cfg_cost_tweak                        Adds to the cost for reaching the goal position
 * @param cfg_max_vel_init_factor               Multiply max linear velocity for first guess (it is used to make sure the initialization begins with some margin from violating constraints)
 * @param cfg_dy_mod_params                     Dynamic model parameters
 * @param cfg_ctrl_prediction_time              Predition time used by the MPC (NCGPC-M)
 * @param cfg_ctrl_integ_step                   Time step for the integration of reference trajectory inside the NCGPC-M
 * @param cfg_respect_obst_const                If true obstacles avoidance constraints will be respected, if false they will not
 * @param cfg_respect_obst_const_term           If true obstacles avoidance constraints in the termination phase will be respected, if false they will not
 * @param cfg_respect_accel_const               If true acceleration constraints will be respected, if false they will not
 * @param cfg_respect_accel_const_term          If true acceleration constraints in termination phase will be respected, if false they will not
 */
void reconfigure (
        double cfg_plan_time, double cfg_plan_granularity, double cfg_comp_time,
        int cfg_traj_spl_n_ctrl_pts, std::string cfg_opt_method,
        double cfg_num_dif_eps, double cfg_opt_objective_func_abs_tol,
        double cfg_opt_objective_func_rel_tol, double cfg_opt_param_abs_tol,
        double cfg_opt_param_rel_tol, double cfg_opt_equetions_abs_tol,
        double cfg_opt_inequetions_abs_tol, double cfg_max_vel_x,
        double cfg_max_vel_theta, double cfg_acc_lim_x, double cfg_acc_lim_theta,
        double cfg_acc_sup_x_init, double cfg_acc_inf_x_init,
        double cfg_timeout_for_computing_first_plan,
        double cfg_robot_obst_safety_dist, double cfg_radius,
        double cfg_last_step_min_dist, double cfg_first_guess_tweak, double cfg_cost_tweak,
        double cfg_max_vel_init_factor, std::vector<double> & cfg_dy_mod_params,
        double cfg_ctrl_prediction_time, double cfg_ctrl_integ_step,
        bool cfg_respect_obst_const, bool cfg_respect_obst_const_term,
        bool cfg_respect_accel_const, bool cfg_respect_accel_const_term, const std::unordered_set<std::string>& comm_list);

/**
 * Given the state feedback of the robot, find the best velocity commands to execute
 * @param  world_time            Accurate and precise clock time
 * @param  x_feedback            Robot's current position in x in the global frame
 * @param  y_feedback            Robot's current position in y in the global frame
 * @param  psi_feedback          Robot's current orientation in z in the global frame
 * @param  target_x_feedback     Target position in x in the global frame
 * @param  target_y_feedback     Target position in y in the global frame
 * @param  target_psi_feedback   Target orientation in z in the global frame
 * @param  target_v_feedback     Target linear velocity in the global frame
 * @param  target_omega_feedback Target angular velocity in the global frame
 * @param  v_feedback            Robot's current linear velocity in the global frame
 * @param  omega_feedback        Robot's current angular velocity in the global frame
 * @param  lin_vel_output        Argument where to store the computed linear velocity command
 * @param  ang_vel_output        Argument where to store the computed angular velocity command
 * @return                       True if successful, false otherwise
 */
bool computeVelocityCommands(
        long unsigned world_time_sec,
        long unsigned world_time_nsec,
        double x_feedback,
        double y_feedback,
        double psi_feedback,
        double target_x_feedback,
        double target_y_feedback,
        double target_psi_feedback,
        double target_v_feedback,
        double target_omega_feedback,
        double v_feedback,
        double omega_feedback,
        double& lin_vel_output,
        double& ang_vel_output);

private:

DictSharedPath paths_from_collision_robots_;

CODRHAComm* comm_link_;
std::unordered_set<std::string> comm_list_;
std::map<std::string, IntendedTraj> others_;

std::string name_;

costmap_2d::Costmap2D* costmap_;
costmap_2d::Costmap2D thread_costmap_;
MF::PoseVectorD costmap_ref_;

std::ofstream traj_log_, traj_log_map_, opt_log_, feedback_log_, command_log_;
Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic> curve_;

double max_velo_in_plan_;

// Underlining trajectory variable
Trajectory<double, MF::flatDim, MF::derivOrdForAccel+1> trajectory_, opt_trajectory_;

// double stop_time_buffer_;   ///< @brief How long before hitting something we're
/// going to enforce that the robot stop
// double pdist_scale_, gdist_scale_, occdist_scale_;
// Eigen::Vector3f vsamples_;

// double sim_period_;   ///< @brief The number of seconds to use to compute
/// max/min vels for dwa
// base_local_planner::Trajectory result_traj_;

double forward_point_distance_;

// std::vector<geometry_msgs::PoseStamped> global_plan_;


boost::mutex configuration_mutex_;
// pcl::PointCloud<base_local_planner::MapGridCostPoint> *traj_cloud_;
// pcl_ros::Publisher<base_local_planner::MapGridCostPoint> traj_cloud_pub_;
bool publish_cost_grid_pc_;                   ///< @brief Whether or not to build and publish a
                                              /// PointCloud
bool publish_traj_pc_;

double cheat_factor_;

// affected by reconfigure ============================================
double plan_time_;
double plan_granularity_;
double comp_time_;
double time_of_exec_;
int traj_spl_n_ctrl_pts_;
std::string opt_method_;
double num_dif_eps_;
double opt_objective_func_abs_tol_;
double opt_objective_func_rel_tol_;
double opt_param_abs_tol_;
double opt_param_rel_tol_;
double opt_equetions_abs_tol_;
double opt_inequetions_abs_tol_;
double max_vel_x_;
double max_vel_theta_;
double max_acc_x_;
double max_acc_theta_;
double acc_sup_x_init_;
double acc_inf_x_init_;
double timeout_for_computing_first_plan_;
double derivative_eps_;
double robot_obst_safety_dist_;
double radius_;
MF::PoseVectorD target_pt_pose_;
MF::VeloVectorD target_pt_velocity_;
MF::PoseVectorD way_pt_pose_;
MF::VeloVectorD way_pt_velocity_;
double last_step_min_dist_;
double min_cost_v_;
double *init_params_;
double *params_;

bool respect_obst_const_;
bool respect_obst_const_term_;
bool respect_accel_const_;
bool respect_accel_const_term_;

// timers =============================================================
int idx_traj_being_computed_;
int idx_traj_being_executed_;
double time_first_call_of_comp_vel_cmds_;
double time_since_first_call_;
double eval_time_;
double time_impl_of_first_traj_;
double inter_rob_saf_dist_;
double world_time_;
long unsigned world_time_sec_;
long unsigned world_time_nsec_;
long unsigned thread_world_time_sec_;
long unsigned thread_world_time_nsec_;
double thread_eval_time_;

std::map<std::string, Eigen::Matrix<double, MF::positionDim, -1> > path_from_conflictual_vehicles_;
std::map<std::string, double> radius_from_conflictual_vehicles_;
std::map<std::string, double> inter_rob_saf_from_conflictual_vehicles_;

// set by the planning thread
bool last_traj_flag_;

// multithreading management ==========================================
boost::mutex planning_thread_mutex_;
boost::thread planning_thread_;
boost::mutex timers_mutex_;

// optimization =======================================================
nlopt_opt opt_pbl_;
unsigned opt_iteractions_counter_;

double first_guess_tweak_;
double cost_tweak_;
double max_vel_init_factor_;

double opt_plan_time_;

Eigen::Matrix<double, MF::positionDim, MF::positionDim> rot_mat_2_wfr_;
Eigen::Matrix<double, MF::positionDim, MF::positionDim> rot_mat_2_rfr_;
// Eigen::Matrix<double, MF::positionDim, MF::positionDim> rot_mat_2_cmapfr_;

MF::PositionVectorD latest_position_;
MF::PoseVectorD latest_pose_;
MF::VeloVectorD latest_velocity_;

// MapObst detected_obstacles_;

std::unordered_set<std::string> collision_vehicles_;
std::unordered_set<std::string> comm_outrage_vehicles_;

// low level controller ===============================================
boost::shared_ptr<Controller> low_level_ctrl_;


double costmap_2d_INSCRIBED_INFLATED_OBSTACLE_, cell_value_at_rad_dist_;

void getCostmapInfo();

/**
 * Calls process of finding a optimized trajectory
 * @return True if successful, false otherwise
 */
bool findTrajectory();

/**
 * Generate first guess as initialization for the optimization part of find the trajectory
 * @param rem_dist        Remaining distance to reach the waypoint as of Tc into the previous trajectory
 * @param change_vel_dist Minimum distance needed for going from latest velocity to the waypoint's velocity
 * @param curr_direc      Orientation of the robot as of Tc into the previous trajectory
 * @param direc_to_wp     Orientation for arriving at the waypoint
 * @param curve           Computed first guess
 */
void generateOptInitGuess(
        double rem_dist,
        double change_vel_dist,
        const MF::PositionVectorD& curr_direc,
        const MF::PositionVectorD& direc_to_wp,
        Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic>& curve);

bool solveOptProblem();

void conflictEval();
void adaptCollisionRobotsInfo();

bool findNextWayPt(const MF::PositionVectorD& direc_to_target);

// base_local_planner::MapGridVisualizer map_viz_; ///< @brief The map grid
/// visualizer for outputting
/// the potential field
/// generated by the cost
/// function

// see constructor body for explanations
// base_local_planner::SimpleTrajectoryGenerator generator_;
// base_local_planner::OscillationCostFunction oscillation_costs_;
// base_local_planner::ObstacleCostFunction obstacle_costs_;
// base_local_planner::MapGridCostFunction path_costs_;
// base_local_planner::MapGridCostFunction goal_costs_;
// base_local_planner::MapGridCostFunction goal_front_costs_;
// base_local_planner::MapGridCostFunction alignment_costs_;

// base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
//
void logTraj();
void logTrajMap();
void logOpt(double, const double*, unsigned, const double*, unsigned);
void logCommand(double lin_cmd,
                double ang_cmd);
void logFeedback(double x_feedback,
                 double y_feedback,
                 double psi_feedback,
                 double target_x_feedback,
                 double target_y_feedback,
                 double target_psi_feedback,
                 double v_feedback,
                 double omega_feedback);

template<class T>
static void evalObjectiveFunc(unsigned, double *result, unsigned n, T x, CODRHAPlanner *context);
template<class T>
static void evalEquationConstraints(unsigned, double *result, unsigned n, T x, CODRHAPlanner *context);
template<class T>
static void evalInequationConstraints(unsigned, double *result, unsigned n, T x, CODRHAPlanner *context);
template<class T>
static void evalObjectiveFuncTermination(unsigned, double *result, unsigned n, T x, CODRHAPlanner *context);
template<class T>
static void evalEquationConstraintsTermination(unsigned, double *result, unsigned n, T x, CODRHAPlanner *context);
template<class T>
static void evalInequationConstraintsTermination(unsigned, double *result, unsigned n, T x, CODRHAPlanner *context);

/**
 * Computes numerical gradient.
 * @param m    function to be diff dimension
 * @param n    dimension of the variable the function will be derivated in
 * @param x    the point around where the gradient will be computed
 * @param grad where to stock the gradient
 * @param eval pointer the function that evaluates the function to be differentiated at a given x
 */
void computeNumGrad(unsigned m,
                    unsigned n,
                    const double* x,
                    double* grad,
                    void (*eval)(unsigned, double*, unsigned, volatile double*, CODRHAPlanner*));
/**
 * Computes numerical gradient.
 * @param m    function to be diff dimension
 * @param n    dimension of the variable the function will be derivated in
 * @param x    the point around where the gradient will be computed
 * @param eps  n*m vector containing the differentiation steps (h) for each dimension of x and the function
 * @param grad where to stock the gradient
 * @param eval pointer the function that evaluates the function to be differentiated at a given x
 */
void computeNumGrad(unsigned m,
                    unsigned n,
                    const double* x,
                    const double* eps,
                    double* grad,
                    void (*eval)(unsigned, double*, unsigned, volatile double*, CODRHAPlanner*));

/**
 * Evaluates the objective function for the optimization problem for all planning phases except the last one
 * @param  n    number of parameters
 * @param  x    vector of parameters
 * @param  grad where to stock the gradient (if grad diff of NULL)
 * @param  data data needed by the optmization
 * @return      the computed cost
 */
static double objectiveFunc(unsigned n,
                            const double *x,
                            double *grad,
                            void *data);
/**
 * Evaluates the equations constraints for the optimization problem for all planning phases except the last one
 * @param m      number of equations constraints
 * @param result vector where to stock the computed values
 * @param n      number of parameters
 * @param x      the parametres
 * @param grad   where to stock the gradient (if grad diff of NULL)
 * @param data   data needed by the optmization
 */
static void equationConstraints(unsigned m,
                                double *result,
                                unsigned n,
                                const double* x,
                                double* grad,
                                void* data);
/**
 * Evaluates the inequations constraints for the optimization problem for all planning phases except the last one
 * @param m      number of inequations constraints
 * @param result vector where to stock the computed values
 * @param n      number of parameters
 * @param x      the parametres
 * @param grad   where to stock the gradient (if grad diff of NULL)
 * @param data   data needed by the optmization
 */
static void inequationConstraints(unsigned m,
                                  double *result,
                                  unsigned n,
                                  const double* x,
                                  double* grad,
                                  void* data);
/**
 * Evaluates the objective function for the optimization problem for the final phase
 * @param  n    number of parameters
 * @param  x    vector of parameters
 * @param  grad where to stock the gradient (if grad diff of NULL)
 * @param  data data needed by the optmization
 * @return      the computed cost
 */
static double objectiveFuncTermination(unsigned n,
                                       const double *x,
                                       double *grad,
                                       void *data);

/**
 * Evaluates the equations constraints for the optimization problem for the final phase
 * @param m      number of equations constraints
 * @param result vector where to stock the computed values
 * @param n      number of parameters
 * @param x      the parametres
 * @param grad   where to stock the gradient (if grad diff of NULL)
 * @param data   data needed by the optmization
 */
static void equationConstraintsTermination(unsigned m,
                                           double *result,
                                           unsigned n,
                                           const double* x,
                                           double* grad,
                                           void* data);
/**
 * Evaluates the inequations constraints for the optimization problem for the final phase
 * @param m      number of inequations constraints
 * @param result vector where to stock the computed values
 * @param n      number of parameters
 * @param x      the parametres
 * @param grad   where to stock the gradient (if grad diff of NULL)
 * @param data   data needed by the optmization
 */
static void inequationConstraintsTermination(unsigned m,
                                             double *result,
                                             unsigned n,
                                             const double* x,
                                             double* grad,
                                             void* data);

/**
 * Interpolation of costmap values using a second order suface and QR
 * decomposition of solving the overdetermined system Ax = b
 *
 * @param  px        world coordinate x of the point
 * @param  py        world coordinate x of the point
 * @param  mx        cell x index
 * @param  my        cell y index
 * @param  costmap   costmap
 * @param  map_max   max value in the map (254 by default)
 * @param  map_front value of cells in the frontier between augmented obstacle and free space
 * @param  radius    radius of the robot
 * @return           the interpolated cost for position px, py
 */
static double interpolateCost(const double& px,
                              const double& py,
                              const unsigned& mx,
                              const unsigned& my,
                              const costmap_2d::Costmap2D& costmap,
                              const unsigned& map_max,
                              const unsigned& map_front,
                              const double& radius);

};

template<class T>
void CODRHAPlanner::evalObjectiveFunc(unsigned m, double *result, unsigned n, T x, CODRHAPlanner *context)
{
        try { boost::this_thread::interruption_point(); }
        catch (boost::thread_interrupted&) { nlopt_force_stop(context->opt_pbl_); }

        #if (OPT_DEBUG_ENABLED)

        if (m != 1)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalObjectiveFunc: unsigned m == 1 = false (m = " << m << " instead).";
                throw(common::MyException(ss.str()));
        }
        if (n != context->opt_trajectory_.nParam())
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalObjectiveFunc: unsigned n == " << context->opt_trajectory_.nParam() << " = false (n = " << n << " instead).";
                throw(common::MyException(ss.str()));
        }

        *result = std::numeric_limits<double>::quiet_NaN();

        #endif

        // Base trajectory
        // double * init_traj = new double[n];
        // context->opt_trajectory_.getParameters(init_traj);
        for (auto i = 0; i < context->opt_trajectory_.nParam(); ++i)
        {
                context->params_[i] = x[i] -1. + context->init_params_[i];
        }

        // Update optimization trajectory with x
        context->opt_trajectory_.update(static_cast<T>(context->params_));
        // context->opt_trajectory_.update(x);

        // Create a Matrix consisting of the flat output and its needed derivatives for the instant Tp (1.0)
        MF::MatForPositionD deriv_flat(
                context->opt_trajectory_(context->opt_plan_time_, MF::derivOrdForPosition));

        // Get pose at Tp from flatoutput
        MF::PositionVectorD position_at_Tp(
                MF::flatToPosition(deriv_flat));

        // position_at_Tp = context->rot_mat_2_wfr_*position_at_Tp + context->latest_position_;

        MF::PositionVectorD way_pt_position(MF::positionInPose(context->way_pt_pose_));

        // way_pt_position = context->rot_mat_2_rfr_*(way_pt_position - context->latest_position_);


        // Compute euclidian distance to square from position at Tp and goal position which will be the cost
        double fx = pow((way_pt_position - position_at_Tp).norm() - context->min_cost_v_, 2);
        // double fx = ((way_pt_position - position_at_Tp).norm() - context->min_cost_v_);
        // double fx = std::abs((way_pt_position - position_at_Tp).norm() - context->min_cost_v_);
        // double fx = pow((way_pt_position - position_at_Tp).norm(), .5);
        // double fx = (way_pt_position - position_at_Tp).norm();

        // if (fx != fx) std::cout << "FOUND QNAN @ evalObjectiveFunc\n";

        *result = fx;
        // *result = 0;
}

template<class T>
void CODRHAPlanner::evalEquationConstraints(unsigned m, double *result, unsigned n, T x, CODRHAPlanner *context)
{
        try { boost::this_thread::interruption_point(); }
        catch (boost::thread_interrupted&) { nlopt_force_stop(context->opt_pbl_); }

        #if (OPT_DEBUG_ENABLED)

        if (m != MF::poseDim + MF::veloDim)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned m == " << MF::poseDim + MF::veloDim << " = false (m = " << m << " instead).";
                throw(common::MyException(ss.str()));
        }
        if (n != context->opt_trajectory_.nParam())
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned n == " << context->opt_trajectory_.nParam() << " = false (n = " << n << " instead).";
                throw(common::MyException(ss.str()));
        }

        for (auto i = 0; i < m; ++i)
        {
                result[i] = std::numeric_limits<double>::quiet_NaN();
        }

        #endif

        for (auto i = 0; i < context->opt_trajectory_.nParam(); ++i)
        {
                context->params_[i] = x[i] -1. + context->init_params_[i];
        }

        // Update optimization trajectory with x
        // context->opt_trajectory_.update(context->rot_mat_2_wfr_*context->opt_trajectory_.cArray2CtrlPtsMat(context->params_) + context->latest_position_.replicate(1, context->opt_trajectory_.nCtrlPts()));
        context->opt_trajectory_.update(static_cast<T>(context->params_));
        // context->opt_trajectory_.update(context->params_);

        //
        MF::MatForVeloD deriv_flat(
                context->opt_trajectory_(0.0, MF::derivOrdForVelo));
        // context->opt_trajectory_(0.0+context->num_dif_eps_, MF::derivOrdForVelo));

        MF::PoseVectorD diff_pose_at_T0(MF::flatToPose(deriv_flat) -
                                        context->latest_pose_);

        MF::yawInPose(diff_pose_at_T0) << common::wrapToPi(MF::yawInPose(diff_pose_at_T0));

        MF::VeloVectorD diff_velocity_at_T0(MF::flatToVelocity(deriv_flat) -
                                            context->latest_velocity_);

        std::vector<double> max_velocity = {context->max_vel_x_, context->max_vel_theta_};

        int i;
        for (i = 0; i < MF::poseDim; ++i)
        {
                result[i] = diff_pose_at_T0[i];
                // if (result[i] != result[i])
                // {
                //         std::cout << "FOUND QNAN @ evalEquationConstraints @ result[" << i << "]\n";
                //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                //         // nlopt_force_stop(context->_opt);
                // }
        }

        for (int j = i; j - i < MF::veloDim; ++j)
        {
                result[j] = diff_velocity_at_T0[j - i]/max_velocity[j-i];
                // result[j] = diff_velocity_at_T0[j - i];
                // result[j] = diff_velocity_at_T0[j - i];
                // if (result[j] != result[j])
                // {
                //         std::cout << "FOUND QNAN @ evalEquationConstraints @ result[" << j << "]\n";
                //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                //         // nlopt_force_stop(context->_opt);
                // }
        }
}

template<class T>
void CODRHAPlanner::evalInequationConstraints(unsigned m, double *result, unsigned n, T x, CODRHAPlanner *context)
{
        try { boost::this_thread::interruption_point(); }
        catch (boost::thread_interrupted&) { nlopt_force_stop(context->opt_pbl_); }

        int n_samples = int(context->opt_plan_time_/context->plan_granularity_);

        #if (OPT_DEBUG_ENABLED)

        unsigned internal_m;

        if (context->respect_obst_const_)
                internal_m = MF::veloDim * n_samples + 1 * n_samples + MF::accelDim * (n_samples + 1) + context->collision_vehicles_.size() * n_samples;
        else
                internal_m = MF::veloDim * n_samples + MF::accelDim * (n_samples + 1) + context->collision_vehicles_.size() * n_samples;

        if (m != internal_m)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalInequationConstraints: unsigned m == "
                   << internal_m << " = false (m = " << m << " instead).";
                throw(common::MyException(ss.str()));
        }
        if (n != context->opt_trajectory_.nParam())
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalInequationConstraints: unsigned n == "
                   << context->opt_trajectory_.nParam()
                   << " = false (n = " << n << " instead).";
                throw(common::MyException(ss.str()));
        }

        for (auto i = 0; i < m; ++i)
        {
                result[i] = std::numeric_limits<double>::quiet_NaN();
        }

        #endif

        for (auto i = 0; i < context->opt_trajectory_.nParam(); ++i)
        {
                context->params_[i] = x[i] -1. + context->init_params_[i];
        }

        MF::PositionVectorD last_position_init_params;
        last_position_init_params << context->init_params_[n-2], context->init_params_[n-1];

        // Update optimization trajectory with x
        // context->opt_trajectory_.update(context->rot_mat_2_wfr_*context->opt_trajectory_.cArray2CtrlPtsMat(context->params_) + context->latest_position_.replicate(1, context->opt_trajectory_.nCtrlPts()));
        context->opt_trajectory_.update(static_cast<T>(context->params_));
        // context->opt_trajectory_.update(context->params_);

        //
        MF::MatForAccelD deriv_flat(context->opt_trajectory_(0.0, MF::derivOrdForAccel));
        // MF::MatForAccelD deriv_flat(context->opt_trajectory_(0.0+context->num_dif_eps_, MF::derivOrdForAccel));

        // Create Matrices for storing velocity and acceleration
        MF::PositionVectorD position;
        MF::PoseVectorD pose;
        MF::VeloVectorD velocity;
        MF::AccelVectorD acceleration;

        // ACCELERATION AT 0.0
        acceleration = MF::flatToAcceleration(deriv_flat);

        std::vector<double> max_acceleration = {context->max_acc_x_, context->max_acc_theta_};
        std::vector<double> max_velocity = {context->max_vel_x_, context->max_vel_theta_};

        unsigned i, j, k, l, o;
        for (i = 0; i < MF::accelDim; ++i)
        {
                result[i*(n_samples+1)] =
                        (acceleration(i, 0) * acceleration(i, 0) -
                         max_acceleration[i] * max_acceleration[i])/
                        (max_acceleration[i] * max_acceleration[i]);
                // result[i] = result[i] != result[i] ? 0.0 : result[i];
                // if (result[i] != result[i])
                // {
                //         std::cout << "FOUND QNAN @ evalIneq @ result[" << i << "]\n";
                //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                //         // nlopt_force_stop(context->_opt);
                // }
                // std::cout << "result[" << i << "] = " << result[i] << std::endl;
        }

        // unsigned nAcc = i;
        // unsigned ineq_per_samples =
        //         (MF::veloDim + MF::accelDim +
        //          1 +
        //          context->collision_vehicles_.size() +
        //          context->comm_outrage_vehicles_.size());

        //#pragma omp parallel for <== DO NOT USE IT, BREAKS THE EVALUATION OF
        //CONSTRAINTS SOMEHOW


        for (i = 1; i <= n_samples; ++i)
        {
                double eval_time = double(i) / n_samples * context->opt_plan_time_;
                //    eval_time = eval_time == context->opt_plan_time_ ? eval_time - 0.0 : eval_time;

                deriv_flat = context->opt_trajectory_(eval_time, MF::derivOrdForAccel);

                position = MF::flatToPosition(deriv_flat);
                pose = MF::flatToPose(deriv_flat);
                velocity = MF::flatToVelocity(deriv_flat);
                acceleration = MF::flatToAcceleration(deriv_flat);

                for (j = 0; j < MF::accelDim; ++j)
                {
                        #if (OPT_DEBUG_ENABLED)
                        if (m <= j*(n_samples+1) + i)
                        {
                                std::stringstream ss;
                                ss << "CODRHAPlanner::evalInequationConstraints: result index out of range (> " << m-1 << ").";
                                throw(common::MyException(ss.str()));
                        }
                        #endif
                        // result[nAcc + j + (i - 1) * ineq_per_samples] =
                        result[j*(n_samples+1) + i] =
                                (acceleration(j, 0) * acceleration(j, 0) -
                                 max_acceleration[j] * max_acceleration[j])/
                                (max_acceleration[j] * max_acceleration[j]);
                        // result[nAcc+j+(i-1)*ineq_per_samples] = result[nAcc+j+(i-1)*ineq_per_samples]
                        // != result[nAcc+j+(i-1)*ineq_per_samples] ? 0.0 :
                        // result[nAcc+j+(i-1)*ineq_per_samples];
                        // if (result[nAcc + j + (i - 1) * ineq_per_samples] !=
                        //                 result[nAcc + j + (i - 1) * ineq_per_samples])
                        // {
                        //         std::cout << "FOUND QNAN @ evalIneq @ result["
                        //                 << nAcc + j + (i - 1) * ineq_per_samples << "]\n";
                        //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                        //         // nlopt_force_stop(context->_opt);
                        // }
                        // std::cout << "result[" << nAcc+j+(i-1)*ineq_per_samples << "] = " <<
                        // result[nAcc+j+(i-1)*ineq_per_samples] << std::endl;
                }
                // j = 0;
                // result[j*(n_samples+2) + (i-1)] =
                //         (velocity(0, 0) * velocity(0, 0) -
                //         max_velocity[0] * max_velocity[0]);
                // result[j*(n_samples+2) + n_samples + (i-1)] =
                //         (velocity(1, 0) * velocity(1, 0) -
                //         max_velocity[1] * max_velocity[1])/
                //         (max_velocity[1] * max_velocity[1]);
                // k=2;
                for (k = 0; k < MF::veloDim; ++k)
                {
                        #if (OPT_DEBUG_ENABLED)
                        if (m <= j*(n_samples+1) + k*n_samples + (i-1))
                        {
                                std::stringstream ss;
                                ss << "CODRHAPlanner::evalInequationConstraints: result index out of range (> " << m-1 << ").";
                                throw(common::MyException(ss.str()));
                        }
                        #endif
                        // result[nAcc + k + (i - 1) * ineq_per_samples] =
                        result[j*(n_samples+1) + k*n_samples + (i-1)] =
                                (velocity(k, 0) * velocity(k, 0) -
                                 // max_velocity[k] * max_velocity[k]);
                                 max_velocity[k] * max_velocity[k])/
                                (max_velocity[k] * max_velocity[k]);
                }

                // MapObst::iterator obsIt;
                // for (obsIt = context->detected_obstacles_.begin(), j = k;
                // j - k < 1; ++j, ++obsIt) {
                l = 0;
                if (context->respect_obst_const_)
                {
                        unsigned mx, my;
                        double px, py;
                        // MF::PositionVectorD position_wrt_cmap;
                        // position_wrt_cmap = context->rot_mat_2_cmapfr_*(position - MF::positionInPose(context->costmap_ref_));
                        // px = position_wrt_cmap.x();
                        // py = position_wrt_cmap.y();

                        // px = position.x();
                        // py = position.y();

                        // worldToMap is waiting for world points in the odom frame! Transfromation from odom0 to odom needed

                        context->comm_link_->transform2DPointFromPlanRef2Odom(position.x(), position.y(), context->thread_world_time_sec_, context->thread_world_time_nsec_, px, py);

                        // comm_link_->transform2DPointFromPlanRef2Odom(position(0), position(1), robot_info->plan_t0.sec, robot_info->plan_t0.nsec, world_time_sec_, world_time_nsec_, x, y);
                        // transform2DPointFromPlanRef2Odom()
                        double obst_cost;
                        if(!context->thread_costmap_.worldToMap(px, py, mx, my))
                        {
                                obst_cost = -context->radius_ / (context->costmap_2d_INSCRIBED_INFLATED_OBSTACLE_ - context->cell_value_at_rad_dist_) * context->cell_value_at_rad_dist_;
                        }
                        else
                        {
                                obst_cost = interpolateCost(px, py, mx, my, context->thread_costmap_, context->costmap_2d_INSCRIBED_INFLATED_OBSTACLE_, context->cell_value_at_rad_dist_, context->radius_);
                                // obst_cost = costs[4]; // DUMMY SOLUTION THAT DOES NOT WORK
                        }

                        #if (OPT_DEBUG_ENABLED)
                        if (m <= j*(n_samples+1) + (k+l)*n_samples + (i-1))
                        {
                                std::stringstream ss;
                                ss << "CODRHAPlanner::evalInequationConstraints: result index out of range (> " << m-1 << ").";
                                throw(common::MyException(ss.str()));
                        }
                        #endif

                        result[j*(n_samples+1) + (k+l)*n_samples + (i-1)] = obst_cost;

                        ++l;
                }
                // }
                // if (context->respect_multirobot_const_)
                o = 0;
                // {

                if (!context->collision_vehicles_.empty())
                {
                        double absolute_eval_time = context->time_of_exec_ + eval_time;
                        for(auto robot_id : context->collision_vehicles_)
                        {
                                double dist;
                                if (context->others_.find(robot_id) != context->others_.end())
                                {
                                        // unsigned splDeg = context->others_[robot_id].splDeg;
                                        // FIXME TODO think about generate Trajectory without knowing the degree
                                        Trajectory< double, MF::flatDim, MF::derivOrdForAccel + 1 > othTraj_map(context->others_[robot_id].nCtrlPts/2);
                                        othTraj_map.update(context->others_[robot_id].ctrlPts, context->others_[robot_id].pvar);

                                        double time_from_stamp = context->others_[robot_id].stamp.sec + context->others_[robot_id].stamp.nsec*1e-9;
                                        double relative_eval_time_for_other_robot = absolute_eval_time - (time_from_stamp + context->others_[robot_id].time_to_exec);

                                        MF::PositionVectorD oth_position;
                                        oth_position = MF::flatToPosition(othTraj_map(relative_eval_time_for_other_robot, MF::derivOrdForPosition));

                                        double my_x, my_y;
                                        context->comm_link_->transform2DPointFromPlanRef2Map(position.x(), position.y(), my_x, my_y);

                                        double rad1 = context->radius_;
                                        double rad2 = context->others_[robot_id].radius;
                                        // std::cout << FG_L_RED << "RADS: " << rad1 << ", " << rad2 << std::endl;
                                        // double max_dist_room = 10.; // FIXME
                                        MF::PositionVectorD position_map;
                                        position_map << my_x, my_y;
                                        dist = (position_map - oth_position).norm() - rad1 - rad2;
                                        // if(dist > 0)
                                        // {
                                        //         dist = dist*dist;
                                        // }
                                        // else
                                        // {
                                        //         dist = -dist*dist;
                                        // }

                                        #if (OPT_DEBUG_ENABLED)
                                        if (m <= j*(n_samples+1) + (k+l+o)*n_samples + (i-1))
                                        {
                                                std::stringstream ss;
                                                ss << "CODRHAPlanner::evalInequationConstraints: result index out of range (> " << m-1 << ").";
                                                throw(common::MyException(ss.str()));
                                        }
                                        #endif

                                        // if (dist < 0.)
                                        //         std::cout << FG_GREEN << "DIST < 0" << RESET << std::endl;
                                }
                                else // FIXME
                                {
                                        double rad1 = context->radius_;
                                        // double max_dist_room = 10.; // FIXME
                                        dist = context->comm_link_->getPlanPtDist(position.x(), position.y(), context->thread_world_time_sec_, context->thread_world_time_nsec_, robot_id) - 2.*rad1;
                                        // if(dist > 0)
                                        // {
                                        //         dist = dist*dist;
                                        // }
                                        // else
                                        // {
                                        //         dist = -dist*dist;
                                        // }
                                }
                                result[j*(n_samples+1) + (k+l+o)*n_samples + (i-1)] = -dist;
                                ++o;
                        }
                }
        }
}

template<class T>
void CODRHAPlanner::evalObjectiveFuncTermination(unsigned m, double *result, unsigned n, T x, CODRHAPlanner *context)
{
        try { boost::this_thread::interruption_point(); }
        catch (boost::thread_interrupted&) { nlopt_force_stop(context->opt_pbl_); }

        #if (OPT_DEBUG_ENABLED)

        if (m != 1)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned m == " << 1 << " = false (m = " << m << " instead).";
                throw(common::MyException(ss.str()));
        }
        if (n != context->opt_trajectory_.nParam()+1)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned n == " << context->opt_trajectory_.nParam()+1 << " = false (n = " << n << " instead).";
                throw(common::MyException(ss.str()));
        }

        *result = std::numeric_limits<double>::quiet_NaN();

        #endif

        double horizon = x[0] -1. + context->init_params_[0];

        if (horizon < 0)
        {
                *result = std::numeric_limits<double>::max();
        }
        else
        {
                *result = horizon*horizon;
        }
}

template<class T>
void CODRHAPlanner::evalEquationConstraintsTermination(unsigned m, double *result, unsigned n, T x, CODRHAPlanner *context)
{
        try { boost::this_thread::interruption_point(); }
        catch (boost::thread_interrupted&) { nlopt_force_stop(context->opt_pbl_); }

        #if (OPT_DEBUG_ENABLED)

        if (m != (MF::poseDim + MF::veloDim) * 2)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned m == " << (MF::poseDim + MF::veloDim) * 2 << " = false (m = " << m << " instead).";
                throw(common::MyException(ss.str()));
        }
        if (n != context->opt_trajectory_.nParam()+1)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned n == " << context->opt_trajectory_.nParam()+1 << " = false (n = " << n << " instead).";
                throw(common::MyException(ss.str()));
        }

        for (auto i = 0; i < m; ++i)
        {
                result[i] = std::numeric_limits<double>::quiet_NaN();
        }

        #endif

        for (auto i = 0; i < n; ++i)
        {
                context->params_[i] = x[i] -1. + context->init_params_[i];
        }
        // Update optimization trajectory with x
        // context->opt_trajectory_.update(context->rot_mat_2_wfr_*context->opt_trajectory_.cArray2CtrlPtsMat(&(x[1])) + context->latest_position_.replicate(1, context->opt_trajectory_.nCtrlPts()), x[0]);
        context->opt_trajectory_.update(static_cast<T>(&(context->params_[1])), context->params_[0]);
        // context->opt_trajectory_.update(&(x[1]), x[0]);

        //
        MF::MatForVeloD deriv_flat(
                context->opt_trajectory_(0.0, MF::derivOrdForVelo));
        // context->opt_trajectory_(0.0+context->num_dif_eps_, MF::derivOrdForVelo));

        // Create a Matrix consisting of the flat output and its needed derivatives
        // for the instant T0 (0.0)
        // MF::MatForVeloD deriv_flat(context->opt_trajectory_(0.0, MF::derivOrdForVelo));
        // MF::MatForVeloD deriv_flat(context->opt_trajectory_(0.0+context->num_dif_eps_, MF::derivOrdForVelo));

        MF::PoseVectorD diff_pose(MF::flatToPose(deriv_flat) -
                                  context->latest_pose_);

        MF::yawInPose(diff_pose) << common::wrapToPi(MF::yawInPose(diff_pose));

        MF::VeloVectorD diff_velocity = MF::flatToVelocity(deriv_flat) -
                                        context->latest_velocity_;

        std::vector<double> max_velocity = {context->max_vel_x_, context->max_vel_theta_};

        int i, j;
        for (i = 0; i < MF::poseDim; ++i)
        {
                result[i] = diff_pose[i];
        }
        for (j = i; j - i < MF::veloDim; ++j)
        {
                // result[j] = diff_velocity[j - i];
                result[j] = diff_velocity[j - i]/max_velocity[j-i];
        }

        deriv_flat =
                context->opt_trajectory_(context->params_[0], MF::derivOrdForVelo);

        diff_pose = MF::flatToPose(deriv_flat) - context->way_pt_pose_;

        MF::yawInPose(diff_pose) << common::wrapToPi(MF::yawInPose(diff_pose));

        diff_velocity = MF::flatToVelocity(deriv_flat) -
                        context->way_pt_velocity_;

        for (i = j; i - j < MF::poseDim; ++i)
        {
                result[i] = diff_pose[i - j];
        }
        for (j = i; j - i < MF::veloDim; ++j)
        {
                // result[j] = diff_velocity[j - i];
                result[j] = diff_velocity[j - i]/max_velocity[j-i];
        }
}

template<class T>
void CODRHAPlanner::evalInequationConstraintsTermination(unsigned m, double *result, unsigned n, T x, CODRHAPlanner *context)
{
        try { boost::this_thread::interruption_point(); }
        catch (boost::thread_interrupted&) { nlopt_force_stop(context->opt_pbl_); }

        int n_samples = int(context->init_params_[0]/context->plan_granularity_);

        #if (OPT_DEBUG_ENABLED)

        unsigned internal_m;
        if (context->respect_obst_const_term_)
                internal_m = MF::veloDim * (n_samples-1) + MF::accelDim * (n_samples+1) + 1 * (n_samples-1);
        else
                internal_m = MF::veloDim * (n_samples-1) + MF::accelDim * (n_samples+1);

        if (m != internal_m)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned m == "
                   << internal_m << " = false (m = " << m << " instead).";
                throw(common::MyException(ss.str()));
        }

        if (n != context->opt_trajectory_.nParam()+1)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::evalEquationConstraints: unsigned n == "
                   << context->opt_trajectory_.nParam()+1
                   << " = false (n = " << n << " instead).";
                throw(common::MyException(ss.str()));
        }

        for (auto i = 0; i < m; ++i)
        {
                result[i] = std::numeric_limits<double>::quiet_NaN();
        }

        #endif

        // std::cout << "::evalIneqLS call" << std::endl;
        // Update optimization trajectory with x
        for (auto i = 0; i < n; ++i)
        {
                context->params_[i] = x[i] -1. + context->init_params_[i];
        }
        // Update optimization trajectory with x
        // context->opt_trajectory_.update(context->rot_mat_2_wfr_*context->opt_trajectory_.cArray2CtrlPtsMat(&(x[1])) + context->latest_position_.replicate(1, context->opt_trajectory_.nCtrlPts()), x[0]);
        context->opt_trajectory_.update(static_cast<T>(&(context->params_[1])), context->params_[0]);

        // INEQUATIONS
        MF::MatForAccelD deriv_flat(context->opt_trajectory_(0.0, MF::derivOrdForAccel));

        // Create Matrices for storing velocity and acceleration
        MF::VeloVectorD velocity;
        MF::AccelVectorD acceleration;
        MF::PoseVectorD pose;
        MF::PositionVectorD position;

        // Superior and inferior limits for optimal time
        // x[0] > context->opt_plan_time_ => context->opt_plan_time_-x[0] < 0
        // x[0] < 2*opt_plan_time_ => x[0] - 2*opt_plan_time_ < 0
        // result[0] = context->opt_plan_time_ - x[0]; // time has to be positive
        // result[1] = x[0] - 5. * context->opt_plan_time_;
        // TODO no magic number

        // ACCELERATION AT 0.0
        acceleration = MF::flatToAcceleration(deriv_flat);

        std::vector<double> max_acceleration = {context->max_acc_x_, context->max_acc_theta_};
        std::vector<double> max_velocity = {context->max_vel_x_, context->max_vel_theta_};

        unsigned i, j, k, l, o;
        for (i = 0; i < MF::accelDim; ++i)
        {
                result[i*(n_samples+1)] =
                        (acceleration(i, 0) * acceleration(i, 0) -
                         max_acceleration[i] * max_acceleration[i])/
                        (max_acceleration[i] * max_acceleration[i]);
                // result[i] = result[i] != result[i] ? 0.0 : result[i];
                // if (result[i] != result[i])
                // {
                //         std::cout << "FOUND QNAN @ evalIneq @ result[" << i << "]\n";
                //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                //         // nlopt_force_stop(context->_opt);
                // }
                // std::cout << "result[" << i << "] = " << result[i] << std::endl;
        }
        // ACCELERATION AT Tf
        deriv_flat = context->opt_trajectory_(x[0], MF::derivOrdForAccel);

        acceleration = MF::flatToAcceleration(deriv_flat);

        for (i = 0; i < MF::accelDim; ++i)
        {
                result[i*(n_samples+1) + n_samples] =
                        (acceleration(i, 0) * acceleration(i, 0) -
                         max_acceleration[i] * max_acceleration[i])/
                        (max_acceleration[i] * max_acceleration[i]);
                // result[i] = result[i] != result[i] ? 0.0 : result[i];
                // if (result[i] != result[i])
                // {
                //         std::cout << "FOUND QNAN @ evalIneq @ result[" << i << "]\n";
                //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                //         // nlopt_force_stop(context->_opt);
                // }
                // std::cout << "result[" << i << "] = " << result[i] << std::endl;
        }

        // unsigned nAcc = j;
        //
        // unsigned ineq_per_samples =
        //         (MF::veloDim + MF::accelDim +
        //          1 +
        //          context->collision_vehicles_.size() +
        //          context->comm_outrage_vehicles_.size());


        for (i = 1; i < n_samples; ++i)
        {
                double eval_time = double(i) / n_samples * context->params_[0];
                //    eval_time = eval_time == context->init_params_[0] ? eval_time - 0.0 : eval_time;

                deriv_flat = context->opt_trajectory_(eval_time, MF::derivOrdForAccel);

                position = MF::flatToPosition(deriv_flat);
                pose = MF::flatToPose(deriv_flat);
                velocity = MF::flatToVelocity(deriv_flat);
                acceleration = MF::flatToAcceleration(deriv_flat);

                for (j = 0; j < MF::accelDim; ++j)
                {
                        #if (OPT_DEBUG_ENABLED)
                        if (m <= j*(n_samples+1) + i)
                        {
                                std::stringstream ss;
                                ss << "CODRHAPlanner::evalEquationConstraints: result index out of range (> " << m-1 << ").";
                                throw(common::MyException(ss.str()));
                        }
                        #endif
                        // result[nAcc + j + (i - 1) * ineq_per_samples] =
                        result[j*(n_samples-1+2) + i] =
                                (acceleration(j, 0) * acceleration(j, 0) -
                                 max_acceleration[j] * max_acceleration[j])/
                                (max_acceleration[j] * max_acceleration[j]);
                        // result[nAcc+j+(i-1)*ineq_per_samples] = result[nAcc+j+(i-1)*ineq_per_samples]
                        // != result[nAcc+j+(i-1)*ineq_per_samples] ? 0.0 :
                        // result[nAcc+j+(i-1)*ineq_per_samples];
                        // if (result[nAcc + j + (i - 1) * ineq_per_samples] !=
                        //                 result[nAcc + j + (i - 1) * ineq_per_samples])
                        // {
                        //         std::cout << "FOUND QNAN @ evalIneq @ result["
                        //                 << nAcc + j + (i - 1) * ineq_per_samples << "]\n";
                        //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                        //         // nlopt_force_stop(context->_opt);
                        // }
                        // std::cout << "result[" << nAcc+j+(i-1)*ineq_per_samples << "] = " <<
                        // result[nAcc+j+(i-1)*ineq_per_samples] << std::endl;
                }
                // j = 0;

                // result[j*(n_samples+2) + (i-1)] =
                //         (velocity(0, 0) * velocity(0, 0) -
                //         max_velocity[0] * max_velocity[0]);
                // result[j*(n_samples+2) + n_samples + (i-1)] =
                //         (velocity(1, 0) * velocity(1, 0) -
                //         max_velocity[1] * max_velocity[1])/
                //         (max_velocity[1] * max_velocity[1]);
                // k=2;
                for (k = 0; k < MF::veloDim; ++k)
                {
                        #if (OPT_DEBUG_ENABLED)
                        if (m <= j*(n_samples-1+2) + k*(n_samples-1) + (i-1))
                        {
                                std::stringstream ss;
                                ss << "CODRHAPlanner::evalEquationConstraints: result index out of range (> " << m-1 << ").";
                                throw(common::MyException(ss.str()));
                        }
                        #endif
                        // result[nAcc + k + (i - 1) * ineq_per_samples] =
                        result[j*(n_samples-1+2) + k*(n_samples-1) + (i-1)] =
                                (velocity(k, 0) * velocity(k, 0) -
                                 // max_velocity[k] * max_velocity[k]);
                                 max_velocity[k] * max_velocity[k])/
                                (max_velocity[k] * max_velocity[k]);
                        //  max_velocity[k] * max_velocity[k])/
                        // (max_velocity[k] * max_velocity[k]);
                        // result[nAcc+k+(i-1)*ineq_per_samples] = result[nAcc+k+(i-1)*ineq_per_samples]
                        // != result[nAcc+k+(i-1)*ineq_per_samples] ? 0.0 :
                        // result[nAcc+k+(i-1)*ineq_per_samples];
                        // if (result[nAcc + k + (i - 1) * ineq_per_samples] !=
                        //                 result[nAcc + k + (i - 1) * ineq_per_samples])
                        // {
                        //         std::cout << "FOUND QNAN @ evalIneq @ result["
                        //                 << nAcc + k + (i - 1) * ineq_per_samples << "]\n";
                        //                 // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                        //                 // nlopt_force_stop(context->_opt);
                        // }
                        // std::cout << "result[" << nAcc+k+(i-1)*ineq_per_samples << "] = " <<
                        // result[nAcc+k+(i-1)*ineq_per_samples] << std::endl;
                }

                // MapObst::iterator obsIt;
                // for (obsIt = context->detected_obstacles_.begin(), j = k;
                // j - k < 1; ++j, ++obsIt) {
                l = 0;

                if (context->respect_obst_const_term_)
                {
                        unsigned mx, my;
                        double px, py;
                        // MF::PositionVectorD position_wrt_cmap;
                        // position_wrt_cmap = context->rot_mat_2_cmapfr_*(position - MF::positionInPose(context->costmap_ref_));
                        px = position.x();
                        py = position.y();

                        double obst_cost;
                        if(!context->thread_costmap_.worldToMap(px, py, mx, my))
                        {
                                obst_cost = -context->radius_ / (context->costmap_2d_INSCRIBED_INFLATED_OBSTACLE_ - context->cell_value_at_rad_dist_) * context->cell_value_at_rad_dist_;
                        }
                        else
                        {
                                obst_cost = interpolateCost(px, py, mx, my, context->thread_costmap_, context->costmap_2d_INSCRIBED_INFLATED_OBSTACLE_, context->cell_value_at_rad_dist_, context->radius_);
                        }

                        #if (OPT_DEBUG_ENABLED)
                        if (m <= j*(n_samples-1+2) + (k+l)*(n_samples-1) + (i-1))
                        {
                                std::stringstream ss;
                                ss << "CODRHAPlanner::evalEquationConstraints: result index out of range (> " << m-1 << ").";
                                throw(common::MyException(ss.str()));
                        }
                        #endif

                        result[j*(n_samples-1+2) + (k+l)*(n_samples-1) + (i-1)] = obst_cost;

                        ++l;
                }
                // }

                // MapAIV::iterator aivIt;
                // for (aivIt = context->collision_vehicles_.begin(), k = j;
                // k - j < context->collision_vehicles_.size(); ++k, ++aivIt) {
                // m = 0;
                // for (auto vehicle_name : context->collision_vehicles_)
                // {
                //
                //         // std::cout << "Insinde conllision\n";
                //         double distanceInterRobots =
                //                 (context->rot_mat_2_wfr_ * position +
                //                  context->latest_position_ -
                //                  context->path_from_conflictual_vehicles_[vehicle_name].col(i - 1)).norm();
                //
                //         // std::cout << "distanceInterRobots " << distanceInterRobots <<
                //         // std::endl;
                //
                //         double radii =
                //                 context->radius_from_conflictual_vehicles_[vehicle_name] + context->radius_;
                //
                //         double dSecurity =
                //                 std::max(context->inter_rob_saf_dist_,
                //                     context->inter_rob_saf_from_conflictual_vehicles_[vehicle_name]);
                //
                //         // result[nAcc + k + (i - 1) * ineq_per_samples] =
                //         result[j*(n_samples+1) + (k+l+m)*n_samples + (i-1)] =
                //                 (-distanceInterRobots + radii + dSecurity);
                //         // std::cout << "result[" << nAcc+k+(i-1)*ineq_per_samples << "] = " <<
                //         // result[nAcc+k+(i-1)*ineq_per_samples] << std::endl;
                //         // result[nAcc+k+(i-1)*ineq_per_samples] = result[nAcc+k+(i-1)*ineq_per_samples]
                //         // != result[nAcc+k+(i-1)*ineq_per_samples] ? 0.0 :
                //         // result[nAcc+k+(i-1)*ineq_per_samples];
                //         // if (result[nAcc + k + (i - 1) * ineq_per_samples] !=
                //         //                 result[nAcc + k + (i - 1) * ineq_per_samples])
                //         // {
                //         //         std::cout << "FOUND QNAN @ evalIneq @ result["
                //         //                 << nAcc + k + (i - 1) * ineq_per_samples << "]\n";
                //         //         // std::cout << context->opt_trajectory_.getCtrlPts() << std::endl;
                //         //         // nlopt_force_stop(context->_opt);
                //         // }
                //         // std::cout << "result[" << nAcc+k+(i-1)*ineq_per_samples << "] = " <<
                //         // result[nAcc+k+(i-1)*ineq_per_samples] << std::endl;
                //         // - distanceInterRobots + ;
                //         // std::cout << "result[nAcc+k+(i-1)*ineq_per_samples] " <<
                //         // result[nAcc+k+(i-1)*ineq_per_samples] << std::endl;
                //         ++m;
                // }

                // for (aivIt = context->comm_outrage_vehicles_.begin(), j = k; j - k <
                // context->comm_outrage_vehicles_.size(); ++j, ++aivIt)
                // {
                // }
        }
}

};

#endif
