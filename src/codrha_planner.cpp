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

#include <cmath>
#include <codrha_local_planner/codrha_planner.h>
#include <boost/current_function.hpp>
#include <angles/angles.h>

#define bthread boost::thread

#define DEBUG_ENABLED (1)
#define LOG_ENABLED (1)

namespace codrha_local_planner
{

// Declarations of static const int defined in MonocycleFlatoutput

const unsigned MonocycleFlatoutput::flatDim;                // 2D curve
const unsigned MonocycleFlatoutput::poseDim;                // unicycle configuration dimension
const unsigned MonocycleFlatoutput::positionDim;            // position in the 2D plane dimension
const unsigned MonocycleFlatoutput::oriDim;                 // position in the 2D plane dimension
const unsigned MonocycleFlatoutput::veloDim;                // velocity dimension in 2D plane
const unsigned MonocycleFlatoutput::accelDim;               // acceleration dimension in 2D plane
const unsigned MonocycleFlatoutput::derivOrdForPosition;
const unsigned MonocycleFlatoutput::derivOrdForPose;
const unsigned MonocycleFlatoutput::derivOrdForVelo;
const unsigned MonocycleFlatoutput::derivOrdForAccel;
const unsigned MonocycleFlatoutput::linSpeedIdx;
const unsigned MonocycleFlatoutput::angSpeedIdx;
const unsigned MonocycleFlatoutput::linAccelIdx;
const unsigned MonocycleFlatoutput::angAccelIdx;
const unsigned MonocycleFlatoutput::positionIdx;
const unsigned MonocycleFlatoutput::oriIdx;

CODRHAPlanner::CODRHAPlanner(std::string name, costmap_2d::Costmap2D* costmap_ros, CODRHAComm* comm_link, const long unsigned sec, const long unsigned nsec) :
        name_(name),
        costmap_(costmap_ros),
        idx_traj_being_computed_(-1),
        idx_traj_being_executed_(-1),
        time_first_call_of_comp_vel_cmds_(-1.),
        time_since_first_call_(-1.),
        low_level_ctrl_(boost::shared_ptr<Controller>(new Controller(name_+"_controller", comm_link))),
        eval_time_(0.0),
        plan_time_(-1.),
        opt_plan_time_(-1.),
        comp_time_(-1.),
        plan_granularity_(-1.),
        traj_spl_n_ctrl_pts_(-1),
        opt_method_("NONE"),
        num_dif_eps_(0.0),
        last_traj_flag_(false),
        min_cost_v_(0.0),
        first_guess_tweak_(0.0),
        cost_tweak_(0.0),
        max_vel_init_factor_(1.0),
        comm_link_(comm_link)
{
        latest_position_.setZero();
        latest_velocity_.setZero();
        latest_pose_.setZero();

        target_pt_pose_.setZero();
        target_pt_velocity_.setZero();

        way_pt_pose_.setZero();
        way_pt_velocity_.setZero();

        collision_vehicles_.clear();
        comm_outrage_vehicles_.clear();

        #if (LOG_ENABLED)

        traj_log_.open("/home/turtlebot/dev/catkin_ws/src/codrha_local_planner/log/traj_log_" + name + ".yaml");
        traj_log_map_.open("/home/turtlebot/dev/catkin_ws/src/codrha_local_planner/log/traj_log_map_" + name + ".yaml");
        opt_log_.open("/home/turtlebot/dev/catkin_ws/src/codrha_local_planner/log/opt_log_" + name + ".yaml");
        feedback_log_.open("/home/turtlebot/dev/catkin_ws/src/codrha_local_planner/log/fb_log_" + name + ".yaml");
        command_log_.open("/home/turtlebot/dev/catkin_ws/src/codrha_local_planner/log/cmmd_log_" + name + ".yaml");

        #endif
}

CODRHAPlanner::~CODRHAPlanner()
{
        #if (LOG_ENABLED)

        traj_log_.close();
        traj_log_map_.close();
        opt_log_.close();
        feedback_log_.close();
        command_log_.close();

        #endif

}

void CODRHAPlanner::reconfigure(
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
        bool cfg_respect_accel_const, bool cfg_respect_accel_const_term, const std::unordered_set<std::string>& comm_list)
{
        std::string func_name(BOOST_CURRENT_FUNCTION);
        std::string delimiter = "CODRHAPlanner::";
        func_name = func_name.substr(func_name.find(delimiter)+delimiter.length(), func_name.length());
        boost::mutex::scoped_lock l(configuration_mutex_);

        plan_time_ = cfg_plan_time;
        opt_plan_time_ = plan_time_;
        plan_granularity_ = cfg_plan_granularity;
        comp_time_ = cfg_comp_time;
        traj_spl_n_ctrl_pts_ = cfg_traj_spl_n_ctrl_pts;
        opt_method_ = cfg_opt_method;
        num_dif_eps_ = cfg_num_dif_eps;
        opt_objective_func_abs_tol_ = cfg_opt_objective_func_abs_tol;
        opt_objective_func_rel_tol_ = cfg_opt_objective_func_rel_tol;
        opt_param_abs_tol_ = cfg_opt_param_abs_tol;
        opt_param_rel_tol_ = cfg_opt_param_rel_tol;
        opt_equetions_abs_tol_ = cfg_opt_equetions_abs_tol;
        opt_inequetions_abs_tol_ = cfg_opt_inequetions_abs_tol;
        max_vel_x_ = cfg_max_vel_x;
        max_vel_theta_ = cfg_max_vel_theta;
        max_acc_x_ = cfg_acc_lim_x;
        max_acc_theta_ = cfg_acc_lim_theta;
        timeout_for_computing_first_plan_ = cfg_timeout_for_computing_first_plan;
        robot_obst_safety_dist_ = cfg_robot_obst_safety_dist;
        radius_ = cfg_radius;
        last_step_min_dist_ = cfg_last_step_min_dist;
        first_guess_tweak_ = cfg_first_guess_tweak;
        cost_tweak_ = cfg_cost_tweak;
        max_vel_init_factor_ = cfg_max_vel_init_factor;

        acc_sup_x_init_ = cfg_acc_sup_x_init;
        acc_inf_x_init_ = cfg_acc_inf_x_init;

        respect_obst_const_ = cfg_respect_obst_const;
        respect_obst_const_term_ = cfg_respect_obst_const_term;
        respect_accel_const_ = cfg_respect_accel_const;
        respect_accel_const_term_ = cfg_respect_accel_const_term;
        comm_list_ = comm_list;


        low_level_ctrl_->reconfigure(cfg_ctrl_prediction_time, cfg_max_vel_x, cfg_max_vel_theta,
                                     cfg_ctrl_integ_step, cfg_dy_mod_params);

        curve_.resize(MF::flatDim, traj_spl_n_ctrl_pts_);

        opt_trajectory_.interpolate(curve_, opt_plan_time_);
        trajectory_.interpolate(curve_, opt_plan_time_);
}

void CODRHAPlanner::getCostmapInfo()
{
        // unsigned mapXDim = thread_costmap_.getSizeInCellsX();
        // unsigned mapYDim = thread_costmap_.getSizeInCellsY();
        // unsigned char * charMap = thread_costmap_.getCharMap();
        //
        // int left = mapXDim;
        // int right = -1;
        // int up = mapYDim;
        // int down = -1;
        //
        // bool flag = true;
        //
        // Eigen::MatrixXd matMat(mapYDim, mapXDim);
        // for(auto j = 0; j < mapXDim; ++j)
        // {
        //         for(auto i = 0; i < mapYDim; ++i)
        //         {
        //                 matMat(i, j) = double(unsigned(charMap[i * mapXDim + j]));
        //                 if (flag && matMat(i, j) == 254)
        //                 {
        //                         flag = false;
        //                         poi = i;
        //                         posy =  j;
        //                         // std::cout << "matMat(" << i << ", " << j << ") = " << matMat(i, j) << std::endl;
        //                         if (left > j) left = int(j);
        //                         if (right < j) right = int(j);
        //                         if (up > i) up = int(i);
        //                         if (down < i) down = int(i);
        //                 }
        //         }
        // }
        costmap_2d_INSCRIBED_INFLATED_OBSTACLE_ = 254.;
        const double cost_scaling_factor = 1.0;
        cell_value_at_rad_dist_ = (costmap_2d_INSCRIBED_INFLATED_OBSTACLE_-1.)*exp(-1.*cost_scaling_factor*(radius_ - 0.0)); // for 0.2 => 207.1388805287294
}


bool CODRHAPlanner::computeVelocityCommands(
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
        double& ang_vel_output)
{
        world_time_ = world_time_sec + 1e-9*world_time_nsec;
        world_time_sec_ = world_time_sec;
        world_time_nsec_ = world_time_nsec;

        #if (LOG_ENABLED)

        logFeedback(x_feedback,
                    y_feedback,
                    psi_feedback,
                    target_x_feedback,
                    target_y_feedback,
                    target_psi_feedback,
                    v_feedback,
                    omega_feedback);

        #endif

        #if (DEBUG_ENABLED)

        std::string func_name(BOOST_CURRENT_FUNCTION);
        std::string delimiter = "CODRHAPlanner::";
        func_name = func_name.substr(func_name.find(delimiter)+delimiter.length(), func_name.length());
        delimiter = "(";
        func_name = func_name.substr(0, func_name.find(delimiter));

        #endif

        // ---------- update timers

        if (idx_traj_being_computed_ == -1)
        {
                double w, v;
                if (std::abs(omega_feedback) < .01) // FIXME
                        w = 0.0;
                else
                        w = omega_feedback;
                if (std::abs(v_feedback) < .002) // FIXME
                        v = 0.002;
                else
                        v = v_feedback;
                latest_pose_ << x_feedback, y_feedback, psi_feedback;
                latest_velocity_ << v, w;
                latest_position_ << x_feedback, y_feedback;

                MF::PositionVectorD curr_robot_ori;
                curr_robot_ori << cos(latest_pose_(MF::oriIdx)), sin(latest_pose_(MF::oriIdx));
                rot_mat_2_wfr_ << curr_robot_ori(0,0), -1.*curr_robot_ori(1,0), curr_robot_ori(1,0), curr_robot_ori(0,0);
                rot_mat_2_rfr_ << curr_robot_ori(0,0), curr_robot_ori(1,0), -1.*curr_robot_ori(1,0), curr_robot_ori(0,0);

                target_pt_pose_ << target_x_feedback, target_y_feedback, target_psi_feedback;
                target_pt_velocity_ << target_v_feedback, target_omega_feedback;
                way_pt_pose_ = target_pt_pose_;
                way_pt_velocity_ = target_pt_velocity_;

                #if (DEBUG_ENABLED)

                std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " <<"planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                        "Init pose: " << RESET << latest_pose_.transpose() << std::endl;
                std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                        "Target pose: " << RESET << way_pt_pose_.transpose() << std::endl;

                std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " <<"planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                        "Init vel: " << RESET << latest_velocity_.transpose() << std::endl;
                std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                        "Target vel: " << RESET << way_pt_velocity_.transpose() << std::endl;

                #endif

                time_first_call_of_comp_vel_cmds_ = world_time_sec + 1e-9*world_time_nsec;
                time_since_first_call_ = 0;

                // INTI COMM (SHOULD CHANGE TO MULTMOD)
                // FIXME
                comm_link_->init();
        }
        else
        {
                time_since_first_call_ = world_time_sec + 1e-9*world_time_nsec - time_first_call_of_comp_vel_cmds_;
        }

        //
        if (idx_traj_being_executed_ >= 0)
        {
                eval_time_ = world_time_ - time_impl_of_first_traj_ -
                             idx_traj_being_executed_ * comp_time_;
                // eval_time_ += world_time_ - last_world_time_;

                // Condition for stopping outputting => executed all of the last plan
                if (last_traj_flag_ && eval_time_ > trajectory_.getParVarInterval())
                {
                        idx_traj_being_executed_ = -2;
                }
        }

        // std::cout << FG_BLUE << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << "eval_time_: " << eval_time_ << RESET << std::endl;

        // ---------- planning thread management

        if (idx_traj_being_executed_ != -2)
        {

                // ---------- If no plan has being computed yet ----------
                if (idx_traj_being_computed_ == -1)
                {
                        planning_thread_mutex_.lock();
                        ++idx_traj_being_computed_;
                        // std::cout << FG_BLUE << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << " Spawn thread" << RESET << std::endl;

                        thread_costmap_ = *costmap_;         // must be a hard copy
                        thread_world_time_sec_ = world_time_sec_;
                        thread_world_time_nsec_ = world_time_nsec_;
                        thread_eval_time_ = eval_time_;
                        costmap_ref_ = (MF::PoseVectorD() << x_feedback, y_feedback, psi_feedback).finished();
                        // MF::PositionVectorD costmap_ref_ori;
                        // costmap_ref_ori << cos(costmap_ref_(MF::oriIdx)), sin(costmap_ref_(MF::oriIdx));
                        // rot_mat_2_cmapfr_ << costmap_ref_ori(0,0), costmap_ref_ori(1,0), -1.*costmap_ref_ori(1,0), costmap_ref_ori(0,0);
                        getCostmapInfo();


                        planning_thread_ = bthread(&CODRHAPlanner::findTrajectory, this);

                        #if (DEBUG_ENABLED)

                        std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "pl anner:" << FG_RED << func_name << ":" << FG_YELLOW << " idx comp, ex " << idx_traj_being_computed_ << ", " << idx_traj_being_executed_ << RESET << std::endl;
                        // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << *(comm_list_.cbegin()) << ": " << comm_link_->getIntendedTraj(*(comm_list_.cbegin()))->dim << RESET << std::endl;

                        #endif
                }
                // ---------- If first plan is being computed ----------
                else if (idx_traj_being_computed_ == 0)
                {

                        // check if planning thread returned (unlocked the mutex)
                        if (planning_thread_mutex_.try_lock())
                        {
                                ++idx_traj_being_computed_;
                        }
                        else if (time_since_first_call_ >= timeout_for_computing_first_plan_)
                        {
                                planning_thread_.interrupt();
                                planning_thread_mutex_.lock();
                                ++idx_traj_being_computed_;
                        }

                        if (idx_traj_being_computed_ == 1)
                        {
                                // load new trajectory solution TODO
                                trajectory_ = opt_trajectory_;
                                // trajectory_.update(rot_mat_2_wfr_*opt_trajectory_.getCtrlPts() + latest_position_.replicate(1, opt_trajectory_.nCtrlPts()));

                                #if (LOG_ENABLED)

                                logTraj();
                                // logTrajMap();

                                #endif

                                collision_vehicles_.clear();

                                #if (DEBUG_ENABLED)

                                // thread_costmap_.saveMap("/home/turtlebot/dev/catkin_ws/src/codrha_local_planner/log/map" + std::to_string(idx_traj_being_computed_) + ".pgm");
                                //
                                // double px, py;
                                // unsigned mx, my;
                                // px = 1.2;
                                // py = 0.0;
                                // unsigned obst_cost = thread_costmap_.worldToMap(px, py, mx, my) ? thread_costmap_.getCost(mx, my) : 0;
                                //
                                // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                                //         "Costmap value at (" << px << ", " << py << ") = " << obst_cost << RESET << std::endl;
                                //
                                // MF::PositionVectorD position = MF::flatToPosition(trajectory_(0.0));
                                // px = position.x();
                                // py = position.y();
                                // obst_cost = thread_costmap_.worldToMap(px, py, mx, my) ? thread_costmap_.getCost(mx, my) : 0;
                                //
                                // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                                //         "Costmap value at beginning of traj (" << px << ", " << py << ") = " << obst_cost << RESET << std::endl;

                                std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                                        "Ctrl Pts WR? of first plan:" << RESET << std::endl;
                                std::cout << trajectory_.getCtrlPts() << std::endl;
                                std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                                        "Planned time interval: " << trajectory_.getParVarInterval() << RESET << std::endl;

                                // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << *(comm_list_.cbegin()) << ": " << comm_link_->getIntendedTraj(*(comm_list_.cbegin()))->dim << RESET << std::endl;

                                #endif

                                eval_time_ = 0.0;
                                time_impl_of_first_traj_ = world_time_;

                                ++idx_traj_being_executed_;

                                if (last_traj_flag_ == true)
                                {
                                        // TODO
                                        idx_traj_being_computed_ = -2;
                                }
                                else
                                {
                                                #if (DEBUG_ENABLED)

                                        std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                                                " Spawn 2nd thread" << RESET << std::endl;

                                                #endif
                                        thread_costmap_ = *costmap_;         // must be a hard copy
                                        thread_world_time_sec_ = world_time_sec_;
                                        thread_world_time_nsec_ = world_time_nsec_;
                                        thread_eval_time_ = eval_time_;
                                        costmap_ref_ = (MF::PoseVectorD() << x_feedback, y_feedback, psi_feedback).finished();
                                        // MF::PositionVectorD costmap_ref_ori;
                                        // costmap_ref_ori << cos(costmap_ref_(MF::oriIdx)), sin(costmap_ref_(MF::oriIdx));
                                        // rot_mat_2_cmapfr_ << costmap_ref_ori(0,0), costmap_ref_ori(1,0), -1.*costmap_ref_ori(1,0), costmap_ref_ori(0,0);
                                        getCostmapInfo();


                                        planning_thread_ = bthread(&CODRHAPlanner::findTrajectory, this);
                                }
                        }
                        // #if (DEBUG_ENABLED)
                        //
                        // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << " idx comp, ex " << idx_traj_being_computed_ << ", " << idx_traj_being_executed_ << RESET << std::endl;
                        //
                        // #endif
                }
                else if (idx_traj_being_computed_ > 0 &&
                         eval_time_ >= comp_time_)
                {                  // >= better then >, uses new plan as soon as possible


                        if (!planning_thread_mutex_.try_lock())         // We are supposed to get this lock
                        {
                                planning_thread_.interrupt();
                                planning_thread_mutex_.lock();
                        }
                        // timers_mutex_.lock();

                        eval_time_ -= comp_time_;         // Fix" eval_time_
                        // timers_mutex_.unlock();

                        ++idx_traj_being_computed_;
                        ++idx_traj_being_executed_;

                        // load new trajectory solution TODO
                        trajectory_ = opt_trajectory_;
                        // trajectory_.update(rot_mat_2_wfr_*opt_trajectory_.getCtrlPts() + latest_position_.replicate(1, opt_trajectory_.nCtrlPts()));
                        #if (LOG_ENABLED)

                        logTraj();
                        // logTrajMap();

                        #endif

                        collision_vehicles_.clear();

                        #if (DEBUG_ENABLED)

                        // thread_costmap_.saveMap("/home/turtlebot/dev/catkin_ws/src/codrha_local_planner/log/map" + std::to_string(idx_traj_being_computed_) + ".pgm");

                        // double px, py;
                        // unsigned mx, my;
                        // px = 1.2;
                        // py = 0.0;
                        // unsigned obst_cost = thread_costmap_.worldToMap(px, py, mx, my) ? thread_costmap_.getCost(mx, my) : 0;
                        //
                        // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                        //         "Costmap value at (" << px << ", " << py << ") = " << obst_cost << RESET << std::endl;
                        //
                        // MF::PositionVectorD position = MF::flatToPosition(trajectory_(0.0));
                        // px = position.x();
                        // py = position.y();
                        // obst_cost = thread_costmap_.worldToMap(px, py, mx, my) ? thread_costmap_.getCost(mx, my) : 0;
                        //
                        // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                        //         "Costmap value at beginning of traj (" << px << ", " << py << ") = " << obst_cost << RESET << std::endl;

                        std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                                "Ctrl Pts:" << RESET << std::endl;
                        std::cout << trajectory_.getCtrlPts() << std::endl;
                        std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW <<
                                "Planned time interval: " << trajectory_.getParVarInterval() << RESET << std::endl;

                        // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << *(comm_list_.cbegin()) << ": " << comm_link_->getIntendedTraj(*(comm_list_.cbegin()))->dim << RESET << std::endl;

                        #endif


                        if (last_traj_flag_ == true)
                        {
                                // TODO
                                planning_thread_mutex_.unlock();
                                idx_traj_being_computed_ = -2;
                                // idx_traj_being_executed_ = -2;
                        }
                        else
                        {
                                #if (DEBUG_ENABLED)

                                std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << " Spawn " << idx_traj_being_computed_+1 << "th thread" << RESET << std::endl;

                                #endif
                                thread_costmap_ = *costmap_;         // must be a hard copy
                                thread_world_time_sec_ = world_time_sec_;
                                thread_world_time_nsec_ = world_time_nsec_;
                                thread_eval_time_ = eval_time_;
                                costmap_ref_ = (MF::PoseVectorD() << x_feedback, y_feedback, psi_feedback).finished();
                                // MF::PositionVectorD costmap_ref_ori;
                                // costmap_ref_ori << cos(costmap_ref_(MF::oriIdx)), sin(costmap_ref_(MF::oriIdx));
                                // rot_mat_2_cmapfr_ << costmap_ref_ori(0,0), costmap_ref_ori(1,0), -1.*costmap_ref_ori(1,0), costmap_ref_ori(0,0);
                                getCostmapInfo();

                                planning_thread_ = bthread(&CODRHAPlanner::findTrajectory, this);
                        }

                        #if (DEBUG_ENABLED)

                        std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << " idx comp, ex " << idx_traj_being_computed_ << ", " << idx_traj_being_executed_ << RESET << std::endl;

                        #endif
                }
                // else
        }

        MF::VeloVectorD vel_ctrl_feed;
        MF::PoseVectorD pose_ctrl_feed;

        // ---------- output computation
        if (idx_traj_being_executed_ >= 0)
        {
                MF::VeloVectorD vel_at_eval_time;
                vel_at_eval_time = MF::flatToVelocity(trajectory_(eval_time_, MF::derivOrdForVelo));
                lin_vel_output = vel_at_eval_time[0];
                ang_vel_output = vel_at_eval_time[1];

                pose_ctrl_feed = MF::flatToPose(trajectory_(eval_time_, MF::derivOrdForPose));
                vel_ctrl_feed = vel_at_eval_time;
        }
        else
        {
                lin_vel_output = 0;
                ang_vel_output = 0;

                pose_ctrl_feed = target_pt_pose_;
                vel_ctrl_feed = target_pt_velocity_;
        }

        // {
        //         std::cout << FG_BLUE << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Call controller" << RESET << std::endl;
        // }
        //
        // stage = 0 means that the robot should not move, no plan is available
        // stage = 1 means is the normal execution stage
        // stage = 2 is the final stage, using the last computed plan
        double stage;
        if (idx_traj_being_executed_ == -1 || (idx_traj_being_executed_ == -2 && idx_traj_being_computed_ == -2))
                stage = 0;
        else if (idx_traj_being_computed_ != -2)
                stage = 1;
        else
        {
                stage = 2;
                // #if (DEBUG_ENABLED)
                //
                // std::cout << FG_BLUE << "[" << time_since_first_call_ << "] " << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << " plan stage " << stage << RESET << std::endl;
                //
                // #endif
        }

        low_level_ctrl_->computeVelocityCommands(
                world_time_sec_,
                world_time_nsec_,
                x_feedback,
                y_feedback,
                psi_feedback,
                v_feedback,
                omega_feedback,
                // pose_ctrl_feed[0],
                // pose_ctrl_feed[1],
                // pose_ctrl_feed[2],
                // vel_ctrl_feed[0],
                // vel_ctrl_feed[1],
                trajectory_,
                eval_time_,
                opt_plan_time_,
                stage,
                lin_vel_output,
                ang_vel_output);


        #if (LOG_ENABLED)

        logCommand(lin_vel_output, ang_vel_output);

        #endif

        // Set outputs
        // lin_vel_output = lin_vel_output;
        // ang_vel_output = ang_vel_output;

        // Everything went as expected
        return true;
}
// maybe put compute collision inside "update" and lock for findTrajectory
bool CODRHAPlanner::findTrajectory()
{
        #if (DEBUG_ENABLED)

        // Get function name
        std::string func_name(BOOST_CURRENT_FUNCTION);
        std::string delimiter = "CODRHAPlanner::";
        func_name = func_name.substr(func_name.find(delimiter)+delimiter.length(), func_name.length());
        delimiter = "(";
        func_name = func_name.substr(0, func_name.find(delimiter));
        std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << " started!" << RESET << std::endl;

        #endif

        // boost::mutex::scoped_lock l(configuration_mutex_);
        // Check if the robot is in the neigbourghood of the way point

        MF::PositionVectorD robot_ori_unit_vec;
        robot_ori_unit_vec << cos(latest_pose_(MF::oriIdx)), sin(latest_pose_(MF::oriIdx));

        MF::PositionVectorD rem_dist_unit_vec = MF::positionInPose(target_pt_pose_) - latest_position_;
        double rem_dist = rem_dist_unit_vec.norm();
        rem_dist_unit_vec /= rem_dist;

        double change_vel_dist = std::abs((target_pt_velocity_(MF::linSpeedIdx)*target_pt_velocity_(MF::linSpeedIdx) - max_vel_x_*max_vel_x_)/(2.*max_acc_x_));
        double change_vel_duration = 2.*change_vel_dist/std::abs((target_pt_velocity_(MF::linSpeedIdx) + max_vel_x_));

        if (rem_dist < comp_time_*max_vel_x_ + change_vel_dist + last_step_min_dist_)
        {
                last_traj_flag_ = true;
                opt_plan_time_ = change_vel_duration + std::max((rem_dist - change_vel_dist)/max_vel_x_, 0.0);
                // planning_thread_mutex_.unlock();
                // return true;
                // TODO recompute n_ctrlpts n_knots accordingly
                // std::stringstream ss;
                // ss << "TODO CODRHAPlanner::findTrajectory: part of implementation of last step prep missing. ";
                // throw(common::MyException(ss.str()));
        }
        else
        {
                opt_plan_time_ = plan_time_;
        }

        // Find next way point if need be

        // idx_traj_being_computed_ != 0
        if (last_traj_flag_ == false && opt_method_ != "SLSQP_ONLY")
        {
                // TODO implementation of find Next Way point
                findNextWayPt(rem_dist_unit_vec);
                // doNotSteerTooMuch();
        }
        else if (last_traj_flag_ == true)
        {
                way_pt_pose_ = target_pt_pose_;
        }

        MF::PositionVectorD direc_to_way_pt_unit_vec = MF::positionInPose(way_pt_pose_) - latest_position_;
        double rem_dist_to_way_pt = direc_to_way_pt_unit_vec.norm();
        direc_to_way_pt_unit_vec /= rem_dist_to_way_pt;

        min_cost_v_ = rem_dist_to_way_pt - max_vel_x_*opt_plan_time_;         // minimization of distance lower bound

        generateOptInitGuess(rem_dist, change_vel_dist, robot_ori_unit_vec, direc_to_way_pt_unit_vec, curve_);         // TODO check return status


        #if (DEBUG_ENABLED)

        if (last_traj_flag_)
        {
                std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Init Curve for LAST STEP (odom_frame):" << RESET << std::endl;
        }
        else
        {
                std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Init Curve (odom_frame):" << RESET << std::endl;
        }
        std::cout << curve_ << std::endl;

        #endif

        // std::cout << FG_BLUE << "planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Dims: (" << curve_.rows() << ", " << curve_.cols() << ")" << RESET << std::endl;
        opt_trajectory_.interpolate(curve_, opt_plan_time_);         // robot frame of reference

        #if (DEBUG_ENABLED)

        std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Init ctrl pts:" << RESET << std::endl;
        std::cout << opt_trajectory_.getCtrlPts() << std::endl;

        #endif

        // std::cout << name << FG_YELLOW << "BEFORE OPT, WRT ROBOT:"<< RESET << std::endl;
        // std::cout << opt_trajectory_.getCtrlPts() << std::endl;

        // Call opt sovler

        // _comAIVsSetMutex.lock();
        // collision_vehicles_.clear();
        // comm_outrage_vehicles_.clear();
        // _pathsFromConflictualAIVs.clear();
        // _comAIVsSetMutex.unlock();

        bool got_interrupted = false;

        double wt = thread_world_time_sec_ + thread_world_time_nsec_*1e-9;
        time_of_exec_ = (comp_time_ - thread_eval_time_) + wt;

        try
        {
                #if (DEBUG_ENABLED)
                std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Call solveOptProblem Step 1" << RESET << std::endl;
                #endif
                got_interrupted = solveOptProblem(); // after this call opt_trajectory_ has the optimized solution
                comm_link_->publishTrajectory(opt_trajectory_, comp_time_ - thread_eval_time_, thread_world_time_sec_, thread_world_time_nsec_, radius_); // FIXME eval_time has no place here, should give the estimated time to begin execution of this plan
        }
        catch (std::exception& e)
        {
                std::stringstream ss;
                ss << "CODRHAPlanner::_plan: call to solve optimization problem failed. " << e.what();
                planning_thread_mutex_.unlock();
                throw(common::MyException(ss.str()));
        }

        if (!got_interrupted)
        {
                #if (DEBUG_ENABLED)
                std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Check conflict" << RESET << std::endl;
                #endif

                // Compute conflicts: based only on other robots positions and max velocities compute if there is a change of collision
                // in the future Tp (more?). It updates collision_vehicles_ list
                conflictEval();

                // // FIXME
                // if (idx_traj_being_computed_ >= 2)
                //         collision_vehicles_.insert("tb01");

                #if (DEBUG_ENABLED)
                std::cout << FG_L_RED  << "list of collision vehicles: " << RESET << std::endl;
                for (auto i : collision_vehicles_)
                {
                        std::cout << i << '\n';
                }
                #endif

                if (!collision_vehicles_.empty())
                {
                        comm_link_->updateRobotsToListenTo(collision_vehicles_);

                        others_.clear();
                        for (auto robot_id : collision_vehicles_)
                        {
                                IntendedTraj traj;
                                bool isOk = comm_link_->getIntendedTraj(robot_id, traj);
                                if(isOk)
                                {
                                        others_[robot_id] = traj;
                                }
                        }

                        try
                        {
                                #if (DEBUG_ENABLED)
                                std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW <<  BG_RED << "Call solveOptProblem Step 2" << RESET << std::endl;
                                #endif

                                got_interrupted = solveOptProblem(); // after this call opt_trajectory_ has the optimized solution
                                // comm_link_->publishTrajectory(opt_trajectory_, comp_time_ - thread_eval_time_, thread_world_time_sec_, thread_world_time_nsec_, radius_); // FIXME eval_time has no place here, should give the estimated time to begin execution of this plan
                        }
                        catch (std::exception& e)
                        {
                                std::stringstream ss;
                                ss << "CODRHAPlanner::_plan: call to solve optimization problem failed. " << e.what();
                                planning_thread_mutex_.unlock();
                                throw(common::MyException(ss.str()));
                        }
                }
        }
        // double time_to_exec = comp_time_ - eval_time_;
        // double world_time_sec = world_time_sec_;
        // double world_time_nsec = world_time_nsec_;
        // comm_link_->publishTrajectory(opt_trajectory_, time_to_exec, world_time_sec, world_time_nsec, radius_);

        #if (DEBUG_ENABLED)
        std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Ctrl Pts after opt:" << RESET << std::endl;
        std::cout << opt_trajectory_.getCtrlPts() << std::endl;
        #endif

        // std::cout << "AFTER OPT, WRT DISPLACED WORLD:\n";
        // std::cout << opt_trajectory_.getCtrlPts() << std::endl;
        // std::cout << name << FG_GREEN << " AFTER OPT, WRT ROBOT:" << RESET << std::endl;
        // std::cout << rot_mat_2_rfr_ * opt_trajectory_.getCtrlPts() << std::endl;

        // CONFLICT COMPUTATION

        // Changing frames of reference
        // opt_trajectory_.update(rot_mat_2_wfr_*opt_trajectory_.getCtrlPts() + latest_position_.replicate(1, opt_trajectory_.nCtrlPts()));
        // curve_ = rot_mat_2_wfr_*curve_ + latest_position_.replicate(1, opt_trajectory_.nCtrlPts());

        // UPDATES

        MF::MatForVeloD deriv_flat = opt_trajectory_(comp_time_, MF::derivOrdForVelo);

        latest_pose_ = MF::flatToPose(deriv_flat);
        latest_position_ = MF::positionInPose(latest_pose_);
        latest_velocity_ = MF::flatToVelocity(deriv_flat);

        robot_ori_unit_vec << cos(latest_pose_(MF::oriIdx)), sin(latest_pose_(MF::oriIdx));
        // std::cout <<  "current direction: " << robot_ori_unit_vec << std::endl;

        rot_mat_2_wfr_ << robot_ori_unit_vec(0,0), -1.*robot_ori_unit_vec(1,0), robot_ori_unit_vec(1,0), robot_ori_unit_vec(0,0);
        rot_mat_2_rfr_ << robot_ori_unit_vec(0,0), robot_ori_unit_vec(1,0), -1.*robot_ori_unit_vec(1,0), robot_ori_unit_vec(0,0);

        planning_thread_mutex_.unlock();
        return true;
        // std::cout << "END OF PLAN\n";
}

/**
 * Compute conflicts: based only on other robots positions and max velocities compute if there is a change of collision
 * in the future Tp (more?). It updates collision_vehicles_ list
 */
void CODRHAPlanner::conflictEval()
{
        // for // over known robots which for now will be the list comm_list_
        // collision_vehicles_.clear();
        for (auto robot_id : comm_list_) // FIXME should be known robots list
        {
                double dist = comm_link_->getDist(thread_world_time_sec_, thread_world_time_nsec_, robot_id);
                std::cout << FG_L_RED << "--- dist from " << robot_id << ": " << dist << RESET << std::endl;
                if (dist < 2.*max_vel_x_*opt_plan_time_) // so this robot is in collision conflit
                {
                        collision_vehicles_.insert(robot_id);
                }
        }
}

void CODRHAPlanner::generateOptInitGuess(
        double rem_dist,
        double change_vel_dist,
        const MF::PositionVectorD& curr_direc,
        const MF::PositionVectorD& direc_to_wp,
        Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic>& curve)
{
        std::string init_mode_for_final_step = "mixed";         // FIXME

        std::stringstream ss;

        double accel = acc_sup_x_init_;

        double max_vel = max_vel_init_factor_*max_vel_x_;

        double maxDisplVariation;
        double prevDispl = 0.0;
        // Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic> curve(MF::flatDim, traj_spl_n_ctrl_pts_);

        maxDisplVariation = (opt_plan_time_/(traj_spl_n_ctrl_pts_ - 1))*max_vel;

        prevDispl = 0.0;

        // Create a sampled trajectory for a "bounded uniformed accelerated motion" in x axis
        Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic> curveCurrDirec(MF::flatDim, traj_spl_n_ctrl_pts_);
        curveCurrDirec.setZero();

        // Create a sampled trajectory for a "bounded uniformed accelerated motion" in (direc-init_direc) direction in the xy plane
        Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic> curveNewDirec(MF::flatDim, traj_spl_n_ctrl_pts_);
        curveNewDirec.setZero();

        double beforeBreakDispl;
        double beforeBreakDeltaT;
        bool breakOn = false;

        // max_velo_in_plan_ = 0;
        // curveNewDirec.col(0) = 0.0;
        // curveCurrDirec.col(0) = 0.0;
        curveNewDirec.col(0) = latest_position_;
        curveCurrDirec.col(0) = latest_position_;
        double prevVel = max_vel;
        for (auto i=1; i < traj_spl_n_ctrl_pts_; ++i)
        {
                double deltaT = i*opt_plan_time_/(traj_spl_n_ctrl_pts_-1);

                double displ;

                if (last_traj_flag_ && prevDispl >= rem_dist - change_vel_dist)
                {
                        if (breakOn == false)
                        {
                                beforeBreakDispl = prevDispl;
                                beforeBreakDeltaT = (i-1)*opt_plan_time_/(traj_spl_n_ctrl_pts_-1);
                                // accel = -1.*max_acc_x_/2.;
                                accel = -1.*std::abs((target_pt_velocity_(MF::linSpeedIdx)*target_pt_velocity_(MF::linSpeedIdx) - max_vel*max_vel)/(2.*rem_dist - 2.*prevDispl));
                                breakOn = true;
                        }
                        displ = beforeBreakDispl + max_vel*(deltaT-beforeBreakDeltaT) + accel/2.*(deltaT-beforeBreakDeltaT)*(deltaT-beforeBreakDeltaT);
                }
                else
                {
                        displ = latest_velocity_(MF::linSpeedIdx)*deltaT + accel/2.*deltaT*deltaT;
                }
                displ = displ - prevDispl < maxDisplVariation ? std::max(displ, prevDispl) : prevDispl + maxDisplVariation;
                prevVel = (displ - prevDispl)/deltaT;
                prevDispl = displ;
                curveCurrDirec.col(i) = displ*curr_direc+latest_position_;
                curveNewDirec.col(i) = displ*direc_to_wp+latest_position_;

                // double max_velocity_in_plan_candidate = displ / (opt_plan_time_/(traj_spl_n_ctrl_pts_-1));
                // max_velo_in_plan_ = max_velocity_in_plan_candidate > max_velo_in_plan_ ? max_velocity_in_plan_candidate : max_velo_in_plan_;
        }


        double magicNumber = first_guess_tweak_;
        double p;

        for (auto i=0; i < traj_spl_n_ctrl_pts_; ++i)
        {
                p = (atan((2*double(i)/traj_spl_n_ctrl_pts_-1)*magicNumber)/atan(magicNumber) + 1.0)/2.0;
                curve.col(i) = curveCurrDirec.col(i)*(1.0-p) + curveNewDirec.col(i)*p;
        }

        // IF LAST STEP LET US COMBINE CURVE WITH ANOTHER ONE THAT SMOOTHS THE ARRIVAL
        if (last_traj_flag_ && init_mode_for_final_step.compare("forward") != 0)
        // if (false)
        {
                MF::PositionVectorD fromGoalDirec;
                fromGoalDirec << cos(target_pt_pose_.tail<1>()(0,0) - M_PI),
                        sin(target_pt_pose_.tail<1>()(0,0) - M_PI);
                MF::PositionVectorD toGoalDirec;
                toGoalDirec << cos(target_pt_pose_.tail<1>()(0,0)),
                        sin(target_pt_pose_.tail<1>()(0,0));
                // std::cout << "fromGoalDirec bef rot\n" << fromGoalDirec << std::endl;

                // fromGoalDirec = fromGoalDirec;

                // MF::PositionVectorD oposed2GoalDirec;
                // oposed2GoalDirec = latest_position_ - MF::positionInPose(target_pt_pose_);
                // oposed2GoalDirec /= oposed2GoalDirec.norm();

                // oposed2GoalDirec = rot_mat_2_rfr_*oposed2GoalDirec;
                // std::cout << "oposed2GoalDirec\n" << oposed2GoalDirec << std::endl;

                maxDisplVariation = (opt_plan_time_/(traj_spl_n_ctrl_pts_ - 1))*max_vel;

                // accel = max_acc_x_/2.0;
                // accel = max_acc_x_;
                accel = -acc_inf_x_init_;

                prevDispl = 0.0;

                // Create a sampled trajectory for a "bounded uniformed accelerated motion" in x axis
                Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic> fromGoalCurve(MF::flatDim, traj_spl_n_ctrl_pts_);
                fromGoalCurve.col(0) = toGoalDirec*.01 + MF::positionInPose(target_pt_pose_);

                for (auto i=1; i<traj_spl_n_ctrl_pts_; ++i)
                {
                        double deltaT = i*opt_plan_time_/(traj_spl_n_ctrl_pts_-1);
                        double displ = way_pt_velocity_(MF::linSpeedIdx)*deltaT + accel/2.*deltaT*deltaT;
                        displ = displ - prevDispl < maxDisplVariation ? std::max(displ, prevDispl) : prevDispl + maxDisplVariation;
                        prevDispl = displ;
                        fromGoalCurve.col(i) = MF::positionInPose(target_pt_pose_) + displ*fromGoalDirec;
                        // toGoalCurve.col(i) = displ*oposed2GoalDirec;
                }
                // Eigen::Matrix<double, MF::flatDim, MF::flatDim> rot180;
                // rot180 << -1, 0, 0, -1;
                Eigen::Matrix<double, MF::flatDim, Eigen::Dynamic> finalFromGoalCurve;
                // std::cout << "fromGoalCurve bef changes\n" << fromGoalCurve << std::endl;

                finalFromGoalCurve = fromGoalCurve.rowwise().reverse();
                // +(rot_mat_2_rfr_*(MF::positionInPose(target_pt_pose_) - latest_position_)).replicate(1, traj_spl_n_ctrl_pts_);

                if (init_mode_for_final_step.compare("mixed") == 0)
                {
                        // std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Using mixed (" << init_mode_for_final_step << ")!!!" << RESET << std::endl;
                        // double magicNumber = _firstGuessMixCte; //FIXME no magic numbers, at least no hard coded
                        // double magicNumber = 1.5; //FIXME no magic numbers, at least no hard coded
                        // throw(common::MyException("FIXME CODRHAPlanner::generateOptInitGuess: no magic numbers. "));
                        // double p;

                        for (auto i=0; i < traj_spl_n_ctrl_pts_; ++i)
                        //for (auto i=0; i < MF::flatDim; ++i)
                        {
                                // p = pow(double(i)/traj_spl_n_ctrl_pts_, magicNumber);
                                p = (atan((2*double(i)/traj_spl_n_ctrl_pts_-1)*magicNumber)/atan(magicNumber) + 1.0)/2.0;
                                curve.col(i) = curve.col(i)*(1.0-p) + finalFromGoalCurve.col(i)*p;
                        }
                }
                else if (init_mode_for_final_step.compare("backward") == 0)
                {
                        // std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << "Using backward (" << init_mode_for_final_step << ")!!!" << RESET << std::endl;
                        curve = finalFromGoalCurve;
                }
                else
                {
                        ss.clear();        //clear any bits set
                        ss.str(std::string());
                        ss << "CODRHAPlanner::plan: invalid value for init_mode_for_final_step [ " << init_mode_for_final_step << " ]. ";
                        throw(common::MyException(ss.str()));
                }
        }
}

bool CODRHAPlanner::findNextWayPt(const MF::PositionVectorD& direc_to_target)
{
        way_pt_pose_ << latest_position_ + direc_to_target * cost_tweak_ * opt_plan_time_ * max_vel_x_, MF::yawInPose(target_pt_pose_);
        return true;
}


/**
 * Call nlopt library to compute the optimal trajectory.
 * It expects an initial guess in the robots frame of reference and outputs the
 * optimized trajectory also in the robots frame of reference.
 * @return true if an interruption to the planning thread was detected, false
 *         otherwisse
 */
bool CODRHAPlanner::solveOptProblem()
{

        #if (DEBUG_ENABLED)

        // Get function name
        std::string func_name(BOOST_CURRENT_FUNCTION);
        std::string delimiter = "CODRHAPlanner::";
        func_name = func_name.substr(func_name.find(delimiter)+delimiter.length(), func_name.length());
        delimiter = "(";
        func_name = func_name.substr(0, func_name.find(delimiter));

        #endif

        unsigned n_param;
        unsigned n_equations;
        unsigned n_inequations;

        // pointer to methods typedefs
        // typedef double (CODRHAPlanner::*ObjFunPointer)(unsigned, const double*, double*, void*);
        // typedef double (CODRHAPlanner::*EqIneqFunPointer)(unsigned, const double*, double*, void*);

        double (*objective_func) (unsigned, const double*, double*, void*);
        void (*equations_func) (unsigned, double*, unsigned, const double*, double*, void*);
        void (*inequations_func) (unsigned, double*, unsigned, const double*, double*, void*);

        double *opt_params;
        double *tol_for_equations;
        double *tol_for_inequations;

        int status;

        bool got_interrupted = false;

        int n_samples = int(opt_plan_time_/plan_granularity_);

        if (last_traj_flag_ == false)
        {
                objective_func = CODRHAPlanner::objectiveFunc;
                equations_func = CODRHAPlanner::equationConstraints;
                inequations_func = CODRHAPlanner::inequationConstraints;
                n_param = opt_trajectory_.nParam();
                n_equations = MF::poseDim + MF::veloDim;
                // n_inequations = MF::veloDim * n_samples +
                // MF::accelDim * (n_samples + 1) +
                // detected_obstacles_.size() * n_samples;
                // n_inequations = MF::veloDim * n_samples +
                //                 MF::accelDim * (n_samples + 1) +
                //                 detected_obstacles_.size() * n_samples +
                //                 collision_vehicles_.size() * n_samples +
                //                 comm_outrage_vehicles_.size() * n_samples;
                // n_inequations = MF::veloDim * n_samples +
                //                 detected_obstacles_.size() * n_samples;

                if (respect_obst_const_)
                        n_inequations = MF::veloDim * n_samples +
                                        MF::accelDim * (n_samples + 1) +
                                        1 * n_samples +
                                        collision_vehicles_.size() * n_samples;         // obstacles
                else
                        n_inequations = MF::veloDim * n_samples +
                                        MF::accelDim * (n_samples + 1) +
                                        collision_vehicles_.size() * n_samples;

                // std::cout <<  "======  Number of detected obstacles: " << detected_obstacles_.size() << std::endl;

                params_ = new double[n_param];
                init_params_ = new double[n_param];
                opt_trajectory_.getParameters(init_params_);

                opt_params = new double[n_param];
                for (auto i = 0; i < n_param; ++i)
                {
                        opt_params[i] = 1.0;
                }
        }
        else
        {
                objective_func = CODRHAPlanner::objectiveFuncTermination;
                equations_func = CODRHAPlanner::equationConstraintsTermination;
                inequations_func = CODRHAPlanner::inequationConstraintsTermination;
                n_param = opt_trajectory_.nParam() + 1;
                n_equations = (MF::poseDim + MF::veloDim) * 2;
                // n_inequations = MF::veloDim * (n_samples - 1) +
                //                 MF::accelDim * (n_samples + 1) +
                //                 detected_obstacles_.size() * (n_samples - 1) +
                //                 collision_vehicles_.size() * (n_samples-1) +
                //                 comm_outrage_vehicles_.size() * (n_samples-1) +
                //                 2; // time > 0
                // n_inequations = MF::veloDim * (n_samples-1) +
                //                 detected_obstacles_.size() * (n_samples-1);
                if (respect_obst_const_term_)
                        n_inequations = MF::veloDim * (n_samples-1) +
                                        MF::accelDim * (n_samples + 1) +
                                        1 * (n_samples-1);
                                        // collision_vehicles_.size() * (n_samples - 1);         // osbtacles
                else
                        n_inequations = MF::veloDim * (n_samples-1) +
                                        MF::accelDim * (n_samples + 1);
                                        // collision_vehicles_.size() * (n_samples - 1);
                // n_inequations = MF::veloDim * (n_samples-1) +
                // MF::accelDim * (n_samples+1) +
                // detected_obstacles_.size() * (n_samples-1);

                params_ = new double[n_param];
                init_params_ = new double[n_param];
                init_params_[0] = opt_plan_time_;
                opt_trajectory_.getParameters(&(init_params_[1]));

                opt_params = new double[n_param];
                for (auto i = 0; i < n_param; ++i)
                {
                        opt_params[i] = 1.0;
                }
        }

        tol_for_equations = new double[n_equations];
        for (unsigned i = 0; i < n_equations; ++i)
        {
                tol_for_equations[i] = opt_equetions_abs_tol_;
        }

        tol_for_inequations = new double[n_inequations];
        for (unsigned i = 0; i < n_inequations; ++i)
        {
                tol_for_inequations[i] = opt_inequetions_abs_tol_;
        }

        if (opt_method_ == "SLSQP" || opt_method_ == "COBYLA")
        {
                // nlopt_opt opt;
                // if (opt_method_ == "COBYLA" || opt_method_ == "COBYLA_ONLY")
                if (opt_method_ == "COBYLA")
                {
                        opt_pbl_ = nlopt_create(NLOPT_LN_COBYLA, n_param);
                }
                // else if (opt_method_ == "SLSQP" || opt_method_ == "SLSQP_ONLY")
                else if (opt_method_ == "SLSQP")
                {
                        opt_pbl_ = nlopt_create(NLOPT_LD_SLSQP, n_param);
                }
                else
                {
                        std::stringstream ss;
                        ss << "CODRHAPlanner::_solveOptPbl: unknown optimization method [" << opt_method_ << "]. Verify config.xml.";
                        delete[] opt_params;
                        delete[] params_;
                        delete[] init_params_;
                        delete[] tol_for_equations;
                        delete[] tol_for_inequations;
                        throw(common::MyException(ss.str()));
                }

                nlopt_set_ftol_abs(opt_pbl_, opt_objective_func_abs_tol_);
                nlopt_set_xtol_rel(opt_pbl_, opt_param_rel_tol_);
                nlopt_set_xtol_abs1(opt_pbl_, opt_param_abs_tol_);
                nlopt_set_min_objective(opt_pbl_, objective_func, this);
                nlopt_add_equality_mconstraint(opt_pbl_, n_equations, equations_func, this, tol_for_equations);
                nlopt_add_inequality_mconstraint(opt_pbl_, n_inequations, inequations_func, this, tol_for_inequations);
                // nlopt_add_equality_mconstraint(opt_pbl_, n_equations, *equations_func, this, tol_for_equations);
                // nlopt_add_inequality_mconstraint(opt_pbl_, n_inequations, *inequations_func, this, tol_for_inequations);

                double minf;

                opt_iteractions_counter_ = 0;

                // std::cout << std::string(name).substr(0,10) << "::Optimizing..." << std::endl;
                #if (DEBUG_ENABLED)

                std::cout << FG_BLUE << "(*)planner:" << FG_RED << func_name << ":" << FG_YELLOW << " Calling nlopt_optimize" << RESET << std::endl;

                #endif

                // LATEST MOMENT FOR ADJUSTING OTHER ROBOTS INFORMATION IF WE
                // WANT OPTMIZATION INFORMATION TO BE CONSTANT (AND WE WANT THAT)
                //
                // FIXME IMPLMENT THIS
                // adaptCollisionRobotsInfo(); //Gets intended trajectories (if available) and works out sampling at time differences


                // status = 1;
                status = nlopt_optimize(opt_pbl_, opt_params, &minf);

                // std::cout << _executingPlanIdx << " Done optimizing" << std::endl;

                //int status = -1;

                //int msg = status;
                std::stringstream msg;

                switch(status)
                {
                case 1:
                        msg << FG_GREEN << "Generic success return value" << RESET;
                        break;
                case 2:
                        msg << FG_GREEN << "Optimization stopped because stopval was reached" << RESET;
                        break;
                case 3:
                        msg << FG_GREEN << "Optimization stopped because ftol_rel or ftol_abs was reached" << RESET;
                        break;
                case 4:
                        msg << FG_GREEN << "Optimization stopped because xtol_rel or xtol_abs was reached" << RESET;
                        break;
                case 5:
                        msg << FG_GREEN << "Optimization stopped because maxeval was reached" << RESET;
                        break;
                case 6:
                        msg << FG_GREEN << "Optimization stopped because maxtime was reached" << RESET;
                        break;
                case -1:
                        msg << FG_RED << "Generic failure" << RESET << " code";
                        break;
                case -2:
                        msg << "Invalid arguments (e.g. lower bounds are bigger than upper bounds, an unknown algorithm was specified, etcetera)";
                        break;
                case -3:
                        msg << "Ran " << FG_RED << "out of memory" << RESET;
                        break;
                case -4:
                        msg << "Halted because " << FG_RED << "roundoff errors" << RESET << " limited progress (in this case, the optimization still typically returns a useful result)";
                        break;
                case -5:
                        msg << "Halted because of a " << FG_RED << "forced termination" << RESET << ": the user called nlopt_force_stop(opt) on the optimization’s nlopt_opt object opt from the user’s objective function or constraints";
                }
                std::cout << "Optimization ended with status: \"" << msg.str() << "\" after " << opt_iteractions_counter_ << " iterations" << std::endl;
                got_interrupted = status == -5;
        }
        else if (opt_method_ == "NONE")
        {
                //do nothing

                // TESTING FOR INTERRUPTIONS
                // try { boost::this_thread::sleep(boost::posix_time::milliseconds(20000)); }
                // catch (boost::thread_interrupted&) { got_interrupted = true; }
                // std::cout << "NONE" << std::endl;
                status = 1;
        }
        else
        {
                status = -1;
                std::stringstream ss;
                ss << "CODRHAPlanner::_solveOptPbl: unknown optimization method [" << opt_method_ << "]. Verify config.xml.";
                delete[] opt_params;
                delete[] params_;
                delete[] init_params_;
                delete[] tol_for_equations;
                delete[] tol_for_inequations;
                throw(common::MyException(ss.str()));
        }

        #if (LOG_ENABLED)

        // Log cost and constraints values for the found solution
        double cost = objective_func(n_param, opt_params, NULL, this);
        double * constrEq = new double[n_equations];
        equations_func(n_equations, constrEq, n_param, opt_params, NULL, this);
        double * constrIneq = new double[n_inequations];
        inequations_func(n_inequations, constrIneq, n_param, opt_params, NULL, this);
        logOpt(cost, constrEq, n_equations, constrIneq, n_inequations);

        delete[] constrEq;
        delete[] constrIneq;

        #endif

        for (int i = 0; i < n_param; ++i)
        {
                params_[i] = opt_params[i] - 1. + init_params_[i];
        }

        // std::cout << opt_trajectory_.cArray2CtrlPtsMat(opt_params) << std::endl;

        // Recovering from error status
        // Recovering means testing if all optimized parameter are valid values, if so they are considered an useful result
        // TODO Decide if we must recover from any < 0 status
        // if (status == -5) // recovering only from forced interruption
        if (status < -3)         // recovering from forced interruption and rundoff errors
        {
                bool isSolutionUponErrorOk = true;

                // Check if results are usable
                for (unsigned i=0; i < n_param; ++i)
                {
                        if(opt_params[i] != opt_params[i])         //Only checking for NAN (and IND)
                        {
                                isSolutionUponErrorOk = false;
                                break;
                        }
                }
                if (isSolutionUponErrorOk)
                {
                        status = 1;
                }
        }

        if (status > 0)
        {
                if (last_traj_flag_)
                {
                        // opt_trajectory_.update(opt_trajectory_.cArray2CtrlPtsMat(&(params_[1])), params_[0]);
                        opt_trajectory_.update(&(params_[1]), params_[0]);
                        opt_plan_time_ = params_[0];
                }
                else
                {
                        // opt_trajectory_.update(opt_trajectory_.cArray2CtrlPtsMat(params_));
                        opt_trajectory_.update(params_);
                }
        }
        else
        {
                if (last_traj_flag_)
                {
                        // opt_trajectory_.update(opt_trajectory_.cArray2CtrlPtsMat(&(init_params_[1])), init_params_[0]);
                        opt_trajectory_.update(&(init_params_[1]), init_params_[0]);
                        opt_plan_time_ = init_params_[0];
                }
                else
                {
                        // opt_trajectory_.update(opt_trajectory_.cArray2CtrlPtsMat(init_params_));
                        opt_trajectory_.update(init_params_);
                }
        }

        // boost::this_thread::interruption_point(); // Say bye bye if optimization took too long

        delete[] opt_params;
        delete[] params_;
        delete[] init_params_;
        delete[] tol_for_equations;
        delete[] tol_for_inequations;

        return got_interrupted;
        // std::cout << "END OF SOLVE OPT PROB\n";
        // boost::this_thread::interruption_point(); // Say bye bye if optimization took too long
}

// //FIXME
// void CODRHAPlanner::adaptCollisionRobotsInfo()
// {
//         paths_from_collision_robots_.clear();
//         for(auto robot_id : collision_vehicles_)
//         {
//                 IntendedTraj* robot_info = comm_link_->getIntendedTraj(robot_id);
//                 if (robot_info != NULL)
//                 {
//                         MF::PositionVectorD robot_position;
//                         // robot_position << robot_info->x[0], robot_info->y[0];
//                         paths_from_collision_robots_[robot_id] = robot_position.replicate(1, int(opt_plan_time_/plan_granularity_));
//                 }
//                 else
//                 {
//                         //TODO
//                 }
//                 // comm_link_->getDist(thread_world_time_sec_, thread_world_time_nsec_, robot_id);
//         }
// }

void CODRHAPlanner::computeNumGrad(unsigned m, unsigned n, const double* x, double* grad, void (*eval)(unsigned, double*, unsigned, volatile double*, CODRHAPlanner*))
{

        double h;
        const double sqrtEps = std::sqrt(num_dif_eps_);

        double *f_at_x_minus_2h = new double[m];
        double *f_at_x_minus_h = new double[m];
        // double *f_at_x = new double[m];
        double *f_at_x_plus_h = new double[m];
        double *f_at_x_plus_2h = new double[m];

        volatile double *x_minus_2h = new double[n];
        volatile double *x_minus_h = new double[n];
        // volatile double *vol_x = new double[n];
        volatile double *x_plus_h = new double[n];
        volatile double *x_plus_2h = new double[n];

        int nanCount=0;

        for (auto i = 0; i < n; ++i)
        {
                x_minus_2h[i] = x[i];
                x_minus_h[i] = x[i];
                // vol_x[i] = x[i];
                x_plus_h[i] = x[i];
                x_plus_2h[i] = x[i];
        }

        //
        // A choice for h which is small without producing a large rounding
        // error is {\displaystyle {\sqrt {\varepsilon }}x} {\sqrt  {\varepsilon }}x
        // (though not when x = 0!) where the machine epsilon ε is typically of
        // the order 2.2×10−16. [7]

        // h = num_dif_eps_;
        for (int i = 0; i < int(n); ++i)
        {

                //compute for i=0 all m js with the five-points formula
                h = x[i] < sqrtEps ? num_dif_eps_ : sqrtEps*x[i];

                x_minus_2h[i] = x[i] - 2.*h;
                x_minus_h[i] = x[i] - h;
                x_plus_h[i] = x[i] + h;
                x_plus_2h[i] = x[i] + 2.*h;

                (*eval)(m, f_at_x_minus_2h, n, x_minus_2h, this);
                (*eval)(m, f_at_x_minus_h, n, x_minus_h, this);
                // (*eval)(m, f_at_x, n, vol_x, this);
                (*eval)(m, f_at_x_plus_h, n, x_plus_h, this);
                (*eval)(m, f_at_x_plus_2h, n, x_plus_2h, this);

                for (int j = 0; j < int(m); ++j)
                {
                        grad[j*n + i] = (f_at_x_minus_2h[j] - 8.*f_at_x_minus_h[j] + 8.*f_at_x_plus_h[j] - f_at_x_plus_2h[j]) /
                                        (12.*h);
                        // (8*x_plus_h[j] - 8*x_minus_h[j] - (x_plus_2h[j] - x_minus_2h[j]));
                        // (12.*h);
                        // grad[j*n + i] = (f_at_x_minus_2h[j] - 8.*f_at_x_minus_h[j] + 8.*f_at_x_plus_h[j] - f_at_x_plus_2h[j]) / (12.*h);
                        // grad[j*n + i] = (f_at_x_plus_h[j] - f_at_x[j]) / (x_plus_h[j] - vol_x[j]);
                        if (boost::math::isnan(grad[j*n + i]))
                        {
                                ++nanCount;
                                std::cout << FG_YELLOW << "NAN" << RESET << std::endl;
                                // grad[j*n + i] = (f_at_x_minus_2h[j] - 8.*f_at_x_minus_h[j] + 8.*f_at_x_plus_h[j] - f_at_x_plus_2h[j]) /
                                // (12.*h);
                                // if (boost::math::isnan(grad[j*n + i]))
                                // {
                                //         std::cout << FG_YELLOW << "NAN" << RESET << std::endl;
                                // }
                                // grad[j*n + i] = 0.0;
                        }
                }

                x_minus_2h[i] = x[i];
                x_minus_h[i] = x[i];
                x_plus_h[i] = x[i];
                x_plus_2h[i] = x[i];
        }

        delete[] f_at_x_minus_2h;
        delete[] f_at_x_minus_h;
        // delete[] f_at_x;
        delete[] f_at_x_plus_h;
        delete[] f_at_x_plus_2h;
        delete[] x_minus_2h;
        delete[] x_minus_h;
        // delete[] vol_x;
        delete[] x_plus_h;
        delete[] x_plus_2h;
}

void CODRHAPlanner::computeNumGrad(unsigned m, unsigned n, const double* x, const double* eps, double* grad, void (*eval)(unsigned, double*, unsigned, volatile double*, CODRHAPlanner*))
{
        double h, normal_h;
        const double sqrtEps = std::sqrt(num_dif_eps_);

        double *f_at_x_minus_2h = new double[m];
        double *f_at_x_minus_h = new double[m];
        // double *f_at_x = new double[m];
        double *f_at_x_plus_h = new double[m];
        double *f_at_x_plus_2h = new double[m];

        volatile double *x_minus_2h = new double[n];
        volatile double *x_minus_h = new double[n];
        // volatile double *vol_x = new double[n];
        volatile double *x_plus_h = new double[n];
        volatile double *x_plus_2h = new double[n];

        int nanCount=0;

        for (auto i = 0; i < n; ++i)
        {
                x_minus_2h[i] = x[i];
                x_minus_h[i] = x[i];
                // vol_x[i] = x[i];
                x_plus_h[i] = x[i];
                x_plus_2h[i] = x[i];
        }

        //
        // A choice for h which is small without producing a large rounding
        // error is {\displaystyle {\sqrt {\varepsilon }}x} {\sqrt  {\varepsilon }}x
        // (though not when x = 0!) where the machine epsilon ε is typically of
        // the order 2.2×10−16. [7]

        // h = num_dif_eps_;
        for (int i = 0; i < int(n); ++i)
        {
                for (int j = 0; j < int(m); ++j)
                {
                        //compute for i=0 all m js with the five-points formula
                        normal_h = x[i] < sqrtEps ? num_dif_eps_ : sqrtEps*x[i];
                        h = eps[i * m + j] == 0.0 ? normal_h : eps[i * m + j];

                        x_minus_2h[i] = x[i] - 2.*h;
                        x_minus_h[i] = x[i] - h;
                        x_plus_h[i] = x[i] + h;
                        x_plus_2h[i] = x[i] + 2.*h;

                        (*eval)(m, f_at_x_minus_2h, n, x_minus_2h, this);
                        (*eval)(m, f_at_x_minus_h, n, x_minus_h, this);
                        // (*eval)(m, f_at_x, n, vol_x, this);
                        (*eval)(m, f_at_x_plus_h, n, x_plus_h, this);
                        (*eval)(m, f_at_x_plus_2h, n, x_plus_2h, this);

                        grad[j*n + i] = (f_at_x_minus_2h[j] - 8.*f_at_x_minus_h[j] + 8.*f_at_x_plus_h[j] - f_at_x_plus_2h[j]) /
                                        (12.*h);
                        // (8*x_plus_h[j] - 8*x_minus_h[j] - (x_plus_2h[j] - x_minus_2h[j]));
                        // (12.*h);
                        // grad[j*n + i] = (f_at_x_minus_2h[j] - 8.*f_at_x_minus_h[j] + 8.*f_at_x_plus_h[j] - f_at_x_plus_2h[j]) / (12.*h);
                        // grad[j*n + i] = (f_at_x_plus_h[j] - f_at_x[j]) / (x_plus_h[j] - vol_x[j]);
                        if (boost::math::isnan(grad[j*n + i]))
                        {
                                ++nanCount;
                                std::cout << FG_YELLOW << "NAN" << RESET << std::endl;
                                // grad[j*n + i] = (f_at_x_minus_2h[j] - 8.*f_at_x_minus_h[j] + 8.*f_at_x_plus_h[j] - f_at_x_plus_2h[j]) /
                                // (12.*h);
                                // if (boost::math::isnan(grad[j*n + i]))
                                // {
                                //         std::cout << FG_YELLOW << "NAN" << RESET << std::endl;
                                // }
                                // grad[j*n + i] = 0.0;
                        }

                        x_minus_2h[i] = x[i];
                        x_minus_h[i] = x[i];
                        x_plus_h[i] = x[i];
                        x_plus_2h[i] = x[i];
                }
        }

        delete[] f_at_x_minus_2h;
        delete[] f_at_x_minus_h;
        // delete[] f_at_x;
        delete[] f_at_x_plus_h;
        delete[] f_at_x_plus_2h;
        delete[] x_minus_2h;
        delete[] x_minus_h;
        // delete[] vol_x;
        delete[] x_plus_h;
        delete[] x_plus_2h;
}

double CODRHAPlanner::objectiveFunc(unsigned n, const double *x, double *grad, void *data)
{
        CODRHAPlanner *context = static_cast<CODRHAPlanner *>(data);
        context->opt_iteractions_counter_++;
        double result;
        evalObjectiveFunc(1, &result, n, x, context);
        if (grad) context->computeNumGrad(1, n, x, grad, &evalObjectiveFunc);
        // Analytical Solution
        // if (grad)
        // {
        //         for (auto i = 0; i < n-2; ++i)
        //         {
        //                 grad[i] = 0.0;
        //         }
        //         MF::PositionVectorD way_pt_position(MF::positionInPose(context->way_pt_pose_));
        //
        //         way_pt_position =
        //                 context->rot_mat_2_rfr_*(way_pt_position - context->latest_position_);
        //         grad[n-1] = 2.*(x[n-1] - way_pt_position[0]);
        //         grad[n-2] = 2.*(x[n-2] - way_pt_position[1]);
        // }

        return result;
}

void CODRHAPlanner::equationConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
{
        CODRHAPlanner *context = static_cast<CODRHAPlanner *>(data);
        evalEquationConstraints(m, result, n, x, context);
        if (grad) context->computeNumGrad(m, n, x, grad, &evalEquationConstraints);
}

// bool test = true;

void CODRHAPlanner::inequationConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
{
        CODRHAPlanner *context = static_cast<CODRHAPlanner *>(data);
        evalInequationConstraints(m, result, n, x, context);
        if (grad)
        {
                context->computeNumGrad(m, n, x, grad, &evalInequationConstraints);
                // if (context->respect_obst_const_)
                // {
                //
                //         int n_samples = int(context->opt_plan_time_/context->plan_granularity_);
                //         double *eps = new double[n*m];
                //
                //         for (auto i = 0; i < n; ++i)
                //         {
                //                 for (auto j = 0; j < m; ++j)
                //                 {
                //                         eps[i * m + j] = j >= m - n_samples ? 0.02 : 0.0; // TODO
                //                 }
                //         }
                //         // if (test)
                //         // {
                //         //         for (auto i = 0; i < n; ++i)
                //         //         {
                //         //                 for (auto j = 0; j < m; ++j)
                //         //                 {
                //         //                         std::cout << eps[i * m + j] << ", ";
                //         //                 }
                //         //                 std::cout << std::endl;
                //         //         }
                //         //         test = false;
                //         // }
                //         context->computeNumGrad(m, n, x, eps, grad, &evalInequationConstraints);
                //         delete[] eps;
                // }
                // else
                // {
                //         context->computeNumGrad(m, n, x, grad, &evalInequationConstraints);
                // }
        }
}

double CODRHAPlanner::objectiveFuncTermination(unsigned n, const double *x, double *grad, void *data)
{
        CODRHAPlanner *context = static_cast<CODRHAPlanner *>(data);
        context->opt_iteractions_counter_++;
        double result;
        evalObjectiveFuncTermination(1, &result, n, x, context);
        if (grad) context->computeNumGrad(1, n, x, grad, &evalObjectiveFuncTermination);
        // Analytical Solution
        // if (grad)
        // {
        //         for (auto i = 1; i < n; ++i)
        //         {
        //                 grad[i] = 0.0;
        //         }
        //         grad[0] = 2.*x[0];
        // }
        return result;
}
void CODRHAPlanner::equationConstraintsTermination(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
{
        CODRHAPlanner *context = static_cast<CODRHAPlanner *>(data);
        evalEquationConstraintsTermination(m, result, n, x, context);
        if (grad) context->computeNumGrad(m, n, x, grad, &evalEquationConstraintsTermination);
}


void CODRHAPlanner::inequationConstraintsTermination(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
{
        CODRHAPlanner *context = static_cast<CODRHAPlanner *>(data);
        evalInequationConstraintsTermination(m, result, n, x, context);
        if (grad)
        {
                context->computeNumGrad(m, n, x, grad, &evalInequationConstraintsTermination);
                // if (context->respect_obst_const_)
                // {
                //         // int n_samples = int(context->opt_plan_time_/context->plan_granularity_);
                //         // double *eps = new double[n*m];
                //         // for (auto i = 0; i < n; ++i)
                //         // {
                //         //         for (auto j = 0; j < m; ++j)
                //         //         {
                //         //                 eps[i * m + j] = j >= m - n_samples + 1 ? 0.03 : 0.0; // TODO
                //         //         }
                //         // }
                //         // if (test)
                //         // {
                //         //         for (auto i = 0; i < n; ++i)
                //         //         {
                //         //                 for (auto j = 0; j < m; ++j)
                //         //                 {
                //         //                         std::cout << eps[i * m + j] << ", ";
                //         //                 }
                //         //                 std::cout << std::endl;
                //         //         }
                //         //         test = false;
                //         // }
                //
                //         // context->computeNumGrad(m, n, x, eps, grad, &evalInequationConstraintsTermination);
                //         // delete[] eps;
                // }
                // else
                // {
                //         context->computeNumGrad(m, n, x, grad, &evalInequationConstraintsTermination);
                // }
        }
}

void CODRHAPlanner::logOpt(double cost, const double* ceq, unsigned n_ceq, const double* cineq, unsigned n_cineq)
{

        int secs = int(world_time_);
        int nsecs = 1e9 * (world_time_ - secs);

        opt_log_ << "header:" << std::endl;
        opt_log_ << "  seq: " << idx_traj_being_computed_ << std::endl;
        opt_log_ << "  stamp:" << std::endl;
        opt_log_ << "    secs: " << secs << std::endl;
        opt_log_ << "    nsecs: " << nsecs << std::endl;
        opt_log_ << "  frame_id: odom" << std::endl;
        opt_log_ << "child_frame_id: base_footprint" << std::endl;

        opt_log_ << "cost: " << cost << std::endl;

        opt_log_ << "constraints:" << std::endl;
        opt_log_ << "  n_equations: " << n_ceq << std::endl;
        opt_log_ << "  equations: [";
        for (auto i = 0; i < n_ceq-1; ++i)
        {
                opt_log_ << ceq[i] << ", ";
        }
        opt_log_ << ceq[n_ceq-1] << "]" << std::endl;

        opt_log_ << "  n_inequations: " << n_cineq << std::endl;
        opt_log_ << "  inequations: [";
        for (auto i = 0; i < n_cineq-1; ++i)
        {
                opt_log_ << cineq[i] << ", ";
        }
        opt_log_ << cineq[n_cineq-1] << "]" << std::endl;

        opt_log_ << "---" << std::endl;
}

void CODRHAPlanner::logCommand(double lin_cmd,
                               double ang_cmd)
{
        int secs = int(world_time_);
        int nsecs = 1e9 * (world_time_ - secs);

        command_log_ << "header:" << std::endl;
        command_log_ << "  seq: " << idx_traj_being_executed_ << std::endl;
        command_log_ << "  stamp:" << std::endl;
        command_log_ << "    secs: " << secs << std::endl;
        command_log_ << "    nsecs: " << nsecs << std::endl;
        command_log_ << "  frame_id: odom" << std::endl;
        command_log_ << "child_frame_id: base_footprint" << std::endl;
        command_log_ << "twist:" << std::endl;
        command_log_ << "  twist:" << std::endl;
        command_log_ << "    linear:" << std::endl;
        command_log_ << "      x: " << lin_cmd << std::endl;
        command_log_ << "      y: " << 0.0 << std::endl;
        command_log_ << "      z: " << 0.0 << std::endl;
        command_log_ << "    angular:" << std::endl;
        command_log_ << "      x: " << 0.0 << std::endl;
        command_log_ << "      y: " << 0.0 << std::endl;
        command_log_ << "      z: " << ang_cmd << std::endl;
        command_log_ << "---" << std::endl;

}
void CODRHAPlanner::logFeedback(double x_feedback,
                                double y_feedback,
                                double psi_feedback,
                                double target_x_feedback,
                                double target_y_feedback,
                                double target_psi_feedback,
                                double v_feedback,
                                double omega_feedback)
{
        long unsigned secs = world_time_sec_;
        long unsigned nsecs = world_time_nsec_;
        double qw = std::cos(psi_feedback * 0.5);
        double qz = std::sin(psi_feedback * 0.5);
        double target_qw = std::cos(target_psi_feedback * 0.5);
        double target_qz = std::sin(target_psi_feedback * 0.5);

        feedback_log_ << "header:" << std::endl;
        feedback_log_ << "  seq: " << -1 << std::endl;
        feedback_log_ << "  stamp:" << std::endl;
        feedback_log_ << "    secs: " << secs << std::endl;
        feedback_log_ << "    nsecs: " << nsecs << std::endl;
        feedback_log_ << "  frame_id: odom" << std::endl;
        feedback_log_ << "child_frame_id: base_footprint" << std::endl;
        feedback_log_ << "pose:" << std::endl;
        feedback_log_ << "  pose:" << std::endl;
        feedback_log_ << "    position:" << std::endl;
        feedback_log_ << "      x: " << x_feedback << std::endl;
        feedback_log_ << "      y: " << y_feedback << std::endl;
        feedback_log_ << "      z: " << 0.0 << std::endl;
        feedback_log_ << "    orientation:" << std::endl;
        feedback_log_ << "      x: " << 0.0 << std::endl;
        feedback_log_ << "      y: " << 0.0 << std::endl;
        feedback_log_ << "      z: " << qz << std::endl;
        feedback_log_ << "      w: " << qw << std::endl;
        feedback_log_ << "twist:" << std::endl;
        feedback_log_ << "  twist:" << std::endl;
        feedback_log_ << "    linear:" << std::endl;
        feedback_log_ << "      x: " << v_feedback << std::endl;
        feedback_log_ << "      y: " << 0.0 << std::endl;
        feedback_log_ << "      z: " << 0.0 << std::endl;
        feedback_log_ << "    angular:" << std::endl;
        feedback_log_ << "      x: " << 0.0 << std::endl;
        feedback_log_ << "      y: " << 0.0 << std::endl;
        feedback_log_ << "      z: " << omega_feedback << std::endl;
        feedback_log_ << "target_pose:" << std::endl;
        feedback_log_ << "  pose:" << std::endl;
        feedback_log_ << "    position:" << std::endl;
        feedback_log_ << "      x: " << target_x_feedback << std::endl;
        feedback_log_ << "      y: " << target_y_feedback << std::endl;
        feedback_log_ << "      z: " << 0.0 << std::endl;
        feedback_log_ << "    orientation:" << std::endl;
        feedback_log_ << "      x: " << 0.0 << std::endl;
        feedback_log_ << "      y: " << 0.0 << std::endl;
        feedback_log_ << "      z: " << target_qz << std::endl;
        feedback_log_ << "      w: " << target_qw << std::endl;
        feedback_log_ << "---" << std::endl;

}

void CODRHAPlanner::logTrajMap()
{
        unsigned nCtrlPts = trajectory_.nCtrlPts();
        unsigned parVarInterval = trajectory_.getParVarInterval();
        double nSamples = 60;
        double eval_time;

        double time_offset = (idx_traj_being_computed_-1)*comp_time_;

        double log_plan_time = opt_plan_time_;
        double log_comp_time = last_traj_flag_ ? opt_plan_time_ : comp_time_;

        int secs = world_time_sec_;
        int nsecs = world_time_nsec_;

        typedef std::vector<double> ListD;
        ListD p_x, p_y, p_theta, v_x, v_y, ext_v_x, ext_v_y, tm, ext_tm;

        MF::PoseVectorD pose;
        MF::VeloVectorD velo;

        for(int i = 0; i < nSamples; ++i)
        {
                eval_time = i/double(nSamples-1)*log_comp_time;
                pose = MF::flatToPose(trajectory_(eval_time, MF::derivOrdForPose));
                double x, y;
                comm_link_->transform2DPointFromPlanRef2Map(pose.x(), pose.y(), x, y);
                velo = MF::flatToVelocity(trajectory_(eval_time, MF::derivOrdForVelo));
                p_x.push_back(x);
                p_y.push_back(y);
                p_theta.push_back(pose(2));
                v_x.push_back(velo(0));
                v_y.push_back(velo(1));
                tm.push_back(time_offset+eval_time);

                // eval_time = i/double(nSamples-1)*log_plan_time;
                velo = MF::flatToVelocity(trajectory_(eval_time, MF::derivOrdForVelo));
                ext_v_x.push_back(velo(0));
                ext_v_y.push_back(velo(1));
                ext_tm.push_back(time_offset+eval_time);
        }

        traj_log_map_ << "header:" << std::endl;
        traj_log_map_ << "  seq: " << idx_traj_being_computed_-1 << std::endl;
        traj_log_map_ << "  stamp:" << std::endl;
        traj_log_map_ << "    secs: " << secs << std::endl;
        traj_log_map_ << "    nsecs: " << nsecs << std::endl;
        traj_log_map_ << "  frame_id: map" << std::endl;
        // traj_log_map_ << "child_frame_id: base_footprint" << std::endl;

        traj_log_map_ << "time_to_exec: " << eval_time_ << std::endl;

        traj_log_map_ << "time_seq: [";

        for(ListD::const_iterator i = tm.begin(); i !=  tm.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << tm.back() << "]" << std::endl;

        traj_log_map_ << "ext_time_seq: [";

        for(ListD::const_iterator i = ext_tm.begin(); i !=  ext_tm.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << ext_tm.back() << "]" << std::endl;

        traj_log_map_ << "control_points:" << std::endl;

        traj_log_map_ << "  x: [";

        for(int i = 0; i < nCtrlPts-1; ++i)
                traj_log_map_ << trajectory_.getCtrlPts()(0,i) << ", ";
        traj_log_map_ << trajectory_.getCtrlPts()(0,nCtrlPts-1) << "]" << std::endl;

        traj_log_map_ << "  y: [";

        for(int i = 0; i < nCtrlPts-1; ++i)
                traj_log_map_ << trajectory_.getCtrlPts()(1,i) << ", ";
        traj_log_map_ << trajectory_.getCtrlPts()(1,nCtrlPts-1) << "]" << std::endl;

        traj_log_map_ << "points:" << std::endl;

        traj_log_map_ << "  x: [";

        for(ListD::const_iterator i = p_x.begin(); i !=  p_x.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << p_x.back() << "]" << std::endl;

        traj_log_map_ << "  y: [";

        for(ListD::const_iterator i = p_y.begin(); i !=  p_y.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << p_y.back() << "]" << std::endl;

        traj_log_map_ << "  theta: [";

        for(ListD::const_iterator i = p_theta.begin(); i !=  p_theta.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << p_theta.back() << "]" << std::endl;

        traj_log_map_ << "init_points:" << std::endl;

        traj_log_map_ << "  x: [";

        for(int i = 0; i < nCtrlPts-1; ++i)
                traj_log_map_ << curve_(0,i) << ", ";
        traj_log_map_ << curve_(0,nCtrlPts-1) << "]" << std::endl;

        traj_log_map_ << "  y: [";

        for(int i = 0; i < nCtrlPts-1; ++i)
                traj_log_map_ << curve_(1,i) << ", ";
        traj_log_map_ << curve_(1,nCtrlPts-1) << "]" << std::endl;

        traj_log_map_ << "velocities:" << std::endl;

        traj_log_map_ << "  x: [";

        for(ListD::const_iterator i = v_x.begin(); i !=  v_x.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << v_x.back() << "]" << std::endl;

        traj_log_map_ << "  y: [";

        for(ListD::const_iterator i = v_y.begin(); i !=  v_y.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << v_y.back() << "]" << std::endl;

        traj_log_map_ << "extended_velocities:" << std::endl;

        traj_log_map_ << "  x: [";

        for(ListD::const_iterator i = ext_v_x.begin(); i !=  ext_v_x.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << ext_v_x.back() << "]" << std::endl;

        traj_log_map_ << "  y: [";

        for(ListD::const_iterator i = ext_v_y.begin(); i !=  ext_v_y.end()-1; ++i)
                traj_log_map_ << *i << ", ";
        traj_log_map_ << ext_v_y.back() << "]" << std::endl;

        // traj_log_map_ << "costmap:" << std::endl;
        //
        // traj_log_map_ << "  resolution: " << thread_costmap_.getResolution() << std::endl;
        // traj_log_map_ << "  size_in_cells_x: " << thread_costmap_.getSizeInCellsX() << std::endl;
        // traj_log_map_ << "  size_in_cells_y: " << thread_costmap_.getSizeInCellsY() << std::endl;
        // traj_log_map_ << "  size_in_meters_x: " << thread_costmap_.getSizeInMetersX() << std::endl;
        // traj_log_map_ << "  size_in_meters_y: " << thread_costmap_.getSizeInMetersY() << std::endl;
        // traj_log_map_ << "  origin_x: " << thread_costmap_.getOriginX() << std::endl;
        // traj_log_map_ << "  origin_y: " << thread_costmap_.getOriginY() << std::endl;
        //
        // traj_log_map_ << "  ref_vec: [";
        // for (auto i = 0; i < MF::poseDim - 1; ++i)
        // {
        //         traj_log_map_ << costmap_ref_[i] << ", ";
        // }
        // traj_log_map_ << costmap_ref_[MF::poseDim - 1] << "]" << std::endl;
        //
        // traj_log_map_ << "  costmap: [";

        // unsigned mapXDim = thread_costmap_.getSizeInCellsX();
        // unsigned mapYDim = thread_costmap_.getSizeInCellsY();
        // unsigned char * charMap = thread_costmap_.getCharMap();
        //
        // for(auto i = 0; i < mapXDim * mapYDim - 1; ++i)
        // {
        //         traj_log_map_ << unsigned(charMap[i]) << ", ";
        // }
        // traj_log_map_ << unsigned(charMap[mapXDim * mapYDim - 1]) << "]" << std::endl;


        traj_log_map_ << "radius: " << radius_ << std::endl;

        if (!collision_vehicles_.empty())
        {
                traj_log_map_ << "others_tb:" << std::endl;

                MF::PositionVectorD position;
                ListD op_x, op_y;

                for(auto robot_id : collision_vehicles_)
                {
                        traj_log_map_ << "  " << robot_id << ":";

                        if (others_.find(robot_id) != others_.end())
                        {
                                // unsigned splDeg = robot_info->splDeg;
                                // FIXME TODO think about generate Trajectory without knowing the degree
                                Trajectory< double, MF::flatDim, MF::derivOrdForAccel + 1 > othTraj(others_[robot_id].nCtrlPts/2);
                                othTraj.update(others_[robot_id].ctrlPts, others_[robot_id].pvar);

                                for(int i = 0; i < nSamples; ++i)
                                {
                                        eval_time = i/double(nSamples-1)*others_[robot_id].pvar;
                                        position = MF::flatToPosition(othTraj(eval_time, MF::derivOrdForPosition));
                                        // velo = MF::flatToVelocity(trajectory_(eval_time, MF::derivOrdForVelo));
                                        // std::string tbnamespace(ros::this_node::getNamespace());
                                        // tbnamespace = tbnamespace.substr(1, tbnamespace.length());
                                        // tf_listener_.lookupTransform(tbnamespace + "_tf/base_footprint",
                                        double x, y;

                                        // comm_link_->transform2DPointFromMapRef2Odom(position.x(), position.y(), world_time_sec_, world_time_nsec_, x, y);
                                        comm_link_->transform2DPointFromMapRef2Odom(position.x(), position.y(), thread_world_time_sec_, thread_world_time_nsec_, x, y);

                                        op_x.push_back(x);
                                        op_y.push_back(y);
                                }


                                traj_log_map_ << std::endl;
                                traj_log_map_ << "    points:" << std::endl;

                                traj_log_map_ << "      x: [";
                                for(ListD::const_iterator i = op_x.begin(); i !=  op_x.end()-1; ++i)
                                        traj_log_map_ << *i << ", ";
                                traj_log_map_ << op_x.back() << "]" << std::endl;

                                traj_log_map_ << "      y: [";
                                for(ListD::const_iterator i = op_y.begin(); i !=  op_y.end()-1; ++i)
                                        traj_log_map_ << *i << ", ";
                                traj_log_map_ << op_y.back() << "]" << std::endl;

                                double time_from_stamp = others_[robot_id].stamp.sec + others_[robot_id].stamp.nsec*1e-9;
                                double relative_eval_time_for_other_robot = time_of_exec_ - (time_from_stamp + others_[robot_id].time_to_exec);
                                traj_log_map_ << "    time_from_stamp: " <<  time_from_stamp - int(time_from_stamp/1000)*1000 << std::endl;
                                traj_log_map_ << "    others_[robot_id].time_to_exec: " <<  others_[robot_id].time_to_exec << std::endl;
                                traj_log_map_ << "    time_of_exec_: " <<  time_of_exec_ - int(time_of_exec_/1000)*1000 << std::endl;
                                traj_log_map_ << "    relative_eval0: " <<  relative_eval_time_for_other_robot << std::endl;

                                op_x.clear();
                                op_y.clear();

                        }
                        else
                        {
                                traj_log_map_ << " null" << std::endl;
                        }
                }
        }
        traj_log_map_ << "---" << std::endl;


        // unsigned char *      getCharMap () const
        //      Will return a pointer to the underlying unsigned char array used as the costmap.

        //         double       getOriginX () const
        //      Accessor for the x origin of the costmap.
        // double       getOriginY () const
        //      Accessor for the y origin of the costmap.
        // double       getResolution () const
        //      Accessor for the resolution of the costmap.
        // unsigned     getSizeInCellsX () const
        //      Accessor for the x size of the costmap in cells.
        // unsigned     getSizeInCellsY () const
        //      Accessor for the y size of the costmap in cells.
        // double       getSizeInMetersX () const
        //      Accessor for the x size of the costmap in meters.
        // double       getSizeInMetersY () const
        //      Accessor for the y size of the costmap in meters.

}

void CODRHAPlanner::logTraj()
{
        unsigned nCtrlPts = trajectory_.nCtrlPts();
        unsigned parVarInterval = trajectory_.getParVarInterval();
        double nSamples = 50;
        double eval_time;

        double time_offset = (idx_traj_being_computed_-1)*comp_time_;

        double log_plan_time = opt_plan_time_;
        double log_comp_time = last_traj_flag_ ? opt_plan_time_ : comp_time_;

        int secs = world_time_sec_;
        int nsecs = world_time_nsec_;

        typedef std::vector<double> ListD;
        ListD p_x, p_y, p_theta, v_x, v_y, ext_p_x, ext_p_y, tm, ext_tm;
        ListD tf_x, tf_y, tf_theta, ext_tf_x, ext_tf_y;

        MF::PoseVectorD pose;
        MF::PositionVectorD positon;
        MF::VeloVectorD velo;

        for(int i = 0; i < nSamples; ++i)
        {
                double x, y, theta;

                // IMPLEMENTED TRAJ
                eval_time = i/double(nSamples-1)*log_comp_time;

                pose = MF::flatToPose(trajectory_(eval_time, MF::derivOrdForPose));
                comm_link_->transform2DPoseFromPlanRef2Odom(pose.x(), pose.y(), pose.z(), secs, nsecs, x, y, theta);

                velo = MF::flatToVelocity(trajectory_(eval_time, MF::derivOrdForVelo));

                p_x.push_back(pose.x());
                p_y.push_back(pose.y());
                p_theta.push_back(pose.z());
                tf_x.push_back(x);
                tf_y.push_back(y);
                tf_theta.push_back(theta);
                v_x.push_back(velo(0));
                v_y.push_back(velo(1));
                tm.push_back(time_offset+eval_time);

                // EXTENDED TRAJ
                // Computed x, y for plan horizon
                eval_time = i/double(nSamples-1)*log_plan_time;
                positon = MF::flatToPosition(trajectory_(eval_time, MF::derivOrdForPose));
                comm_link_->transform2DPointFromPlanRef2Odom(positon.x(), positon.y(), secs, nsecs, x, y);
                // x = positon.x();
                // y = positon.y();
                ext_p_x.push_back(positon.x());
                ext_p_y.push_back(positon.y());
                ext_tf_x.push_back(x);
                ext_tf_y.push_back(y);

                ext_tm.push_back(time_offset+eval_time);
        }

        traj_log_ << "header:" << std::endl;
        traj_log_ << "  seq: " << idx_traj_being_computed_-1 << std::endl;
        traj_log_ << "  stamp:" << std::endl;
        traj_log_ << "    secs: " << secs << std::endl;
        traj_log_ << "    nsecs: " << nsecs << std::endl;
        traj_log_ << "  frame_id: odom" << std::endl;
        // traj_log_ << "child_frame_id: base_footprint" << std::endl;

        traj_log_ << "time_to_exec: " << 0.0 << std::endl;

        traj_log_ << "time_seq: [";

        for(ListD::const_iterator i = tm.begin(); i !=  tm.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << tm.back() << "]" << std::endl;

        traj_log_ << "ext_time_seq: [";

        for(ListD::const_iterator i = ext_tm.begin(); i !=  ext_tm.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << ext_tm.back() << "]" << std::endl;

        // traj_log_ << "control_points:" << std::endl;
        //
        // traj_log_ << "  x: [";
        //
        // for(int i = 0; i < nCtrlPts-1; ++i)
        //         traj_log_ << trajectory_.getCtrlPts()(0,i) << ", ";
        // traj_log_ << trajectory_.getCtrlPts()(0,nCtrlPts-1) << "]" << std::endl;
        //
        // traj_log_ << "  y: [";
        //
        // for(int i = 0; i < nCtrlPts-1; ++i)
        //         traj_log_ << trajectory_.getCtrlPts()(1,i) << ", ";
        // traj_log_ << trajectory_.getCtrlPts()(1,nCtrlPts-1) << "]" << std::endl;

        traj_log_ << "points:" << std::endl;

        traj_log_ << "  x: [";

        for(ListD::const_iterator i = p_x.begin(); i !=  p_x.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << p_x.back() << "]" << std::endl;

        traj_log_ << "  y: [";

        for(ListD::const_iterator i = p_y.begin(); i !=  p_y.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << p_y.back() << "]" << std::endl;

        traj_log_ << "  theta: [";

        for(ListD::const_iterator i = p_theta.begin(); i !=  p_theta.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << p_theta.back() << "]" << std::endl;

        traj_log_ << "tf_poitns:" << std::endl;

        traj_log_ << "  x: [";

        for(ListD::const_iterator i = tf_x.begin(); i !=  tf_x.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << tf_x.back() << "]" << std::endl;

        traj_log_ << "  y: [";

        for(ListD::const_iterator i = tf_y.begin(); i !=  tf_y.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << tf_y.back() << "]" << std::endl;

        traj_log_ << "  theta: [";

        for(ListD::const_iterator i = tf_theta.begin(); i !=  tf_theta.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << tf_theta.back() << "]" << std::endl;

        traj_log_ << "extended_points:" << std::endl;

        traj_log_ << "  x: [";

        for(ListD::const_iterator i = ext_p_x.begin(); i !=  ext_p_x.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << ext_p_x.back() << "]" << std::endl;

        traj_log_ << "  y: [";

        for(ListD::const_iterator i = ext_p_y.begin(); i !=  ext_p_y.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << ext_p_y.back() << "]" << std::endl;

        traj_log_ << "extended_tf_points:" << std::endl;

        traj_log_ << "  x: [";

        for(ListD::const_iterator i = ext_tf_x.begin(); i !=  ext_tf_x.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << ext_tf_x.back() << "]" << std::endl;

        traj_log_ << "  y: [";

        for(ListD::const_iterator i = ext_tf_y.begin(); i !=  ext_tf_y.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << ext_tf_y.back() << "]" << std::endl;

        // traj_log_ << "  theta: [";
        //
        // for(ListD::const_iterator i = ext_p_theta.begin(); i !=  ext_p_theta.end()-1; ++i)
        //         traj_log_ << *i << ", ";
        // traj_log_ << ext_p_theta.back() << "]" << std::endl;

        // traj_log_ << "init_points:" << std::endl;
        //
        // traj_log_ << "  x: [";
        //
        // for(int i = 0; i < nCtrlPts-1; ++i)
        //         traj_log_ << curve_(0,i) << ", ";
        // traj_log_ << curve_(0,nCtrlPts-1) << "]" << std::endl;
        //
        // traj_log_ << "  y: [";
        //
        // for(int i = 0; i < nCtrlPts-1; ++i)
        //         traj_log_ << curve_(1,i) << ", ";
        // traj_log_ << curve_(1,nCtrlPts-1) << "]" << std::endl;

        traj_log_ << "velocities:" << std::endl;

        traj_log_ << "  x: [";

        for(ListD::const_iterator i = v_x.begin(); i !=  v_x.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << v_x.back() << "]" << std::endl;

        traj_log_ << "  y: [";

        for(ListD::const_iterator i = v_y.begin(); i !=  v_y.end()-1; ++i)
                traj_log_ << *i << ", ";
        traj_log_ << v_y.back() << "]" << std::endl;

        // traj_log_ << "extended_velocities:" << std::endl;
        //
        // traj_log_ << "  x: [";
        //
        // for(ListD::const_iterator i = ext_v_x.begin(); i !=  ext_v_x.end()-1; ++i)
        //         traj_log_ << *i << ", ";
        // traj_log_ << ext_v_x.back() << "]" << std::endl;
        //
        // traj_log_ << "  y: [";
        //
        // for(ListD::const_iterator i = ext_v_y.begin(); i !=  ext_v_y.end()-1; ++i)
        //         traj_log_ << *i << ", ";
        // traj_log_ << ext_v_y.back() << "]" << std::endl;

        traj_log_ << "costmap:" << std::endl;

        traj_log_ << "  resolution: " << thread_costmap_.getResolution() << std::endl;
        traj_log_ << "  size_in_cells_x: " << thread_costmap_.getSizeInCellsX() << std::endl;
        traj_log_ << "  size_in_cells_y: " << thread_costmap_.getSizeInCellsY() << std::endl;
        traj_log_ << "  size_in_meters_x: " << thread_costmap_.getSizeInMetersX() << std::endl;
        traj_log_ << "  size_in_meters_y: " << thread_costmap_.getSizeInMetersY() << std::endl;
        traj_log_ << "  origin_x: " << thread_costmap_.getOriginX() << std::endl;
        traj_log_ << "  origin_y: " << thread_costmap_.getOriginY() << std::endl;

        traj_log_ << "  ref_vec: [";
        for (auto i = 0; i < MF::poseDim - 1; ++i)
        {
                traj_log_ << costmap_ref_[i] << ", ";
        }
        traj_log_ << costmap_ref_[MF::poseDim - 1] << "]" << std::endl;

        traj_log_ << "  costmap: [";

        unsigned mapXDim = thread_costmap_.getSizeInCellsX();
        unsigned mapYDim = thread_costmap_.getSizeInCellsY();
        unsigned char * charMap = thread_costmap_.getCharMap();

        for(auto i = 0; i < mapXDim * mapYDim - 1; ++i)
        {
                traj_log_ << unsigned(charMap[i]) << ", ";
        }
        traj_log_ << unsigned(charMap[mapXDim * mapYDim - 1]) << "]" << std::endl;

        traj_log_ << "radius: " << radius_ << std::endl;

        if (!collision_vehicles_.empty())
        {
                traj_log_ << "others_tb:" << std::endl;

                MF::PositionVectorD position;
                ListD op_x, op_y;

                for(auto robot_id : collision_vehicles_)
                {
                        traj_log_ << "  " << robot_id << ":";

                        // IntendedTraj* robot_info = comm_link_->getIntendedTraj(robot_id);
                        // if (robot_info != NULL)
                        if(others_.find(robot_id) != others_.end())
                        {
                                double time_from_stamp = others_[robot_id].stamp.sec + others_[robot_id].stamp.nsec*1e-9;
                                double relative_eval_time_for_other_robot = time_of_exec_ - (time_from_stamp + others_[robot_id].time_to_exec);

                                // unsigned splDeg = robot_info->splDeg;
                                // FIXME TODO think about generate Trajectory without knowing the degree
                                Trajectory< double, MF::flatDim, MF::derivOrdForAccel + 1 > othTraj(others_[robot_id].nCtrlPts/2);
                                othTraj.update(others_[robot_id].ctrlPts, others_[robot_id].pvar);

                                for(int i = 0; i < nSamples; ++i)
                                {
                                        eval_time = i/double(nSamples-1)*others_[robot_id].pvar;
                                        // position = MF::flatToPosition(othTraj(eval_time+relative_eval_time_for_other_robot, MF::derivOrdForPosition));
                                        position = MF::flatToPosition(othTraj(eval_time, MF::derivOrdForPosition));
                                        // velo = MF::flatToVelocity(trajectory_(eval_time, MF::derivOrdForVelo));
                                        // std::string tbnamespace(ros::this_node::getNamespace());
                                        // tbnamespace = tbnamespace.substr(1, tbnamespace.length());
                                        // tf_listener_.lookupTransform(tbnamespace + "_tf/base_footprint",
                                        double x, y;

                                        comm_link_->transform2DPointFromMapRef2Odom(position.x(), position.y(), secs, nsecs, x, y);

                                        op_x.push_back(x);
                                        op_y.push_back(y);
                                }


                                traj_log_ << std::endl;
                                traj_log_ << "    points:" << std::endl;

                                traj_log_ << "      x: [";
                                for(ListD::const_iterator i = op_x.begin(); i !=  op_x.end()-1; ++i)
                                        traj_log_ << *i << ", ";
                                traj_log_ << op_x.back() << "]" << std::endl;

                                traj_log_ << "      y: [";
                                for(ListD::const_iterator i = op_y.begin(); i !=  op_y.end()-1; ++i)
                                        traj_log_ << *i << ", ";
                                traj_log_ << op_y.back() << "]" << std::endl;

                                // position = MF::flatToPosition(othTraj(relative_eval_time_for_other_robot, MF::derivOrdForPosition));
                                // position = MF::flatToPosition(othTraj(relative_eval_time_for_other_robot+others_[robot_id].pvar, MF::derivOrdForPosition));

                                traj_log_ << "    this traj time_of_exec_: " <<  time_of_exec_ - int(time_of_exec_/1000)*1000 << std::endl;
                                traj_log_ << "    time_from_stamp: " <<  time_from_stamp - int(time_from_stamp/1000)*1000 << std::endl;
                                traj_log_ << "    time_to_exec: " <<  others_[robot_id].time_to_exec << std::endl;
                                traj_log_ << "    time_from_stamp + time_to_exec: " <<  others_[robot_id].time_to_exec + time_from_stamp - int(time_from_stamp/1000)*1000 << std::endl;
                                traj_log_ << "    relative_eval0: " <<  relative_eval_time_for_other_robot << std::endl;

                                op_x.clear();
                                op_y.clear();

                        }
                        else
                        {
                                traj_log_ << " null" << std::endl;
                        }
                }
        }
        traj_log_ << "---" << std::endl;
}

double CODRHAPlanner::interpolateCost(const double& px,
                                      const double& py,
                                      const unsigned& mx,
                                      const unsigned& my,
                                      const costmap_2d::Costmap2D& costmap,
                                      const unsigned& max_cell_value,
                                      const unsigned& rad_dist_cell_value,
                                      const double& radius)
{
        // We'll solve Ax = b
        const unsigned cols = 5;         // x2 y2 x y 1
        const unsigned rows = 9;

        // Let's create A and b
        Eigen::MatrixXd A(rows, cols);

        Eigen::VectorXd b(rows);

        double wx, wy;
        unsigned i = 0;
        for (auto drow = -1; drow < 2; ++drow)
        {
                for (auto dcol = -1; dcol < 2; ++dcol)
                {
                        unsigned cost = costmap.getCost(mx+dcol, my+drow);
                        b[i] = radius / (max_cell_value - rad_dist_cell_value) * cost - radius / (max_cell_value - rad_dist_cell_value) * rad_dist_cell_value;

                        costmap.mapToWorld(mx+dcol, my+drow, wx, wy);         // TODO use bool return
                        A.row(i) << wx*wx, wy*wy, wx, wy, 1.;
                        ++i;
                }
        }

        // P times x (from A and b) = interpolated_cost
        // P is the following
        Eigen::RowVectorXd P(cols);
        P << px*px, py*py, px, py, 1.;

        // Using QR decomposition with column pivoting for computing x
        double interpolated_cost = P * A.colPivHouseholderQr().solve(b);
        // SVD decomposition for computing x
        // double interpolated_cost = P * A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        // std::cout << "--------\n" << "cost[4]: " << b[4] << "\ninterpolated_cost: " << interpolated_cost << "\n--------\n";
        return interpolated_cost;
}

};
