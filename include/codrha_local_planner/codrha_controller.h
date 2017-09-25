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

#ifndef CODRHA_LOCAL_PLANNER_CONTROLLER_H_
#define CODRHA_LOCAL_PLANNER_CONTROLLER_H_

#include <codrha_local_planner/trajectory.h>
#include <codrha_local_planner/monocycle_flatoutput.h>
#include <codrha_local_planner/codrha_comm.h>

#define CTRL_DEBUG_ENABLED (1)

namespace codrha_local_planner
{

class Controller
{

private:
        // static const int traj_dim_ = 2; ///< @brief k dimensions
        // static const int traj_deriv_deg_ = 3; ///< @brief k+1 | traj is a C^k func

        static const unsigned relative_deg_of_non_lin_MIMO_sum = 9;         // equals to (relativeDegOfNonLinMIMO+1).sum(), but sum() is runtime (obviously!), I want it at compitation time

        static const unsigned state_dim = 5;
        static const unsigned observ_dim = 3;
        static const unsigned output_dim = 2;
        static const unsigned dy_mod_params_dim = 6;
        typedef Eigen::Matrix< double, state_dim, 1 > StatVector;
        typedef Eigen::Matrix< double, observ_dim, 1 > ObservVector;
        typedef Eigen::Matrix< double, output_dim, 1 > OutputVector;
        typedef Eigen::Matrix< double, dy_mod_params_dim, 1 > DyModParamVector;

        DyModParamVector dy_mod_params_;

        double prediction_time_;
        double integration_time_step_;

        Eigen::Matrix<double, observ_dim, relative_deg_of_non_lin_MIMO_sum> Ks_;
        Eigen::Matrix<double, observ_dim, observ_dim> KssInv_;

        double max_lin_vel_, max_ang_vel_;

        CODRHAComm* comm_link_;


public:

        std::string name_;

        // typedef Trajectory<double, traj_dim_, traj_deriv_deg_> Trajectory2DC2;
        /**
         * @brief  Constructor for the controller
         * @param name The name of the controller
         */
        Controller(std::string name, CODRHAComm* comm_link);

        /**
         * @brief  Destructor for the planner
         */
        ~Controller();

        /**
         * @brief Reconfigures the controller
         */
        void reconfigure (double, double, double, double, std::vector<double> & cfg_dy_mod_params);

        template<const unsigned t_dim, const unsigned t_degree>
        bool computeVelocityCommands(
                double wtsec, double wtnsec,
                double x_feedback, double y_feedback, double psi_feedback,
                double v_feedback, double omega_feedback,
                // const Eigen::DenseBase<Derived>& trajectory,
                const Trajectory<double, t_dim, t_degree> & trajectory,
                // const Trajectory<double, MonocycleFlatoutput::flatDim, MonocycleFlatoutput::derivOrdForAccel> & trajectory,
                double eval_time, double plan_time, unsigned plan_stage,
                double & lin_vel_output, double & ang_vel_output);

};

template<const unsigned t_dim, const unsigned t_degree>
bool Controller::computeVelocityCommands(
        double wtsec, double wtnsec,
        double x_feedback, double y_feedback, double psi_feedback,
        double v_feedback, double omega_feedback,
        // const Eigen::DenseBase<Derived>& trajectory,
        const Trajectory<double, t_dim, t_degree> & trajectory,
        // const Trajectory<double, MF::flatDim, MF::derivOrdForAccel> & trajectory,
        double eval_time, double plan_time, unsigned plan_stage,
        double & lin_vel_output, double & ang_vel_output)
{
        typedef MonocycleFlatoutput MF;
        double u = v_feedback;
        double w = omega_feedback;
        double x = x_feedback;
        double y = y_feedback;
        double theta = psi_feedback;

        // Compute information from trajectory
        // MF::VeloVectorD velocity(MF::flatToVelocity<double, MF::derivOrdForAccel+1>(deriv_flat));
        // MF::AccelVectorD acceleration(MF::flatToAcceleration<double, MF::derivOrdForAccel+1>(deriv_flat));

        // double xdot_ref = lin_vel_ref * cos(theta_ref);
        // double ydot_ref = lin_vel_ref * sin(theta_ref);
        // double thetadot_ref = ang_vel_ref;
        // double xdotdot_ref = lin_acc_ref * cos(theta_ref);
        // double ydotdot_ref = lin_acc_ref * sin(theta_ref);
        // double thetadotdot_ref = ang_acc_ref;
        MF::PoseVectorD Rs;
        MF::PoseVectorD pose_at_eval_time;


        //
        if (plan_stage == 0)
        {
                lin_vel_output = 0.0;
                ang_vel_output = 0.0;
                return true;
        }
        // else if (plan_stage == -2)
        // // else if (plan_stage == 3 || plan_stage == 2)
        // {
        //         std::stringstream ss;
        //         ss << "Controller::computeVelocityCommands: implementation of stage 3 needed. ";
        //         // throw(common::MyException(ss.str()));
        //         // if (abs(common::wrapToPi(theta-theta_ref)) > 30.*M_PI/180.)
        //         // {
        //         lin_vel_output = 0.0;
        //         ang_vel_output = 0.0;
        //         return true;
        //         // }
        //         // Rs = Ks_* (Eigen::Matrix < double, relative_deg_of_non_lin_MIMO_sum, 1>() <<
        //         //      x_ref,
        //         //      xdot_ref,
        //         //      xdotdot_ref,
        //         //      y_ref,
        //         //      ydot_ref,
        //         //      ydotdot_ref,
        //         //      0.0,
        //         //      thetadot_ref,
        //         //      thetadotdot_ref).finished();
        // }
        else
        {
                double prediction_time = prediction_time_;

                if (plan_stage == 2 && prediction_time_ > plan_time - eval_time)
                {
                        // #if (CTRL_DEBUG_ENABLED)
                        //
                        // std::cout << FG_BLUE << "[" << eval_time << "] " << "(*)controller:" << FG_YELLOW << " plan stage " << plan_stage << RESET << std::endl;
                        //
                        // #endif
                        //
                        // MF::VeloVectorD velocity(MF::flatToVelocity(trajectory(eval_time, MF::derivOrdForVelo)));
                        //
                        // lin_vel_output = velocity[0];
                        // ang_vel_output = velocity[1];
                        // return true;

                        prediction_time = plan_time - eval_time;
                        // update Gain Matrix
                        KssInv_ << 20./std::pow(prediction_time, 5), 0, 0,
                                           0, 20./std::pow(prediction_time, 5), 0,
                                           0, 0, 20./std::pow(prediction_time*1., 5);

                        Ks_ << std::pow(prediction_time, 3)/6.,std::pow(prediction_time, 4)/8.,std::pow(prediction_time, 5)/20.,0,0,0,0,0,0,
                                0,0,0,std::pow(prediction_time,3)/6.,std::pow(prediction_time,4)/8.,std::pow(prediction_time,5)/20.,0,0,0,
                                0,0,0,0,0,0,std::pow(prediction_time*1., 3)/6.,std::pow(prediction_time*1., 4)/8.,std::pow(prediction_time*1., 5)/20.;
                }

                pose_at_eval_time = MF::flatToPose(trajectory(eval_time, MF::derivOrdForPose));

                double px, py, ptheta;
                // comm_link_->transform2DPointFromPlanRef2Odom(pose_from_traj.x(), pose_from_traj.y(), wtsec, wtnsec, px, py);
                comm_link_->transform2DPoseFromPlanRef2Odom(pose_at_eval_time.x(), pose_at_eval_time.y(), pose_at_eval_time.z(), wtsec, wtnsec, px, py, ptheta);

                pose_at_eval_time << px, py, ptheta;

                int samples = int(prediction_time/integration_time_step_);

                Rs = MF::PoseVectorD::Zero();
                MF::PoseVectorD previous_pose_tau_sq = MF::PoseVectorD::Zero();
                MF::PoseVectorD pose_from_traj;

                // Linearized solution:
                // Rs = Ks_* (Eigen::Matrix < double, 9, 1>() <<
                //                 x_ref,
                //                 xdot_ref,
                //                 xdotdot_ref,
                //                 y_ref,
                //                 ydot_ref,
                //                 ydotdot_ref,
                //                 theta_ref,
                //                 thetadot_ref,
                //                 thetadotdot_ref).finished();

                for (int i = 1; i <= samples; ++i)
                {
                        double tau = double(i)/samples * prediction_time;

                        if (eval_time+tau <= plan_time)
                        {
                                // MF::MatForPoseD derivFlat();
                                pose_from_traj = MF::flatToPose(
                                        trajectory(eval_time+tau, MF::derivOrdForPose));

                                comm_link_->transform2DPoseFromPlanRef2Odom(pose_from_traj.x(), pose_from_traj.y(), pose_from_traj.z(), wtsec, wtnsec, px, py, ptheta);
                                pose_from_traj << px, py, ptheta;

                                // pose_from_traj(0,0) += .1;
                                MF::yawInPose(pose_from_traj) <<
                                        common::wrapToPi(MF::yawInPose(pose_from_traj) - MF::yawInPose(pose_at_eval_time));
                        }
                        else
                        {
                                std::cout << "eval_time: " << eval_time << "tau: " << tau << "plan_time: " << plan_time << "\neval_time+tau <= plan_time" <<std::endl;
                                // TODO
                                std::stringstream ss;
                                ss << "Controller::computeVelocityCommands: implementation of case were evaluation time is equal to or bigger than the sim time";
                                throw(common::MyException(ss.str()));
                                // pose_from_traj = _targetedPose;
                                // pose_from_traj.tail<1>()(0,0) = common::wrapToPi(pose_from_traj.tail<1>()(0,0) - theta_ref);
                        }


                        Rs += (pose_from_traj*tau*tau + previous_pose_tau_sq)/4. * prediction_time/samples;

                        previous_pose_tau_sq = pose_from_traj*tau*tau;
                }
        }

        double aux = dy_mod_params_[2]/dy_mod_params_[0]*w*w - dy_mod_params_[3]/dy_mod_params_[0]*u;
        double L2fy3 = -dy_mod_params_[4]/dy_mod_params_[1]*u*w - dy_mod_params_[5]/dy_mod_params_[1]*w;
        double L2fy2 = sin(theta) * aux + u*w*cos(theta);
        double L2fy1 = cos(theta) * aux - u*w*sin(theta);
        // double Lfy4 = dy_mod_params_[2]/dy_mod_params_[0]*w*w - dy_mod_params_[3]/dy_mod_params_[0]*u;
        // double Lfy5 = -dy_mod_params_[4]/dy_mod_params_[1]*u*w - dy_mod_params_[5]/dy_mod_params_[1]*w;

        Eigen::Matrix < double, relative_deg_of_non_lin_MIMO_sum, 1> Ly =
                (Eigen::Matrix < double, relative_deg_of_non_lin_MIMO_sum, 1>() <<
                        x,
                        u*cos(theta),
                        L2fy1,
                        y,
                        u*sin(theta),
                        L2fy2,
                        // 0,
                        common::wrapScalarToPi(theta - MF::yawInPose(pose_at_eval_time)(0,0)),
                        // theta,
                        w,
                        L2fy3
                ).finished();

        Eigen::Matrix < double, output_dim, observ_dim> DtDInvDt =
                (Eigen::Matrix < double, output_dim, observ_dim>() <<
                        cos(theta)*dy_mod_params_[0], sin(theta)*dy_mod_params_[0], 0,
                        0, 0, dy_mod_params_[1]).finished();

        Eigen::Vector2d cntrlOut = -DtDInvDt*KssInv_*(Ks_*Ly - Rs);

        // double a = ydot_ref;
        // double b = -xdot_ref;
        // double c = xdot_ref * y_ref - ydot_ref * x_ref;
        //
        // double signedDist = (a*x + b*y + c) * common::finvsqrt(a*a + b*b);

        lin_vel_output = cntrlOut(0,0);
        ang_vel_output = cntrlOut(1,0);

        // ang_vel_output = abs(common::wrapToPi(theta-theta_ref)) > 1*M_PI/180. ? cntrlOut(1,0) : cntrlOut(1,0) + cntrlOut(1,0)*.1*common::sgn(signedDist);

        // std::cout << "---------- ctr LOGGING ----------" << std::endl;
        // std::cout << "ctr::DtDInvDt\n" << DtDInvDt << std::endl;
        // std::cout << "ctr::KssInv\n" << KssInv_ << std::endl;
        // std::cout << "ctr::Ks\n" << Ks_ << std::endl;
        // std::cout << "ctr::Ly\n" << Ly << std::endl;
        // std::cout << "ctr::Rs\n" << Rs << std::endl;
        // std::cout << "ctr::E\n" << Ks_*Ly - Rs << std::endl;
        //
        // std::cout << "ctr::lin_vel_output " << lin_vel_output << std::endl;
        // std::cout << "ctr::ang_vel_output " << ang_vel_output << std::endl;
        // std::cin.get();
        // std::cout << std::endl;

        // limiting output
        // lin_vel_output = std::max( std::min( lin_vel_output, max_lin_vel_ ), -1*max_lin_vel_ );
        lin_vel_output = std::max( std::min( lin_vel_output, 1.2 ), -1*1.2 );
        ang_vel_output = std::max( std::min( ang_vel_output, 10. ), -1*10. );

        return true;
}

}
#endif
