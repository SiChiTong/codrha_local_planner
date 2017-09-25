// #include <Eigen/Core>
#include <codrha_local_planner/codrha_planner.h>
#include <codrha_local_planner/common.h>
#include <iostream>
#include <iomanip>
#include <boost/current_function.hpp>
#include <sys/time.h> /* gethrtime(), gettimeofday() */

using namespace codrha_local_planner;

int main()
{
        double lin_vel_output, ang_vel_output;


        double dy_mod[] = {0.04200441, 0.27468742, -0.01248822, 1.00119437,
                           0.00545974, 1.03107639};
        const double plan_granularity = .1;
        const double time_step = 0.0001;
        const int n_ctrl_pts = 8;
        const std::string opt_method = "SLSQP";
        // const double opt_objective_func_rel_tol = 1e-6;
        const double opt_objective_func_rel_tol = 0;
        // const double opt_param_rel_tol = 1e-6;
        const double opt_param_rel_tol = 0;

        const double comp_time = 0.5;
        const double plan_time = 1.5;
        // const double num_dif_eps = std::numeric_limits< double >::epsilon();
        const double num_dif_eps = 1e-9;
        // const double opt_objective_func_abs_tol = 0; // disable
        const double opt_objective_func_abs_tol = 0;
        const double opt_param_abs_tol = 0; // disable
        // const double opt_param_abs_tol = 1e-40;
        // const double opt_equetions_abs_tol = 0;
        const double opt_equetions_abs_tol = 0;
        // const double opt_inequetions_tol_abs_tol = 0;
        const double opt_inequetions_tol_abs_tol =  0;

        const double max_vel_x = 1.0;
        const double max_vel_theta = 12.0;
        const double acc_lim_x = 1e20;
        const double acc_lim_theta = 1e20;
        // const double acc_lim_x = 0.8;
        // const double acc_lim_theta = 2.0;
        const double timeout_first_plan = 2.5;
        const double robot_obst_safety_dist = .1;
        const double radius = 0.5;
        MF::PoseVectorD target_pt_pose;
        target_pt_pose << 14., 14., 0.;
        MF::VeloVectorD target_pt_velocity;
        target_pt_velocity << .0, .0;
        const double last_step_min_dist = .1;
        double first_guess_tweak = 0.2;


        struct timeval tv;

        // toc = common::getRealTime<double>()*1e-6;
        // planner->computeVelocityCommands(toc, lin_vel_output, ang_vel_output);
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Velocities: " << lin_vel_output << ", " << ang_vel_output << RESET << std::endl;
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC-TIC: " << toc - tic << RESET << std::endl;

        gettimeofday(&tv,NULL);
        double tic_sec = double(tv.tv_sec);
        double tic_usec = double(tv.tv_usec);
        // (sec2 - sec1)*1e6 + (usec2 - usec1)
        double toc_sec = tic_sec;
        double toc_usec = tic_usec;

        std::stringstream name;
        name << "v_planner";

        CODRHAPlanner *planner = new CODRHAPlanner(name.str());

        planner->reconfigure(
                plan_time, plan_granularity, comp_time,
                n_ctrl_pts, opt_method, num_dif_eps,
                opt_objective_func_abs_tol, opt_objective_func_rel_tol,
                opt_param_abs_tol, opt_param_rel_tol,
                opt_equetions_abs_tol, opt_inequetions_tol_abs_tol,
                max_vel_x, max_vel_theta, acc_lim_x,
                acc_lim_theta, timeout_first_plan,
                robot_obst_safety_dist, radius, target_pt_pose, target_pt_velocity,
                last_step_min_dist, first_guess_tweak);


        double x[] = {6.94702e-310, 6.94702e-310, 6.95702e-310, 6.95702e-310, 0.0370522, -0.000640767, 0.146438, 0.0305818, 0.254304, 0.0814644, 0.423453, 0.200753, 0.579686, 0.376099, 0.599686, 0.476099};
        // double *grad = new double[n];
        double * gradObjF = new double[n_ctrl_pts*2];

        double cost = CODRHAPlanner::objectiveFunc(n_ctrl_pts*2,
                             x,
                             gradObjF,
                             planner);
        MF::PositionVectorD last_cp;
        last_cp << x[2*n_ctrl_pts-2] , x[2*n_ctrl_pts-1];

        if (cost == pow((last_cp - target_pt_pose.block<2,1>(0,0)).norm(), 2))
        {
                std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Cost: " << cost << RESET << std::endl;
        }
        else
        {
                std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Wrong cost! " << cost << RESET << std::endl;
        }
        for (int i = 0; i < n_ctrl_pts*2; ++i)
        {
                std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "GObj[" << i << "]: " << gradObjF[i] << RESET << std::endl;
        }

        int n_samples = int(plan_time/plan_granularity);
        int n_equations = 3 + 2;
        int n_inequations = 3 * n_samples;

        double * gradEq = new double[n_ctrl_pts*2*n_equations];
        // double * eq = new double[n_equations];
        double * result = new double[n_equations];
        CODRHAPlanner::equationConstraints(n_equations,
                                 result,
                                 n_ctrl_pts*2,
                                 x,
                                 gradEq,
                                 planner);
        for (int i = 0; i < n_equations; ++i)
        {
                std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "eq: " << result[i] << RESET << std::endl;
        }

        std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "GEq:" << RESET << std::endl;

        // for (int i = 0; i < n_ctrl_pts*2; ++i)
        // {
        //         for (int j =0; j < n_equations; ++j)
        //         {
        //                 std::cout << std::setw(15) << std::left << gradEq[j*n_ctrl_pts*2 + i] << " ";
        //         }
        //         std::cout << std::endl;
        // }
        for (int j =0; j < n_equations; ++j)
        {
                for (int i = 0; i < n_ctrl_pts*2; ++i)
                {
                        std::cout << std::setw(12) << std::left << gradEq[j*n_ctrl_pts*2 + i] << " ";
                }
                std::cout << std::endl;
        }

        std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "End Grad Eq" << RESET << std::endl;

        double * gradInEq = new double[n_ctrl_pts*2*n_inequations];
        // double * eq = new double[n_equations];
        double * resultIn = new double[n_inequations];
        std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "End Alloc" << RESET << std::endl;
        CODRHAPlanner::inequationConstraints(n_inequations,
                                 resultIn,
                                 n_ctrl_pts*2,
                                 x,
                                 gradInEq,
                                 planner);

        std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "End Ineq Constr Call" << RESET << std::endl;

        for (int i = 0; i < n_inequations; ++i)
        {
                std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "ineq: " << resultIn[i] << RESET << std::endl;
        }
        // for (int i = 0; i < n_ctrl_pts*2*n_inequations; ++i)
        // {
        //         std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "GInEq[" << i << "]: " << gradInEq[i] << RESET << std::endl;
        // }
        for (int j =0; j < n_inequations; ++j)
        {
                for (int i = 0; i < n_ctrl_pts*2; ++i)
                {
                        std::cout << std::setw(13) << std::left << gradInEq[j*n_ctrl_pts*2 + i] << " ";
                }
                std::cout << std::endl;
        }
                //
                // static void inequationConstraints(unsigned m,
                //                            double *result,
                //                            unsigned n,
                //                            const double* x,
                //                            double* grad,
                //                            void* data);
                //
                // static double objectiveFuncTermination(unsigned n,
                //                                 const double *x,
                //                                 double *grad,
                //                                 void *data);
                //
                // static void equationConstraintsTermination(unsigned m,
                //                                     double *result,
                //                                     unsigned n,
                //                                     const double* x,
                //                                     double* grad,
                //                                     void* data);
                //
                // static void inequationConstraintsTermination(unsigned m,
                //                                       double *result,
                //                                       unsigned n,
                //                                       const double* x,
                //                                       double* grad,
                //                                       void* data);

        // planner->computeVelocityCommands(toc_sec + toc_usec*1e-6, lin_vel_output, ang_vel_output);
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC-TIC: \t\t\t" << (toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) << RESET << std::endl;
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TIC: " << tic << RESET << std::endl;
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC: " << toc << RESET << std::endl;
        planner->~CODRHAPlanner();

        // while ((toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) < 8*1e6)
        // {
        //
        //         planner->computeVelocityCommands(toc_sec + toc_usec*1e-6, lin_vel_output, ang_vel_output);
        //         // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Velocities: " << lin_vel_output << ", " << ang_vel_output << RESET << std::endl;
        //         // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC-TIC: \t\t\t" << (toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) << RESET << std::endl;
        //         // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TIC: " << tic << RESET << std::endl;
        //         // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC: " << toc << RESET << std::endl;
        //         gettimeofday(&tv,NULL);
        //         toc_sec = double(tv.tv_sec);
        //         toc_usec = double(tv.tv_usec);
        // }

        return 0;
}
