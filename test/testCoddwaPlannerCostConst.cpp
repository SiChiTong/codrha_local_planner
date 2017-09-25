// #include <Eigen/Core>
#include <codrha_local_planner/codrha_planner.h>
#include <codrha_local_planner/common.h>
#include <iostream>
#include <boost/current_function.hpp>
#include <sys/time.h> /* gethrtime(), gettimeofday() */

using namespace codrha_local_planner;

int main()
{

        const double total_sim_time = 10; // in seconds

        double lin_vel_output, ang_vel_output;


        double dy_mod[] = {0.04200441, 0.27468742, -0.01248822, 1.00119437,
                           0.00545974, 1.03107639};
        const int n_ctrl_pts = 8;
        const double plan_granularity = .1;
        const double time_step = 0.0001;
        const std::string opt_method = "SLSQP";
        // const std::string opt_method = "NONE";
        // const double opt_objective_func_rel_tol = 1e-6;
        const double opt_objective_func_rel_tol = 0;
        // const double opt_param_rel_tol = 1e-6;
        const double opt_param_rel_tol = 0;

        const double comp_time = 0.6;
        const double plan_time = 1.51;
        const double num_dif_eps = std::numeric_limits< double >::epsilon();
        // const double opt_objective_func_abs_tol = 0; // disabled
        const double opt_objective_func_abs_tol = 5e-5;
        const double opt_param_abs_tol = 0; // disabled
        // const double opt_param_abs_tol = 1e-12;
        const double opt_equetions_abs_tol = 0; // disabled
        // const double opt_equetions_abs_tol = 1e-12;
        const double opt_inequetions_tol_abs_tol = 0; // disabled
        // const double opt_inequetions_tol_abs_tol =  1e-12;

        const double max_vel_x = 1.0;
        const double max_vel_theta = 12.0;
        const double acc_lim_x = .8;
        const double acc_lim_theta = 2.;
        // const double acc_lim_x = 0.8;
        // const double acc_lim_theta = 2.0;
        const double timeout_first_plan = 1.5;
        const double robot_obst_safety_dist = .00;
        const double radius = 0.5;
        MF::PoseVectorD target_pt_pose;
        target_pt_pose << 7., 7., 0.;
        MF::VeloVectorD target_pt_velocity;
        target_pt_velocity << .0, .0;
        const double last_step_min_dist = .1;
        double first_guess_tweak = 0.02;


        struct timeval tv;

        // toc = common::getRealTime<double>()*1e-6;
        // planner->computeVelocityCommands(toc, lin_vel_output, ang_vel_output);
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Velocities: " << lin_vel_output << ", " << ang_vel_output << RESET << std::endl;
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC-TIC: " << toc - tic << RESET << std::endl;

        // for (unsigned expon = 0; expon < 1; ++expon)
        // for (unsigned expon = 7; expon < 8; ++expon)
        // for (unsigned expon = 4; expon < 12; ++expon)
        std::vector<double> const_values = {1.1, 1.2, 1.3, 1.5, 2.0, 3.0, 4.0, 6.0, 9.0, 13.0, 16.0, 20., 25., 40., 100., 200., 1000.};

        for (unsigned expon = 2; expon < 9; ++expon)
        // for (unsigned expon = 11; expon < 12; ++expon)
        {
                for (auto el : const_values)
                {
                        gettimeofday(&tv,NULL);
                        double tic_sec = double(tv.tv_sec);
                        double tic_usec = double(tv.tv_usec);
                        // (sec2 - sec1)*1e6 + (usec2 - usec1)
                        double toc_sec = tic_sec;
                        double toc_usec = tic_usec;


                        std::stringstream name;
                        name << "v" << expon << "_" << el << "_planner";

                        CODRHAPlanner *planner = new CODRHAPlanner(name.str());

                        planner->reconfigure(
                                plan_time, plan_granularity, comp_time,
                                n_ctrl_pts, opt_method, num_dif_eps*pow(10, expon),
                                // n_ctrl_pts, opt_method, num_dif_eps*(expon+1),
                                opt_objective_func_abs_tol, opt_objective_func_rel_tol,
                                opt_param_abs_tol, opt_param_rel_tol,
                                opt_equetions_abs_tol, opt_inequetions_tol_abs_tol,
                                max_vel_x, max_vel_theta, acc_lim_x,
                                acc_lim_theta, timeout_first_plan,
                                robot_obst_safety_dist, radius, target_pt_pose, target_pt_velocity,
                                last_step_min_dist, first_guess_tweak, el);

                        while ((toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) < total_sim_time*1e6)
                        // while ((toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) < 12*1e6)
                        {

                                planner->computeVelocityCommands(toc_sec + toc_usec*1e-6, lin_vel_output, ang_vel_output);
                                // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Velocities: " << lin_vel_output << ", " << ang_vel_output << RESET << std::endl;
                                // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC-TIC: \t\t\t" << (toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) << RESET << std::endl;
                                // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TIC: " << tic << RESET << std::endl;
                                // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC: " << toc << RESET << std::endl;
                                gettimeofday(&tv,NULL);
                                toc_sec = double(tv.tv_sec);
                                toc_usec = double(tv.tv_usec);
                        }
                        planner->~CODRHAPlanner();
                }
        }

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
