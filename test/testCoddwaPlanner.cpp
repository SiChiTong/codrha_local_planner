// #include <Eigen/Core>
#include <codrha_local_planner/codrha_planner.h>
#include <codrha_local_planner/common.h>
#include <iostream>
#include <boost/current_function.hpp>
#include <sys/time.h> /* gethrtime(), gettimeofday() */
#include <boost/thread/thread.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

using namespace codrha_local_planner;

int main()
{

        const double num_dif_eps = std::numeric_limits< double >::epsilon();

        double lin_vel_output, ang_vel_output;


        double aux[] = {0.09846696, 0.21094797, 0.03676359, 1.16053614, -0.82994671, 1.43638553};
        std::vector<double> dy_mod(aux, aux + sizeof(aux) / sizeof(double));

        const int n_ctrl_pts = 8;
        const double plan_granularity = .1; // every .1 s will check the constraints
        // const double time_step = 0.0001;
        const std::string opt_method = "SLSQP";
        // const std::string opt_method = "NONE";
        // const double opt_objective_func_rel_tol = 1e-6;
        const double opt_objective_func_rel_tol = 0;
        // const double opt_param_rel_tol = 1e-6;
        const double opt_param_rel_tol = 0;

        const double max_vel_x = .5;
        const double max_vel_theta = 3.14;

        const double acc_sup_x_init = 0.5;
        const double acc_inf_x_init = -0.5;
        const double acc_lim_x = 0.5;
        // const double acc_lim_x = 3.0;
        const double acc_lim_theta = 14;
        // const double acc_lim_theta = 6.;

        const double timeout_first_plan = 1.5;
        const double robot_obst_safety_dist = .00;
        const double radius = 0.2;
        MF::PoseVectorD init_pt_pose;
        init_pt_pose << 0.0, 0.0, 0.0;
        // init_pt_pose << -0.0057924, -0.00936754, 0.0521853;
        MF::VeloVectorD init_pt_velocity;
        init_pt_velocity << .001, .0;
        MF::PoseVectorD target_pt_pose;
        target_pt_pose << 2.1, 0.0, 0.0;
        // target_pt_pose << 2.69218,  -0.032651, -0.0287516;
        // target_pt_pose << 2.23964, -0.00857861, 0.0200369;
        MF::VeloVectorD target_pt_velocity;
        target_pt_velocity << .001, .0; // final linear velocity should'nt be zero to avoid division by ~0 causing high angular velocity (.001 m/s)
        const double last_step_min_dist = .0;
        double first_guess_tweak = 1.01;
        double vel_for_init_factor = 0.99;

        // IMPORTANT PARAMETERS
        const double comp_time = 0.7;
        const double plan_time = 1.3;
        // const double opt_objective_func_abs_tol = 0; // disabled
        const double opt_objective_func_abs_tol = 1e-9;
        const double opt_param_abs_tol = 0; // disabled
        // const double opt_param_abs_tol = 1e-12;
        const double opt_equetions_abs_tol = 0; // disabled
        // const double opt_equetions_abs_tol = 1e-12;
        const double opt_inequetions_tol_abs_tol = 0; // disabled
        // const double opt_inequetions_tol_abs_tol =  1e-12;
        double cost_tweak = 2.; // function of robots size?
        double eps_multip_inf = 5;
        double eps_multip_sup = 6;
        double total_sim_time = 2.4 * (MF::positionInPose(init_pt_pose) - MF::positionInPose(target_pt_pose)).norm()/max_vel_x;


        double ctrl_prediction_time = .3;
        double ctrl_integ_step = .001;

        struct timeval tv;

        // toc = common::getRealTime<double>()*1e-6;
        // planner->computeVelocityCommands(toc, lin_vel_output, ang_vel_output);
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Velocities: " << lin_vel_output << ", " << ang_vel_output << RESET << std::endl;
        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC-TIC: " << toc - tic << RESET << std::endl;

        // for (unsigned expon = 0; expon < 1; ++expon)
        // for (unsigned expon = 7; expon < 8; ++expon)
        // for (unsigned expon = 4; expon < 12; ++expon)
        for (unsigned expon = eps_multip_inf; expon < eps_multip_sup; ++expon)
        // for (unsigned expon = 11; expon < 12; ++expon)
        {
                gettimeofday(&tv,NULL);
                double tic_sec = double(tv.tv_sec);
                double tic_usec = double(tv.tv_usec);
                // (sec2 - sec1)*1e6 + (usec2 - usec1)
                double toc_sec = tic_sec;
                double toc_usec = tic_usec;

                std::stringstream name;
                name << "v" << expon << "_planner";

                // CODRHAPlanner *planner = new CODRHAPlanner(name.str());
                // tf::TransformListener fake_tf_listener;
                CODRHAPlanner planner(name.str());

                // planner->reconfigure(
                planner.reconfigure(
                        plan_time, plan_granularity, comp_time,
                        n_ctrl_pts, opt_method, num_dif_eps*pow(10, expon),
                        // n_ctrl_pts, opt_method, num_dif_eps*(expon+1),
                        opt_objective_func_abs_tol, opt_objective_func_rel_tol,
                        opt_param_abs_tol, opt_param_rel_tol,
                        opt_equetions_abs_tol, opt_inequetions_tol_abs_tol,
                        max_vel_x, max_vel_theta, acc_lim_x,
                        acc_lim_theta, acc_sup_x_init, acc_inf_x_init, timeout_first_plan,
                        robot_obst_safety_dist, radius,
                        last_step_min_dist, first_guess_tweak, cost_tweak, vel_for_init_factor, dy_mod,
                        ctrl_prediction_time, ctrl_integ_step, true, true, true, true);

                while ((toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) < total_sim_time*1e6)
                // while ((toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) < 12*1e6)
                {

                        // planner.computeVelocityCommands(
                        planner.computeVelocityCommands(
                                toc_sec + toc_usec*1e-6,
                                init_pt_pose[0],
                                init_pt_pose[1],
                                init_pt_pose[2],
                                target_pt_pose[0],
                                target_pt_pose[1],
                                target_pt_pose[2],
                                target_pt_velocity[0],
                                target_pt_velocity[1],
                                0.0,
                                0.0,
                                lin_vel_output,
                                ang_vel_output);
                        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "Velocities: " << lin_vel_output << ", " << ang_vel_output << RESET << std::endl;
                        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC-TIC: \t\t\t" << (toc_sec - tic_sec)*1e6 + (toc_usec - tic_usec) << RESET << std::endl;
                        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TIC: " << tic << RESET << std::endl;
                        // std::cout << FG_BLUE << BOOST_CURRENT_FUNCTION << ":" << FG_YELLOW << "TOC: " << toc << RESET << std::endl;
                        boost::this_thread::sleep( boost::posix_time::milliseconds(50) );

                        gettimeofday(&tv,NULL);
                        toc_sec = double(tv.tv_sec);
                        toc_usec = double(tv.tv_usec);
                }
                // delete planner;
                // planner->~CODRHAPlanner();
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
