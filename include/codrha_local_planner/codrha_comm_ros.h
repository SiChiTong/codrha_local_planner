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


#include <ros/ros.h>
#include <codrha_local_planner/codrha_comm.h>
#include <codrha_local_planner/trajectory_msg.h>
#include <tf/transform_listener.h>

#ifndef CODRHA_COMM_ROS_H_
#define CODRHA_COMM_ROS_H_

namespace codrha_local_planner {

class CODRHACommROS: public CODRHAComm {

public:

        CODRHACommROS(const std::string& name);

        // void initialize();

        ~CODRHACommROS();

        void publishTrajectory(const Trajectory<double, MF::flatDim, MF::derivOrdForAccel+1>& traj,
                                const double time_to_exec, const long unsigned wtsec, const long unsigned wtnsec, const double radius);

        /**
         * Manage subscriptions
         * @param robot_id [description]
         */
        void updateRobotsToListenTo(const std::unordered_set<std::string>& robot_ids);
        // void updateRobotsToListenTo(const std::string& robot_id);

        /**
         * Callback updating trajectories upon the arrival of a new trajectory_msg
         * @param msg the trajectory message
         */
        void updateTrajectoryCB(const trajectory_msg::ConstPtr& msg);
        // FIXME receive a list of strings

        /**
         * [getCurrentDist description]
         * @param  robot_id [description]
         * @return          [description]
         */
        double getDist(const long unsigned& wtsec, const long unsigned& wtnsec, std::string robot_id) const;

        double getPlanPtDist(const double& x, const double& y, const long unsigned& wtsec, const long unsigned& wtnsec, std::string robot_id) const;


        void transform2DPointFromPlanRef2Odom(const double& x,
                                             const double& y,
                                             const long& wtsec,
                                             const long& wtnsec,
                                             double& x_out,
                                             double& y_out) const;

        void transform2DPointFromMapRef2Odom(const double& x,
                                             const double& y,
                                             const long& wtsec,
                                             const long& wtnsec,
                                             double& x_out,
                                             double& y_out) const;

        void transform2DPointFromPlanRef2Map(const double& x,
                                                  const double& y,
                                                  double& x_out,
                                                  double& y_out) const;

        void transform2DPoseFromPlanRef2Odom(const double& x,
                                        const double& y,
                                        const double& theta,
                                        const long& wtsec,
                                        const long& wtnsec,
                                        double& x_out,
                                        double& y_out,
                                        double& theta_out) const;

        void init();

        // bool isInitialized() { return initialized_; }

private:

        // for multi-robot planning, publisher of intended trajectory
        std::string name_;
        ros::Publisher intended_traj_pub_;
        std::map<std::string, ros::Subscriber> subscription_dict_;
        unsigned pub_counter_;
        tf::TransformListener tf_listener_;
        std::string tbnamespace_;
        tf::StampedTransform transform2map_;
};
};
#endif
