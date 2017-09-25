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

#ifndef CODRHA_COMM_H_
#define CODRHA_COMM_H_

#include <codrha_local_planner/trajectory.h>
#include <codrha_local_planner/common.h>
#include <codrha_local_planner/monocycle_flatoutput.h>
#include <unordered_set>
#include <map>

namespace codrha_local_planner {

struct IntendedTraj {
        struct mTime {
                long unsigned sec;
                long unsigned nsec;
        } stamp;
        std::string frame_id;
        double* ctrlPts;
        unsigned nCtrlPts;
        unsigned splDeg;
        double pvar;
        double time_to_exec;
        double radius;
};

class CODRHAComm {

public:
        /**
         * Publishes the robot's parametric trajectory so it can be read by other robots
         * @param traj the parametric trajectory
         * @param t0   reference time stamp
         */
        virtual void publishTrajectory(const Trajectory<double, MF::flatDim, MF::derivOrdForAccel+1>& traj,
                                        const double time_to_exec, const long unsigned wtsec, const long unsigned wtnsec, const double radius) = 0;


        // FIXME receive a list of strings
        virtual void updateRobotsToListenTo(const std::unordered_set<std::string>& robotIDs) = 0;

        virtual double getDist(const long unsigned& wtsec, const long unsigned& wtnsec, std::string robot_id) const = 0;

        virtual double getPlanPtDist(const double& x, const double& y, const long unsigned& wtsec, const long unsigned& wtnsec, std::string robot_id) const = 0;

        bool getIntendedTraj(std::string robot_id, IntendedTraj& others_traj)
        {
                if (others_traj_.find(robot_id) != others_traj_.end())
                {
                        others_traj = others_traj_[robot_id];
                        return true;
                }
                else
                {
                        return false;
                }
        };

        virtual void transform2DPointFromPlanRef2Odom(const double& x,
                                                const double& y,
                                                const long& wtsec,
                                                const long& wtnsec,
                                                double& x_out,
                                                double& y_out) const = 0;

        virtual void transform2DPointFromMapRef2Odom(const double& x,
                                                const double& y,
                                                const long& wtsec,
                                                const long& wtnsec,
                                                double& x_out,
                                                double& y_out) const = 0;

        virtual void transform2DPointFromPlanRef2Map(const double& x,
                                                const double& y,
                                                double& x_out,
                                                double& y_out) const = 0;

        virtual void transform2DPoseFromPlanRef2Odom(const double& x,
                                                const double& y,
                                                const double& theta,
                                                const long& wtsec,
                                                const long& wtnsec,
                                                double& x_out,
                                                double& y_out,
                                                double& theta_out) const = 0;

        virtual void init()
        {
                if (initialized_)
                {
                        std::stringstream ss;
                        ss << "initialization must be called only once per planning. ";
                        throw(common::MyException(ss.str()));
                }
                initialized_ = true;
        };

protected:
        std::map<std::string, IntendedTraj> others_traj_;
        bool initialized_;
};
};

#endif
