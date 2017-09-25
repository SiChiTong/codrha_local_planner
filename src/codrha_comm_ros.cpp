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

#include <codrha_local_planner/codrha_comm_ros.h>
#include <limits>
// #include <std_msgs/String.h>

namespace codrha_local_planner {

CODRHACommROS::CODRHACommROS(const std::string& name) :
        name_(name),
        pub_counter_(0)
{
        initialized_ = false;
        ros::NodeHandle private_nh("~/" + name);
        intended_traj_pub_ = private_nh.advertise<trajectory_msg>("intended_trajectory", 1);

        std::string aux(ros::this_node::getNamespace());
        tbnamespace_ = aux.substr(1, aux.length());
}

CODRHACommROS::~CODRHACommROS()
{
}
/**
 * publishes a trajectory in the map frame of ref. a reference world time and how much time from that ref instant the trajectory will be executed
 * @param traj         [description]
 * @param time_to_exec [description]
 * @param wtsec        [description]
 * @param wtnsec       [description]
 * @param radius       [description]
 */
void CODRHACommROS::publishTrajectory(const Trajectory<double, MF::flatDim, MF::derivOrdForAccel+1>& traj,
                                      const double time_to_exec, const long unsigned wtsec, const long unsigned wtnsec, const double radius)
{
        if (!initialized_)
        {
                ROS_ERROR("called CODRHACommROS::publishTrajectory before initialization. ");
        }

        trajectory_msg msg;

        msg.header.seq = pub_counter_++;
        msg.header.stamp.sec = wtsec;
        msg.header.stamp.nsec = wtnsec;

        msg.header.frame_id = "/map";

        msg.robot_id = ros::this_node::getNamespace(); // programmatically solution
        msg.robot_id = msg.robot_id.substr(2, msg.robot_id.length()); // from "//tbXX" to "tbXX"

        double * params = new double[traj.nParam()];
        traj.getParameters(params);

        double x, y;
        std::vector<double> tranformed_pts (params, params + traj.nParam());
        for (auto i = 0; i < traj.nCtrlPts(); ++i)
        {
                x = params[i*MF::flatDim];
                y = params[i*MF::flatDim + 1];

                tf::Point pt_in(x, y, 0.0);
                tf::Point pt_out(transform2map_ * pt_in);

                tranformed_pts[i*MF::flatDim] = pt_out.x();
                tranformed_pts[i*MF::flatDim + 1] = pt_out.y();
        }

        msg.ctrl_pts = tranformed_pts;

        msg.pvar = traj.getParVarInterval();
        msg.spl_deg = traj.splDegree();
        msg.time_to_exec = time_to_exec;
        msg.radius = radius;

        intended_traj_pub_.publish(msg);
}

void CODRHACommROS::updateRobotsToListenTo(const std::unordered_set<std::string>& robot_ids)
{
        if (!initialized_)
        {
                ROS_ERROR("called CODRHACommROS::updateRobotsToListenTo before initialization. ");
        }

        // for (auto&& [robot_id, subscriber]  : subscription_dict_) {
        for (auto it = subscription_dict_.begin(); it != subscription_dict_.end(); /* no increment */)
        {
                if (std::find(robot_ids.begin(), robot_ids.end(), it->first) == robot_ids.end())
                {
                        it->second.shutdown();
                        it = subscription_dict_.erase(it);
                }
                else
                {
                        ++it;
                }
        }

        ros::NodeHandle private_nh("~/" + name_);

        for (auto robot_id : robot_ids)
        {
                if (subscription_dict_.find(robot_id) == subscription_dict_.end())
                {
                        subscription_dict_[robot_id] = private_nh.subscribe("/" + robot_id + "/move_base/" + name_ + "/intended_trajectory", 1, &CODRHACommROS::updateTrajectoryCB, this);
                        ROS_INFO_STREAM("Sub to: /" << robot_id << "/move_base/" << name_ << "/intended_trajectory");
                }
        }
}

void CODRHACommROS::updateTrajectoryCB(const trajectory_msg::ConstPtr& msg)
{
        // It has to parse the trajectory msg and update date for the correct robot
        ROS_WARN_STREAM("Got info from : " << msg->robot_id);
        IntendedTraj a_msg;

        a_msg.stamp.sec = msg->header.stamp.sec;
        a_msg.stamp.nsec = msg->header.stamp.nsec;

        a_msg.ctrlPts = new double[msg->ctrl_pts.size()];
        std::copy(msg->ctrl_pts.begin(), msg->ctrl_pts.end(), a_msg.ctrlPts);

        a_msg.nCtrlPts = msg->ctrl_pts.size();

        a_msg.splDeg = msg->spl_deg;

        a_msg.pvar = msg->pvar;
        a_msg.time_to_exec = msg->time_to_exec;

        a_msg.frame_id = msg->header.frame_id;

        a_msg.radius = msg->radius;

        others_traj_[msg->robot_id] = a_msg;
}

double CODRHACommROS::getDist(const long unsigned& wtsec, const long unsigned& wtnsec, std::string robot_id) const
{
        if (!initialized_)
        {
                ROS_ERROR("called CODRHACommROS::getCurrentDist before initialization. ");
        }

        if (!tf_listener_.frameExists(robot_id + "_tf/base_footprint"))
        {
                return std::numeric_limits<double>::infinity();
        }

        ros::Time wt;
        wt.sec = wtsec;
        wt.nsec = wtnsec;

        tf::StampedTransform transform;

        try
        {
                tf_listener_.lookupTransform(tbnamespace_ + "_tf/base_footprint", robot_id + "_tf/base_footprint", wt, transform);
        }
        catch (tf::TransformException ex)
        {
                ROS_ERROR("%s",ex.what());
        }
        return transform.getOrigin().length();
}

double CODRHACommROS::getPlanPtDist(const double& x, const double& y, const long unsigned& wtsec, const long unsigned& wtnsec, std::string robot_id) const
{
        if (!initialized_)
        {
                ROS_ERROR("called CODRHACommROS::getPlanPtDist before initialization. ");
        }

        ros::Time wt;
        wt.sec = wtsec;
        wt.nsec = wtnsec;

        double x_odom, y_odom;
        transform2DPointFromPlanRef2Odom(x, y, wtsec, wtnsec, x_odom, y_odom);

        tf::StampedTransform transform;
        try
        {
                tf_listener_.lookupTransform(robot_id + "_tf/base_footprint", tbnamespace_ + "_tf/odom", wt, transform);
        }
        catch (tf::TransformException ex)
        {
                ROS_ERROR("%s",ex.what());
        }

        tf::Point pt_in(x, y, 0.0);

        // if (!tf_listener_.frameExists(robot_id + "_tf/base_footprint"))
        // {
        //         return std::numeric_limits<double>::infinity();
        // }

        tf::Point pt_inter(transform * pt_in);

        return pt_inter.length();
}

void CODRHACommROS::transform2DPoseFromPlanRef2Odom(const double& x,
                                                const double& y,
                                                const double& theta,
                                                const long& wtsec,
                                                const long& wtnsec,
                                                double& x_out,
                                                double& y_out,
                                                double& theta_out) const
{
        if (!initialized_)
        {
                ROS_ERROR("called CODRHACommROS::transform2DPointFromPlanRef2Odom before initialization. ");
        }

        // TODO
        double x_inter, y_inter, th_inter;
        transform2DPointFromPlanRef2Map(x, y, x_inter, y_inter);
        th_inter = theta + tf::getYaw(transform2map_.getRotation());


        // TODO
        // transform2DPoseFromMapRef2Odom
        {
                ros::Time current_time;
                current_time.sec = wtsec;
                current_time.nsec = wtnsec;

                tf::Point pt_in(x_inter, y_inter, 0.0);

                tf::StampedTransform transform2odom;
                try
                {
                        tf_listener_.lookupTransform(tbnamespace_ + "_tf/odom", "/map", current_time, transform2odom);
                }
                catch (tf::TransformException ex)
                {
                        ROS_ERROR("%s",ex.what());
                }

                tf::Point pt_out(transform2odom * pt_in);

                x_out = pt_out.x();
                y_out = pt_out.y();
                theta_out = th_inter - tf::getYaw(transform2odom.getRotation());
        }
}

void CODRHACommROS::transform2DPointFromPlanRef2Odom(const double& x,
                                                     const double& y,
                                                     const long& current_time_sec,
                                                     const long& current_time_nsec,
                                                     double& x_out,
                                                     double& y_out) const
{
        if (!initialized_)
        {
                ROS_ERROR("called CODRHACommROS::transform2DPointFromPlanRef2Odom before initialization. ");
        }

        double x_inter, y_inter;
        transform2DPointFromPlanRef2Map(x, y, x_inter, y_inter);

        transform2DPointFromMapRef2Odom(x_inter, y_inter, current_time_sec, current_time_nsec, x_out, y_out);
}

void CODRHACommROS::transform2DPointFromPlanRef2Map(const double& x,
                                     const double& y,
                                     double& x_out,
                                     double& y_out) const
{
        tf::Point pt_in(x, y, 0.0);
        tf::Point pt_out(transform2map_ * pt_in);
        x_out = pt_out.x();
        y_out = pt_out.y();
}

void CODRHACommROS::transform2DPointFromMapRef2Odom(const double& x,
                                                     const double& y,
                                                     const long& current_time_sec,
                                                     const long& current_time_nsec,
                                                     double& x_out,
                                                     double& y_out) const
{
        ros::Time current_time;
        current_time.sec = current_time_sec;
        current_time.nsec = current_time_nsec;

        tf::Point pt_in(x, y, 0.0);

        tf::StampedTransform transform2odom;
        try
        {
                tf_listener_.lookupTransform(tbnamespace_ + "_tf/odom", "/map", current_time, transform2odom);
        }
        catch (tf::TransformException ex)
        {
                ROS_ERROR("%s",ex.what());
        }

        tf::Point pt_out(transform2odom * pt_in);
        x_out = pt_out.x();
        y_out = pt_out.y();
}

void CODRHACommROS::init()
{
        if (initialized_)
        {
                ROS_ERROR("initialization must be called only once per planning. ");
        }

        tf_listener_.lookupTransform("/map", tbnamespace_ + "_tf/odom", ros::Time(0), transform2map_);

        initialized_ = true;
}

}
