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

#include <codrha_local_planner/codrha_controller.h>

namespace codrha_local_planner
{

typedef MonocycleFlatoutput MF;

Controller::Controller(std::string name, CODRHAComm* comm_link) :
        name_(name),
        comm_link_(comm_link) {
}

Controller::~Controller(){
}

void Controller::reconfigure (double cfg_prediction_time,
                              double cfg_max_lin_vel, double cfg_max_ang_vel,
                              double cfg_integration_time_step,
                              std::vector<double> & cfg_dy_mod_params)
{
        prediction_time_ = cfg_prediction_time;
        max_lin_vel_ = cfg_max_lin_vel;
        max_ang_vel_ = cfg_max_ang_vel;

        KssInv_ << 20./std::pow(prediction_time_, 5), 0, 0,
                0, 20./std::pow(prediction_time_, 5), 0,
                0, 0, 20./std::pow(prediction_time_*1., 5);

        Ks_ << std::pow(prediction_time_, 3)/6.,std::pow(prediction_time_, 4)/8.,std::pow(prediction_time_, 5)/20.,0,0,0,0,0,0,
                0,0,0,std::pow(prediction_time_,3)/6.,std::pow(prediction_time_,4)/8.,std::pow(prediction_time_,5)/20.,0,0,0,
                0,0,0,0,0,0,std::pow(prediction_time_*1., 3)/6.,std::pow(prediction_time_*1., 4)/8.,std::pow(prediction_time_*1., 5)/20.;

        dy_mod_params_ = Eigen::Map<DyModParamVector>(cfg_dy_mod_params.data());

        integration_time_step_ = cfg_integration_time_step;
        // std::cout << "RECONFIG time step " << cfg_integration_time_step;
}
/**
 * Computes velocity commands. It expects a reference trajectory and state
 * feedback to be in the same frame of reference.
 * @param  x_feedback     [description]
 * @param  y_feedback     [description]
 * @param  psi_feedback   [description]
 * @param  v_feedback     [description]
 * @param  omega_feedback [description]
 * @param  trajectory     [description]
 * @param  eval_time      [description]
 * @param  plan_time      [description]
 * @param  plan_stage     [description]
 * @param  lin_vel_output [description]
 * @param  ang_vel_output [description]
 * @return                false if it encounters problems computing the velocities
 *                        in this case output will be invalid, true otherwise
 */

}
