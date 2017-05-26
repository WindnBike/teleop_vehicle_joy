/**
BSD 3-Clause License

Copyright (c) 2017, WindnBike
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_vehicle_joy/teleop_vehicle_joy.h"
#include "teleop_vehicle_joy/tele_vehicle.h"
#include <string>


namespace teleop_vehicle_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopVehicleJoy
 * directly into base nodes.
 */
    struct TeleopVehicleJoy::Impl
    {
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        ros::Subscriber joy_sub;
        ros::Publisher cmd_vel_pub;

        /**
         * key mapping definitions
         */
        int left_axis_x;
        int left_axis_y;
        int right_axis_x;
        int right_axis_y;
        int arrow_key_x;
        int arrow_key_y;
        int button_a;
        int button_b;
        int button_x;
        int button_y;
        int left_bumper;
        int right_bumper;
        int left_trigger;
        int right_trigger;
        int back_button;
        int start_button;
        int axis_range;

        double ex_velocity;
        double ex_angular;
        /**
         * value mapping definitions
         */
        Phy_val op_velocity;
        Phy_val op_angular;

        bool sent_disable_msg;
        bool op_enable_flag;
    };

/**
 * Constructs TeleopVehicleJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
    TeleopVehicleJoy::TeleopVehicleJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
    {
        pimpl_ = new Impl;

        pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopVehicleJoy::Impl::joyCallback, pimpl_);

        nh_param->param<int>("left_axis_x", pimpl_->left_axis_x, 0);
        nh_param->param<int>("left_axis_y", pimpl_->left_axis_y, 1);
        nh_param->param<int>("right_axis_x", pimpl_->right_axis_x, 2);
        nh_param->param<int>("right_axis_y", pimpl_->right_axis_y, 3);
        nh_param->param<int>("arrow_key_x", pimpl_->arrow_key_x, 4);
        nh_param->param<int>("arrow_key_y", pimpl_->arrow_key_y, 5);
        nh_param->param<int>("button_a", pimpl_->button_a, 0);
        nh_param->param<int>("button_b", pimpl_->button_b, 1);
        nh_param->param<int>("button_x", pimpl_->button_x, 2);
        nh_param->param<int>("button_y", pimpl_->button_y, 3);
        nh_param->param<int>("left_bumper", pimpl_->left_bumper, 4);
        nh_param->param<int>("right_bumper", pimpl_->right_bumper, 5);
        nh_param->param<int>("left_trigger", pimpl_->left_trigger, 6);
        nh_param->param<int>("right_trigger", pimpl_->right_trigger, 7);
        nh_param->param<int>("back_button", pimpl_->back_button, 8);
        nh_param->param<int>("start_button", pimpl_->start_button, 9);
        nh_param->param<int>("axis_range", pimpl_->axis_range, 32768);

        nh_param->param<double>("max_vel", pimpl_->op_velocity.max_val, 0.25);
        nh_param->param<double>("max_vel_acc", pimpl_->op_velocity.max_1st_dif, 0.01);
        nh_param->param<double>("vel_restore", pimpl_->op_velocity.res_1st_dif, 0.005);
        nh_param->param<double>("max_ang", pimpl_->op_angular.max_val, 2.1);
        nh_param->param<double>("max_ang_acc", pimpl_->op_angular.max_1st_dif, 0.2);
        nh_param->param<double>("ang_restore", pimpl_->op_angular.res_1st_dif, 0.5);
        pimpl_->op_velocity.max_rval=pimpl_->op_velocity.max_val;
        pimpl_->op_velocity.max_1st_rdif=pimpl_->op_velocity.max_1st_dif;
        pimpl_->op_angular.max_rval=pimpl_->op_angular.max_val;
        pimpl_->op_angular.max_1st_rdif=pimpl_->op_angular.max_1st_dif;

        teleVehicle_ = new TeleVehicle(pimpl_->op_velocity,pimpl_->op_angular);

        /*
        ROS_INFO_NAMED("TeleopVehicleJoy", "Teleop enable button %i.", pimpl_->enable_button);
        ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopVehicleJoy",
                            "Turbo on button %i.", pimpl_->enable_turbo_button);

        for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
             it != pimpl_->axis_linear_map.end(); ++it)
        {
            ROS_INFO_NAMED("TeleopVehicleJoy", "Linear axis %s on %i at scale %f.",
                           it->first.c_str(), it->second, pimpl_->scale_linear_map[it->first]);
            ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopVehicleJoy",
                                "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_turbo_map[it->first]);
        }

        for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
             it != pimpl_->axis_angular_map.end(); ++it)
        {
            ROS_INFO_NAMED("TeleopVehicleJoy", "Angular axis %s on %i at scale %f.",
                           it->first.c_str(), it->second, pimpl_->scale_angular_map[it->first]);
            ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopVehicleJoy",
                                "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_turbo_map[it->first]);
        }
        */

        pimpl_->sent_disable_msg = false;
        pimpl_->op_enable_flag = true;
    }

    void TeleopVehicleJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        // Initializes with zeros by default.
        geometry_msgs::Twist cmd_vel_msg;
        if(!op_enable_flag && joy_msg->buttons[start_button]){
            sent_disable_msg = false;
            op_enable_flag = true;
        }
        if (op_enable_flag && !joy_msg->buttons[back_button]){
            if(joy_msg->buttons[left_bumper]){
                //TODO write the func
            }
            if(joy_msg->buttons[right_bumper]){
                //TODO write the func
            }
            if(joy_msg->buttons[left_trigger]){
                //TODO write the func
            }
            if(joy_msg->buttons[right_trigger]){
                //TODO write the func
            }
            if(joy_msg->buttons[button_x]){
                //TODO write the func
            }else if(joy_msg->buttons[button_y]){
                //TODO write the func
            }else if(joy_msg->buttons[button_a]){
                //TODO write the func
            }else if(joy_msg->buttons[button_b]){
                teleVehicle_->reset();
            }else {
                ex_velocity = joy_msg->axes[left_axis_y] / axis_range ;
                ex_angular = joy_msg->axes[right_axis_x] / axis_range ;
                teleVehicle_->changeVel(ex_velocity);
                teleVehicle_->changeVel(ex_angular);
            }
            //TODO write the func for translate TeleVehicle to cmd_vel.
            cmd_vel_pub.publish(cmd_vel_msg);
            sent_disable_msg = false;
        }else if(op_enable_flag && joy_msg->buttons[back_button]){
            //TODO write the func for slow down.
            cmd_vel_pub.publish(cmd_vel_msg);
            sent_disable_msg = true;
            op_enable_flag = false;
        }
    }

}  // namespace teleop_vehicle_joy
