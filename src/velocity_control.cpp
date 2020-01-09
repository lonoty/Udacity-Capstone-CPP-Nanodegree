#include "ros/ros.h"
#include "capstone/ref_param.h"
#include "capstone/encoder_value.h"
#include "capstone/pwm_output.h"
#include "capstone/tune_param.h"
#include "control_system.h"
#include "velocity_control.h"
#include <vector>


  node_manager::node_manager()
  {

    motors.push_back(control_sys(0.1, 3, 0.001, 0.05, 0, 255, 0));
    motors.push_back(control_sys(0.1, 2, 0.001, 0.05, 0, 255, 0));
    _motor_pub = _n.advertise<capstone::pwm_output>("motor_pwm", 1000);
    _sub = _n.subscribe("encoder_vel", 1000, &node_manager::encoder_velocity_callback, this);
    _tune_sub = _n.subscribe("tune_param", 1000, &node_manager::control_tune_callback, this);
    _ref_sub = _n.subscribe("reference", 1000, &node_manager::control_reference_callback, this);

  }
  void node_manager::control_reference_callback(const capstone::ref_param::ConstPtr& msg){
    motors[msg->motor_num].set_reference(msg->ref);
  }
  void node_manager::encoder_velocity_callback(const capstone::encoder_value::ConstPtr& msg)
  {
  capstone::pwm_output pwm;
  pwm.right_pwm =  motors[0].update(msg->right_encoder_vel);
  pwm.left_pwm =  motors[1].update(msg->left_encoder_vel);
  _motor_pub.publish(pwm);
  }
  void node_manager::control_tune_callback(const capstone::tune_param::ConstPtr& msg)
  {
    motors[msg->motor_num].set_all_param(msg->kp, msg->ki, msg->kd);
  }

//callback everytime the encoder/microcontroller sends data


int main(int argc, char **argv)
{
  //set up of the control system

  //iniciating the node and subsribers
  ros::init(argc, argv, "velocity_control");
  node_manager node;
  //sleep until message comes in and sends to callback
  ros::spin();

  return 0;
}
