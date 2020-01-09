#ifndef VEL_CTL_H
#define VEL_CTL_H
#include "ros/ros.h"
#include "capstone/encoder_value.h"
#include "capstone/pwm_output.h"
#include "capstone/tune_param.h"
#include "control_system.h"
#include <vector>

class node_manager
{
public:
  node_manager();
  void encoder_velocity_callback(const capstone::encoder_value::ConstPtr& msg);
  void control_tune_callback(const capstone::tune_param::ConstPtr& msg);
  void control_reference_callback(const capstone::ref_param::ConstPtr& msg);
private:
  ros::NodeHandle _n;
  ros::Publisher _motor_pub;
  ros::Subscriber _sub, _tune_sub,_ref_sub;
  std::vector<control_sys> motors;
  int _num_motors = 2;
};






#endif
