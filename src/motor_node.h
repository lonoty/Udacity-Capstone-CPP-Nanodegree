#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include "ros/ros.h"
#include "capstone/encoder_value.h"
#include "capstone/pwm_output.h"
#include "motor_sim.h"
#include <vector>
#include <mutex>

class node_manager
{
public:
  node_manager();
  void pwm_input_callback(const capstone::pwm_output::ConstPtr& msg);
  void motor_simulation();
private:
  ros::NodeHandle _n;
  ros::Publisher _pub;
  ros::Subscriber _sub;
  std::vector<motor_sim> motors;
  int _num_motors = 2;
  int cycleDuration = 50;
  std::vector<int> motor_pwm;
  std::mutex _mutex;
};
#endif
