#include "ros/ros.h"
#include "capstone/encoder_value.h"
#include "capstone/pwm_output.h"
#include "motor_sim.h"
#include "motor_node.h"
#include <chrono>
#include <vector>
#include <mutex>
#include <thread>
#include <future>



  node_manager::node_manager()
  {

    for(int i; i < _num_motors; i++){
      motors.push_back(motor_sim());
    }
    _pub = _n.advertise<capstone::encoder_value>("encoder_vel", 1000);
    _sub = _n.subscribe("motor_pwm", 1000, &node_manager::pwm_input_callback, this);
    for(int i; i < _num_motors; i++){
      motors[i] = motor_sim();
    }


  }

  void node_manager::pwm_input_callback(const capstone::pwm_output::ConstPtr& msg)
  {
  //  std::unique_lock<std::mutex> lck(_mutex);
    motor_pwm[0] = msg->right_pwm;
    motor_pwm[1] = msg->left_pwm;


  }
  void node_manager::motor_simulation(){
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    lastUpdate = std::chrono::system_clock::now();
    while(true){
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration)
        {
          /*
          std::unique_lock<std::mutex> lck(_mutex);
          std::vector<std::future<int>> ftr;
          for(int i = 0; i < _num_motors; i++){
            int a = motor_pwm[i];
          ftr.emplace_back(std::async(&motor_sim::motor_update, motors[i], a));
          }
          lck.unlock();
          std::vector<int> encoder_data;
          for(int i = 0; i < _num_motors; i++){
            encoder_data[i] = ftr[i].get();
          }*/
          std::vector<int> encoder_data;
          for(int i = 0; i < _num_motors; i++){
            encoder_data[i] = motors[i].motor_update(motor_pwm[i]);
          }
          capstone::encoder_value data;
          data.left_encoder_vel = encoder_data[0];
          data.right_encoder_vel = encoder_data[1];
          _pub.publish(data);
          lastUpdate = std::chrono::system_clock::now();
        }
      }

  }

int main(int argc, char **argv)
{
  //set up of the control system

  //iniciating the node and subsribers
  ros::init(argc, argv, "motor_simulation");
  node_manager node;
  node.motor_simulation();
  //sleep until message comes in and sends to callback


  return 0;
}
