#include "ros/ros.h"
#include "capstone/encoder_value.h"
#include "capstone/pwm_output.h"
//#include "motor_sim.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <mutex>
#include <thread>
#include <future>


std::vector<int> motor_pwm{0,0};

void pwm_input_callback(const capstone::pwm_output::ConstPtr& msg)
{
//  std::unique_lock<std::mutex> lck(_mutex);
    motor_pwm[0] = msg->right_pwm;
    motor_pwm[1] = msg->left_pwm;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_simulation");
  ros::NodeHandle n;


  //set up of the control system
  ros::Publisher pub = n.advertise<capstone::encoder_value>("encoder_vel", 1000);
  ros::Subscriber sub = n.subscribe("motor_pwm", 1000, pwm_input_callback);
  //iniciating the node ad subsribers
  int cycleDuration = 50;


  std::chrono::time_point<std::chrono::system_clock> lastUpdate;
  lastUpdate = std::chrono::system_clock::now();
  while(true){
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
    if (timeSinceLastUpdate >= cycleDuration)
    {

      capstone::encoder_value data;

      data.right_encoder_vel = (int)((200*log((.02 *(motor_pwm[0])) + 2.014))-140);
      data.left_encoder_vel = (int)((250*log((.02 *(motor_pwm[1])) + 1.7507))-140);
      //std::cout << motor_pwm[1] << std::endl;
      pub.publish(data);

      lastUpdate = std::chrono::system_clock::now();
    }
  //sleep until message comes in and sends to callback
  }

  return 0;
}
