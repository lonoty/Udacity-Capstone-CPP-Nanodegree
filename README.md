# ROS_robot_motor_velocity_control_using_I-PI (Udacity C++ Nanodegree Capstone project)
This ROS packet is a control system that allows the velocity of 2 motor to be controlled concurrently, making it useful for differential drive robots since having a robust control is key for more advanced applications. The project consist in the implementation of a control system class that can use PID or I-PD, set the control update time, tunning parameters (kp,ki,kd) and saturation, making it a very flexible and useful option for many aplications. For each motor the control system will run in separate threads to allow faster response and scalability. 

How to build using ros and requierements (simulations also)

how it works

System control class

thread creation and execution

output and interpretation

rubric
