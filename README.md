# ROS_motor_velocity_control_system (Udacity C++ Nanodegree Capstone project)
This ROS packet is a control system that allows the velocity of 2 motor to be controlled, making it useful for differential drive robots since having a robust control is key for more advanced applications. The project consist in the implementation of a control system class that uses PID, set the control update time, tunning parameters (kp,ki,kd) and saturation, making it a very flexible and useful option for many aplications. To simulate this without using physical componentes we created a simulated motor, that will represent physical motors. 

# How to build
This control system is a packet that can be integrated in your current ROS workspace. Using ROS (Robotic Operating System) can be a bit complicated if you don't have any previous knowledge of ROS. For Simplicity I will give the procedures of how to create a workspace and install the packet in the workspace (ROS must be previously installed. Here is a helful link for the installation and tutorials of ROS http://wiki.ros.org/ROS/Tutorials)
#### 1.How to create a workspace
First we need to create a ros workspace and for that we will create a directory in the desiered location and call it how ever you want (Ill use "nanodegree_ws"). We then call "catkin_make" which will build the workspace. After that we will see that more files were created. (NOTE: {distro} is the Ros version that you initially installed, I used melodic)
```
source /opt/ros/{distro}/setup.bash
mkdir -p ~/nanodegree_ws/src
cd ~/nanodegree_ws/
catkin_make
```
#### 2.How to install the packet and compile
We will use git-clone to copy this file into the src directory of the workspace and apply catkin_make once more on the workspace main directory. This will compile the packet and be ready to run. 
```
cd ~/nanodegree_ws/src
git clone {this github link}
cd ~/nanodegree_ws/
catkin_make
```
#### 3.Run the control system
To run the the main program which is the control system we must first source the workspace so that ROS knows what packet we are using. We then use "rosrun" that will run the executable using the ros environmet. The program will be waiting until another sends data to it. NOTE: If you already have a robot that sends velocity data over a node, you can consult the code documentation section below to integratecit to the system.
IN A SPEARATE TERMINAL TAB
```
source /opt/ros/{distro}/setup.bash
source  ~/nanodegree_ws/devel/setup.bash
rosrun Udacity-Capstone-CPP-Nanodegree control
```
#### 4.Run the motor simulation
(OPTIONAL) This part is optional becuase for testing purposes and if there is no robot currently being used but it allows for simulation of an physical motor.
IN A SPEARATE TERMINAL TAB
```
source /opt/ros/{distro}/setup.bash
source  ~/nanodegree_ws/devel/setup.bash
rosrun Udacity-Capstone-CPP-Nanodegree simulation
```
#### 5.Run the plotter
One the nodes are running you can graph the control system to see how it responds and make further adjustments to the control system, we will use rqt_plot to do this.
```
source /opt/ros/{distro}/setup.bash
source  ~/nanodegree_ws/devel/setup.bash
rosrun rqt_plot rqt_plot
```
You can now add the topics that being sent on the system, by adding the topics using the "+" button. Note: If you dont input any reference (see *user input*) all the values will be 0, therefore we must input to see the response of the system.
 ADD PHOTO

### User Input
#### Set the Reference (Target Velocity of the Motor)
We can input the reference velocity via the "ref_param" topic (in depth explanation in code documentation) for this we use "rostopic pub" to manualy publish data. If you already have a Node that can publish reference velocity of your robot you can just publish to the ref_param using the ref_param.msg (more in depth in explanation). We can set which motor we want to set the reference velocity "motor_num" (for this application it must be either 0 or 1) and the reference speed (only int)

```
source /opt/ros/{distro}/setup.bash
source  ~/nanodegree_ws/devel/setup.bash
rostopic pub /reference capstone/ref_param "motor_num: 0
ref: 100" 
"
```
#### Set the PID Tune parameters (kp,ki,kd)
We can input the Tune parameters via the "tune_param" topic (in depth explanation in code documentation) for this we use "rostopic pub" to manualy publish data. If you already have a Node that can publish tuning data (maybe a auto pid tuner) you can publish to the tune_param using the tune_param.msg (more in depth in explanation). We can set which motor we want to set the tune parameters "motor_num" (for this application it must be either 0 or 1) and the respective kp, ki, and kd parameters (float values).

```
source /opt/ros/{distro}/setup.bash
source  ~/nanodegree_ws/devel/setup.bash
rostopic pub /tune_param capstone/tune_param "motor_num: 0
kp: 0.0
ki: 0.0
kd: 0.0" 
```

System control class



output and interpretation

rubric
