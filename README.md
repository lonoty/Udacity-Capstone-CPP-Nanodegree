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
You can now add the topics that being sent on the system, by adding the topics on the textbox write /encoder_vel and /motor_pwm using the "+" button you create them. Note: If you don't input any reference (see *user input*) all the values will be 0, therefore we must input to see the response of the system.
![plot](https://github.com/lonoty/Udacity-Capstone-CPP-Nanodegree/img/graph.png)
 

# User Input
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
# Code documentation
The system uses ROS messages to communicate from node to node, in this case the control system by it self is a node that recieves the encoder velocity data and outputs the proper PWM signal to be process. Meanwhile the there must be another node or nodes that actually interpert the PWM and outputs it to the motor it can be a moto-rcontroller that takes in a byte 0-255 to represent the duty cycle of the PWM. Another system then must inteprete the motor movement using encoder and return the velocity of the system. This can be seen as the node representation of the system (see image).
PUT IMAGE HERE
## Control System Class
The control system class (control_system.cpp) is a class that implements a PID system in a class where there are tuneable parameters, refresh time and reference. The system by it self can be implemented on anything that needs a pid, this is due to the flexibility and versatibility of the system, it only applies a general PID system meanwhile the velocity control adapts it to motor velocity needs.
### Main Parameters
The main parameters of this system are in the private sectoin of the class so that a user can't modify data without using setters and getters, the parameters are as follows

- **double _refresh_time{.1}** : The refresh rate of the system 
- **double _kp{1}, _kd{1}, _ki{1}** : Tuning parameters of system
- **int _saturationLOW{0}, _saturationHIGH{100}** : Saturation range (this helpes the system have limit so that the output is controlled)
- **double _prev_derivative{0}, _prev_error{0}** : Preveious values to take deviative and integral value
- **double integral** : Accumulation of the error to get integral
- **double _reference{0}** : The reference of the system

### Constructors
This system uses contructor overloading to have flexibility on the use and how a user can start the system.
```
control_sys();
control_sys(double kp, double ki, double kd);
control_sys(double kp, double ki, double kd, double refresh);
control_sys(double kp, double ki, double kd, double refresh, int satLOW, int satHIGH, double reference);
```
### getters and setters
The getters and setters allow for a flexible and very tuneable operation of the system
```
//setters
void set_reference(double reference);
void set_saturation_range(int satLOW, int satHIGH);
void set_all_param(double kp , double ki, double kd);
void set_kp(double kp);
void set_ki(double ki);
void set_kd(double kd);
//getters
double get_reference();
int get_saturation_low();
int get_saturation_high();
double get_kp();
double get_ki();
double get_kd();
```
### Functions
The system only consist of two functions:

**int update(int feedback)** : This function is the main algorithm of the PID implementation where the feedback (the encoder response) is taken as an argument then it returns an int tht is the Output PWM. Note: This function has to be called at the same time intervals set as the refresh rate. This is due to the discrete properties of this controller.

**int saturate(int value)** : This fuction is a saturation fuction that allows for a maximum and minimum output to be retuned, this helps to the system to not exeed the physical/electrical demands of a system.

## Velocity Control
The velocity Control program (velocity_control.cpp) is the contains main code where the it implements the PID system class to the needs of velocity control and also integrates it with ROS messages so it can be used as a node.

### Node Manager
Since whe are using ROS we created a class that manages the nodes subscribers and publishers so when ever an external node publishes data to the system, node manager will call a callback function to allow the data to be processed. 

#### Constructor

The constructor first initiates 2 control_sys classes with preset parameters and pushes it to a vector that stores the objects. Then it initiates all the comunication protocols used by ROS, to the desired callback functions. The protocols allow for the the user to input tune_parameters, encoder_velocity, and the references, Then it outputs the desired PWM data.
```
motors.push_back(control_sys(0.1, 3, 0.001, 0.05, 0, 255, 0));
motors.push_back(control_sys(0.1, 2, 0.001, 0.05, 0, 255, 0));
_motor_pub = _n.advertise<capstone::pwm_output>("motor_pwm", 1000);
_sub = _n.subscribe("encoder_vel", 1000, &node_manager::encoder_velocity_callback, this);
_tune_sub = _n.subscribe("tune_param", 1000, &node_manager::control_tune_callback, this);
_ref_sub = _n.subscribe("reference", 1000, &node_manager::control_reference_callback, this);
```
#### Messages
ROS requires there to be communication protocals that send specific data called messages, this messages are then handeled by the system. For this applicatoin I created 4 message structures which ecapsulates a structure of data. 

**encoder_value.msg** : is used to send the speed of the 2 motor (right and left)

**pwm_ouput** : is used to send the pwm output of the 2 motors (right and left)

**ref_param** : is used to send the refence (target speed of a motor) and decide which motor is needed (the right 0, or the left 1)

**tune_param** : is used to allow the user to send tuning parameters of the system (kp, ki, kd) and to what motor to send it (the right 0, or the left 1)

#### Callbacks
**control_reference_callback** the control reference callback takes as an argument the ref_param message as a constant pointer, this will call the set_reference from the control system class with the desired motor (called in the vector object) to set the reference

**control_tune_callback** the control tune callback takes as an argument the tune_param message as a constant pointer, this will call the set_all_param from the control system class with the desired motor (called in the vector object) to set the three tuning parameters

**encoder_velocity_callback** the enconder velocity callback takes as an argument the encoder_value message as a constant pointer, this callback is called whenever data form the motor encoder is being sent (it must be the same refresh frequency as the system), this will call the update function from the control system class for both the motors that then will proceed to the PID algorithm and return the pwm values for both systems, this will then publish data to the motor using the pwm_output message


### Main 
The main takes two arguments that are used in the ROS iniciation then initiates the ros node with the name "velocity control", then creates the node_manager, and at last ros::spin() allows for the program be idle until a message arrives which then calls the function and then the callback function.

## Simulation
The simulation node is just a simple representation of the motor, which will take in the pwm value of the system and output the corresponding motor velocity. The system returns data at a said refresh rate asuming that it is a microcontroller that send data in intervals. To adquire the data I set a formula that looked closed to the normal response of a duty cycle with respect to the RPM. I got this simulation from a research paper: 

# My prototype 
I used this packet with my own robot to have a robust control system (see robot). The pid algorithm is succesful with my parameters and allows for an easy implementation of the system. You can see a video of the demo in the video folder.
# Rubric
- The project demonstrates an understanding of C++ functions and control structures.
  - while loop in motor_node.cpp(line. 39)
  - if statemet in motor_node.cpp(line. 43)
- The project accepts user input and processes the input.
  - This can be seen in the user input section of the readme.md
  - Takes a callback that is given by the user to set reference of motor in velocity_control.cpp (line. 22)
  - takes a callback that is given by the user to set tune variables of the motor in velocity_control.cpp (line.32)
- The project uses Object Oriented Programming techniques.
  - Class used for the motor pid called control_sys control_system.h (line. 7)
  - class used for the node manager velocity_control.h (line. 10)
- Classes use appropriate access specifiers for class members.
  - Access specifiers for control_sys class control_system.h (line. 33)
  - Access specifiers for node managerclass velocity_control.h (line. 17)
- Class constructors utilize member initialization lists.
  - constructors utilize it in control_sys class control_system.cpp (line.3-5)
- Classes abstract implementation details from their interfaces.
  - the readme.md is a formal documentation of the code
  - variables are also commented in program for clarity control_system.h (line. 34 - 46)
- Classes encapsulate behavior.
  - the control_sys class by it self can work in other applications because it has all the required parameters to work out of this context of velocity control  control_system.h (line. 7)
- Overloaded functions allow the same function to operate on different parameters.
  - the constructor of the control_sys class can be iniciated in multiple ways depending in the need of the user control_system.h (line 9-12)
- The project makes use of references in function declarations.
  - to pass the callback functions reference was used so that ros knew what callabe function to refer to (cant be done without reference) velocity_control.cpp (line. 17-19)
- The project uses smart pointers instead of raw pointers.
  -not exactly smart pointers but ROS sends boot::ConstPtr velocity_control.cpp (line.22 ,line.25)
  
  
