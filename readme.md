
# MAVSim-B

MAVSim-B is a virtual drone arena for simulating its flight dynamics & control. The physics as well as the flight control, guidance, and navigation algorithms are implemented in MATLAB, while the animation is developed in Unity 3D; it looks like an indoor arena. It is an easy-to-use flight simulator intended to help the guys who are interested to learn or to do research and development on aerial robotics.



For a tutorial guide of MAVSim-B, go to its [documentation](https://www.professordavisantos.com/) webpage. 



## How to use it

To execute MAVSim-B, you need to have MATLAB 2019b (or later) and a joystick (IPEGA PG-9076).  

1. Turn on your joystick. 
2. Run `~/UNITY3D/MAVSimAnimation.exe`.
3. From MATLAB, run `~/MATLAB/MAIN.m`.
4. In the MATLAB command window, interact with the app until the simulation is *ready* to start. At this point, you are able to arm your drone.
5. To initiate the flight, you need first take-off automatically (see all the joystick commands in the figure below).
6. After taking off, the flight enters into the MANUAL state. Now you can use the joystick to play!  
7. From the MANUAL mode, you can also initiate the waypoint-based guidance.

In other words, the simulator behaves pretty much the same as a real drone flight in your lab's arena!



## Comments about the implementation

The MATLAB app of MAVSim-B is implemented using the object orientation paradigm. Here is the list of classes developed for this project:

* `CMav`: Include the MAV parameters, input and output variables, and the functions to compute resulting torque and force, propellers' thrusts, and integration of the equations of motion.

* `CSensors`: Implements the models of the navigation sensor's measurements. 

* `CUncer`: Implements the force and torque disturbances. 

* `CState`: Implements the state machine which rules the control modes (MANUAL, WAYPOINT, TAKE-OFF, LANDING, ARMED, etc.) and transition between them. 

* `CControl`: Include the flight controller parameters, input and output variables, and the functions to compute the attitude control, the position control, the control allocation, the reference filter, and the 3D attitude command.

* `CGuidance`: Implements waypoint-based guidance as an external law over the MAV in closed loop with position and attitude controllers. 
  
* `CNavigation`: Implements an attitude determination extended Kalman filter (EKF) and process the GPS measurements to estimate 3D position and velocity. 

* `CJoy`: Implements the interface with a Bluetooth joystick. 

* `CSocket`: Implements the TCP socket between the MATLAB and Unity apps.

* `CData`: prepare the message to send from MATLAB to Unity. 

  


## Joystick commands



## Acknowledgement

I would like to thank a lot all my students and colleagues who somehow helped me to start this project. Many of the ideas and methods implemented in MAVSim-B have been themes of the course [MP-282 Dynamic Modeling and Control of Multirotor Aerial Vehicles](https://www.professordavisantos.com/modeling-control-mav/) I have offered since 2014 in the Aeronautics and Mechanics Graduate Program at [ITA/Brazil](https://www.ita.br).  





