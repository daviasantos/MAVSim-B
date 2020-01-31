---------------------------------------------------------------------------
MAVSim: Simulation of an MAV dynamics & flight control 
---------------------------------------------------------------------------
Objetive: to serve as a simulator for people who are interested to do 
research and development on aerial robotics.

---------------------------------------------------------------------------
Author: Davi A. Santos (ITA)
---------------------------------------------------------------------------


This program implements flight dynamics simulation of an MAV. It includes two
apps: a MATLAB one, which is responsible for the physic simulation; and a UNITY 
3D one, which provides the visual representation of the flight scene. The current 
version uses an H-shaped quadrotor UAV.

Characteristics:

It consists of pure MATLAB simulation ...
    
- ... of THE MAV dynamics and environment physics     (class: CMav)         
- ... of flight controllers                           (class: CControl)
- ... of attitude determination and gps               (class: CNavigation)
- ... of a joystick for manual mode                   (class: CJoy)
- ... of a TCP socket to communicate with Unity 3D    (class: CSocket)

To execute it:

- set the Parameters.m (or use the provided ones);
- start the Unity app 
- run MAIN.m;
 
