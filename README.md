# Drone Obstacle Avoidance
 
![](https://github.com/farhannz/Drone-Test/raw/main/test.gif)


Toolkit :
* Unity ML-Agents Toolkit


Description :   
An attempt to simulate a drone obstacle avoidance system based on computer vision using reinforcement learning in Unity3D.
The drone's core control system is implemented by attaching a PID (Proportion-Integral-Derivative) controller to each rotor blade. The core control system will receive input signals and forward them to the PID controller, then the PID controller will return outputs that control the rotor blade.
In this case, this paper uses tools from the Unity ML-Agents Toolkit to generate random input signals for yaw, roll, and pitch. These generated input signals and the camera attached to the drone will be used as input for reinforcement learning. 

