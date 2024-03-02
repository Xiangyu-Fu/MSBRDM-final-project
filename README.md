# MSBRDM FINAL PROJECT
Before you begin, ensure that your router is properly connected, your computer is linked to the robot via an Ethernet cable, and the router's signal is active. Here's a refined guide for your README file:

### 1. Building the Environment
Execute the following command to build the environment:
```bash
catkin build
```

### 2. Launching the System
Start the system by running these commands:

```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch impedance_controller impedance_controller.launch
```

Observing these commands in action, you'll notice the robotic arm transitioning from its initial stance to the 'home' position. You can maneuver the arm using the following services:

```bash
rosservice call /move_arm_joint "{joint0: 0.0, joint1: -1.0, joint2: -1.0, joint3: -1.0, joint4: 1.0, joint5: 0.0}"
rosservice call /move_arm_cartesian "{x: 0.5, y: -0.2, z: 0.7, rx: 0.0, ry: 0.0, rz: 0.0}"

```

### 3. Enabling Force Feedback and Monitoring
To activate the force feedback and monitor the data, use:

```bash
roslaunch tum_ics_schunk_netbox sensor_publisher.launch
rostopic echo /schunk_netbox/raw
```

This command outputs force data in the terminal, predominantly showing the z-direction force around an average of 485.

### 4. Connecting and Using the Knob
To link and control the knob, initiate the following script:

```bash
rosrun knob_controller knob_control.py
```

A graphical interface will appear, allowing you to toggle between controller modes.

> Remember to wait for the "Joint Control mode activated" message before switching modes via the GUI. 

Once a mode is selected, rotating the knob will move the robotic arm. When the arm makes contact with an object, the knob will provide tactile feedback.








