# MSBRDM FINAL PROJECT

### TODO
- [x] Add the ROS Service for Joint Control
- [x] VS Code debugger for the ROS nodes
- [x] Add the ROS Service for Cartesian Control
- [ ] impedance controller for the end-effector
- [ ] addmitance controller for the end-effector
- [ ] orientation spline
- [ ] Fix vibration of cartesian control

### BUGS
- [x] Vibrations when change the control mode
- [x] EE position is not accurate

## Impedance Controller
### Run the Impedance Controller
```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch impedance_controller impedance_controller.launch
```

### Test the Impedance Controller
Now you can only test the joint control:
```bash
rosrun knob_controller knob_control.py
```

### Ros Service
Move the arm to a specific joint position:
```bash
rosservice call /move_arm_joint "{joint0: 0.0, joint1: -1.0, joint2: -1.0, joint3: -1.0, joint4: 1.0, joint5: 0.0}"
```

Move the arm to a specific pose [ON PROGRESS]:
```bash
rosservice call /move_arm_cartesian "{x: 0.5, y: -0.2, z: 0.7, rx: 0.0, ry: 0.0, rz: 0.0}"
```

## Build

install the pkg:

```bash
sudo dpkg -i tum*
sudo dpkg -i lib*
sudo dpkg -i ros*
```

```bash
catkin build
```

> Note: If you face any issues with the build, please first see the [Issue #35](https://gitlab.lrz.de/msbrdm/msbrdm-lecture-2023/-/issues/35).


## Knob
Check the IP addr
```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch knob_robot_control rosserial.launch
rosrun knob_robot_control qt5_gui.py 
roslaunch impedance_controller impedance_controller.launch
```

## Run the Test Environment
Please follow the following commands:
```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch tum_ics_ur10_controller_tutorial simple_effort_controller.launch
```

## Force Torque Sensor

```bash
roslaunch tum_ics_schunk_netbox sensor_publisher.launch
rostopic echo /schunk_netbox/raw 
header: 
  seq: 27494
  stamp: 
    secs: 1708778150
    nsecs: 385334359
  frame_id: "ft_sensor_link"
wrench: 
  force: 
    x: -131.479277
    y: -76.275814
    z: 485.809591
  torque: 
    x: -0.78713
    y: -6.932118
    z: -4.1215
---

rostopic info /schunk_netbox/raw 
Type: geometry_msgs/WrenchStamped

Publishers: 
 * /schunk_netbox/schunk_netbox (http://redball:36389/)

Subscribers: None


```

## Run the Real Ur

```bash
roscore

roslaunch tum_ics_ur10_bringup bringUR10.launch

roslaunch tum_ics_ur_robot_manager robot_script_manager_ur10.launch

roslaunch impedance_controller impedance_controller.launch

roslaunch tum_ics_schunk_netbox sensor_publisher.launch

rosrun knob_robot_control fake_ft_pub.py

```
