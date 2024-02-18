# MSBRDM FINAL PROJECT
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

## Run the Test Environment
Please follow the following commands:
```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch tum_ics_ur10_controller_tutorial simple_effort_controller.launch
```

## Impedance Controller

### Run the Impedance Controller
```bash
roslaunch impedance_controller impedance_controller.launch
```

### Test the Impedance Controller
Now you can only test the joint control:
```bash
rosrun knob_controller knob_control.py
```

### RosService
Move the arm to a specific joint position:
```bash
rosservice call /move_arm_joint "{joint0: 0.0, joint1: -1.0, joint2: -1.0, joint3: -1.0, joint4: 1.0, joint5: 0.0}"
```

Move the arm to a specific pose [ON PROGRESS]:
```bash
rosservice call /move_arm_cartesian "{x: 0.5, y: -0.2, z: 0.7, rx: 0.0, ry: 0.0, rz: 0.0}"
```