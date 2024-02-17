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

### RosService
Move the arm to a specific joint position:
```bash
rosservice call /move_arm_joint "{joint0: 0.0, joint1: -1.0, joint2: -1.0, 
joint3: -1.0, joint4: 1.0, joint5: 0.0}"
```