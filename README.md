# MSBRDM FINAL PROJECT
## Build

install the pkg:

```bash
sudo dpkg -i tum*
sudo dpkg -i lib*
sudo dpkg -i ros*
```

```bash
catkin build -DTUM_ICS_USE_QT5=1
```

> Note: If you face any issues with the build, please first see the [Issue #35](https://gitlab.lrz.de/msbrdm/msbrdm-lecture-2023/-/issues/35).

## Run the Test Environment
Please follow the following commands:
```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
roslaunch tum_ics_ur10_controller_tutorial simple_effort_controller.launch
```

## Some Useful Path
```bash
\\wsl.localhost\Ubuntu-20.04\opt\ros\noetic\include\tum_ics_ur_robot_lli
```

## Run the Impedance Controller
```bash
roslaunch impedance_controller impedance_controller.launch
```