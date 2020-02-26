# Robot-Runner (README Work In Progress)
This framework enables automated experiment execution on ROS devices, both on physical (**real-life**, hereon after called **native**) robots and in a simulated setting (**Gazebo**, hereon after referred to as **sim**).

Robot-runner is developed to work with both **ROS1**, tested with **Kinetic** and **Melodic**, and **ROS2** , tested with **Eloquent**. Please carefully read and follow this README for the correct setup for your use case.

Robot-runner is developed and tested using **Python 3.6.9**, hereon after referred to as **Python**. Any other version of Python can possibly be used, but this has not been tested nor is it recommended.

Robot-runner is developed specifically for experimenting with a Turtlebot3 Burger, as this is the native ROS device provided by the **Vrije Universiteit Amsterdam**. Therefore example experiments and install guides also involve Turtlebot3 install guides. However, robot-runner is, as said before, developed to be used with **any** ROS device and thus can also be used without any Turtlebot software present. This would however mean that the example experiments provided, with the exception of the native experiments, are unusable. 

Read this README therefore carefully and adjust the install to your own personal needs.

```
WARNING: 

Currently ROS2 Native experiments are NOT WORKING. ROS2 Sim experiments ARE WORKING however. ROS2 Native experiments are not working since the recommended image for the Raspberry Pi in combination with ROS2: Ubuntu 18.04 Server image for ARM processors has a known bug in the Linux kernel. The Raspberry Pi gets too hot under any significant load and tries to throttle the CPU, however this results in an endless hardware interrupt from which the system is unable to recover, leading to a crash.

**This issue is being worked on**
```

# Intro

Robot-runner, has been developed out of a need to be able to perform many academic experiments repeatedly while preventing as much interference and manual labor as possible. With as end goal that these results can be used to perform academic research and draw scientifically sound conclusions.


Robot-runner has been developed by **Stan Swanborn** in early 2020 under the supervision of **Ivano Malavolta** as part of the course **Individual Systems Practical** as part of the ***Master's degree* Computer Science** at the **Vrije Universiteit Amsterdam**.

Robot-runner is specifically developed to be able to do research into the energy efficiency of architectural tactics for ROS devices (experimented specifically on a **Turtlebot3 Burger**) however, the framework is developed to be versatile and can be used for any sort of experiment or just any automated sequence of actions on any ROS device (native or simulated).

# Short description

Robot-runner is as versatile as it can be, this has as a consequence that it can be used in many different ways, leading to many different use cases. However, this README is written in the context of someone that wants to perform an academic, scientifically sound, automated ROS experiment. Using this context, this README explains to the user how to set up such an experiment. Since there are so many ways a user can start doing that, the most defining factors for any experiment are explained here.

Considering these factors use some implementation specific terms which are needed for explaining them, some terms are introduced before they are explained. In any case that this happens, the introduced term links to the section in which it is explained in detail.

This short description is given before any install guide, considering a correct understanding of the global workings of robot-runner and its use cases are vital for knowing what to install and why.

Any robot-runner experiment can be defined as a combination of these factors:

**<ros_version> | \<sim or native\> | <run_script or not> | \<timed or programmatic stop\>**

| Factor                     | Value                                                        |
| -------------------------- | ------------------------------------------------------------ |
| ros_version                | ROS1 (**Kinetic** on Ubuntu 16.04, **Melodic** on Ubuntu 18.04) <br />ROS2 (**Eloquent**, only Ubuntu 18.04) |
| sim or native              | Running an automated experiment on a simulation or on a native device |
| run_script or not          | Experiment actions defined in separate, controllable run script or not |
| timed or programmatic stop | Experiment run ends after a set timeout or when the run_completed ROS topic is published. |

These factors can be set and manipulated by the user in the **config.json** that will be described later in this README. Here the focus is to give a global, simplified understanding of robot-runner. With this simplified knowledge a general idea of combination of factors needed for any specific use case can already be derived. Considering the user has most probably already a use case in mind, a correct install and experiment setup guide can be chosen and followed using the understanding gained from this short description.

#### ros_version

The ROS version that robot-runner uses is the version that is currently activated on the system robot-runner runs on. Any system can have multiple installs of multiple ROS versions (multiple ROS1 installs, multiple ROS2 installs, all side-by-side), however only one ROS install can be active at a time. This one, active install, sets its version in the environment variable **ROS_VERSION**, the value of this environment variable is used by robot-runner to correctly determine the currently running ROS instance.

The **config.json** that can be defined by the user is later defined in this README. The ROS version is deliberately kept from this config to prevent human error; specifying ROS1 while a ROS2 install is activated.

#### sim or native

How robot-runner will set up any experiment and its communication with the ROS device is directly determined by the user setting in the **config.json**. There, the user is able to set the field: **use_simulator** to either **true** or **false**. When this field is false, a native experiment will be executed instead. This needs a correct setup and a running **ClientRunner** (sub-module of robot-runner) instance on the native ROS device. For more information on how to do this go **_HERE_???**

#### run_script or not

Robot-runner is specifically designed to be as versatile as possible and to only reduce the manual overhead needed to run an experiment. Therefore, it leaves any definition of an **experiment** (***the definition of the behavior of the ROS device***) to the user. 

The user has many options available as to where to define the experiment, provided natively by ROS. The user can, for example, define a node which manipulates a basic client on the ROS device. This node can be programmed in any language supported by the ROS library, and can be launched from a launch file. 

However, when the node is programmed in Python it can also be kept from the launch file and directly specified to robot-runner as the run script. This can be done by giving the path to the file in the **config.json** under the **run_script** field. This option has the advantage that **arguments** can be passed to the run script, these can also be specified in the **config.json**.

**Concrete scenario:** when the user runs a native experiment, the experiment can be defined on the ROS device as its behavior. However, this moves the variability of experiment definitions to the ROS device instead of the remote desktop. Instead, the user could choose to develop one client, which can be manipulated by any other ROS node. This would then be the run script, as specified to robot_runner. To run different experiments, the user would then only need to specify different run scripts. Moving the added memory and changing of experiments to the remote desktop instead of the ROS device.

#### timed or programmatic stop

An experiment consists of a specified number of replications, called runs. Each run needs to be stopped after being started, this can be done using a timeout (a set number of milliseconds in the **config.json** under the field: **duration**)  or using a programmatic stop. The programmatic stop is automatically enabled by robot-runner when the **duration** field is set to 0.

The programmatic stop has robot-runner initialize a ROS node (poll_run_complete) which subscribes to the topic: **robot_runner/run_completed**. When a message of type **std_msgs.msg.Bool** <u>Bool(True)</u> is published to the topic, the run is automatically stopped. This is deliberately designed to be a topic, as they are asynchronous and non-blocking. This means that any experiment node which publishes True does not have to wait for robot-runner to respond but can close immediately, ensuring a minimal impact of the robot-runner overhead on any gathered statistics.

# Install

Robot-runner has been developed mainly using Ubuntu 18.04. However, it has also been tested on Ubuntu 16.04, albeit with an older ROS1 version: **Kinetic**, as this is the last ROS1 version to be guaranteed on Ubuntu 16.04.

The install options available are thus:

| Ubuntu           | ROS1 Latest version | ROS2 Latest version | Use case                                                     |
| ---------------- | ------------------- | ------------------- | :----------------------------------------------------------- |
| Ubuntu 16.04     | Kinetic             | NONE                | In case any software used is dependent on Ubuntu 16.04. This install should be considered **Legacy support**. |
| **Ubuntu 18.04** | **Melodic**         | **Eloquent**        | **Recommended install**, ROS1 and ROS2 work with latest versions. Robot-runner is mainly developed and tested on this Ubuntu distribution. |

```
WARNING:

Robot-runner comes with a set of example experiments. Since robot-runner is specifically designed to experiment with a Turtlebot3 Burger, these examples need Turtlebot3 software to be installed with it. This is mentioned in each install guide, if you do not need or want these examples to work, then only follow the ROS install guide. 

Considering the Turltebot3 simulation does not provide any battery simulation, a Gazebo plugin is provided with each example. This plugin is developed by the CMU (Carnegie Mellon University), and used and modified for this project under the MIT license.

Examples that use this battery plugin are called VU_BATTSIM_1*.04 (* = 6 or 8, (16.04 or 18.04)). Please note that only Ubuntu 16.04 fully supports the original plugin with motor power taken into account in the battery discharge simulation. Ubuntu 18.04 does not provide this functionality, it thus only provides linear discharge, as legacy software used is not supported on 18.04.

This is however of no or slight importance to the user, the examples are only just that. The recommended install is therefore still Ubuntu 18.04.

NOTE: The VU_BATTSIM battery simulation is only available for ROS1.
```

After cloning, robot-runner is not immediately able to run successfully. It relies on installed tools and packages to be available, which can be easily installed using the correct setup scripts for your Operating System (Ubuntu 16.04 or Ubuntu 18.04). These setup scripts can be found in the corresponding experiment example folders: ``vu_battsim_16.04 || vu_battsim_18.04``.

The setup scripts are present in the <strong>install</strong> folder in each of the corresponding example packages.

### Recommended setup guides:

**FOR MANUAL ROS1 INSTALL**: http://wiki.ros.org/melodic/Installation/Ubuntu

​	If Turtlebot3 packages are required for your use case, follow the ROBOTIS guide as well: (**PC Setup**)

​	Follow from step 1.3:

​	http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc-setup

**FOR MANUAL ROS2 INSTALL**: https://index.ros.org/doc/ros2/Installation/Eloquent/

​	If Turtlebot3 packages are required for your use case, follow the ROBOTIS guide as well: (**PC Setup**)

​	Follow from step 1.3:

​	http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/

When choosing manual install but you still want the VU_BATTSIM example experiment to be able to run, still run `install_vu_battsim.sh` for each corresponding automatic install.

## Setup ROS1 for Ubuntu 16.04 (Automatic)<br>`not recommended`

**Note:** This is not recommended as the manual install guides are kept up-to-date by their respective developers and thus offer a better long-term support. Also knowing each individual setup step enables the user to create an install which fits perfectly to their specific use case.

`cd into /examples/ros1/experiments/vu_battsim_16.04/install and:`

<strong>STEP 1</strong>: ``chmod 755`` on ``install_ros_kinetic.sh``, ``install_ros_pkgs.sh`` and ``install_vu_battsim.sh``

<strong>STEP 2</strong>: run ``install_ros_kinetic.sh`` to install the correct ROS version (<strong>KINETIC</strong>)

<strong>STEP 3</strong>: run ``install_ros_pkgs.sh`` to install all necessary packages

<strong>STEP 4</strong>: run ``install_vu_battsim.sh`` to install the Gazebo battery simulation plugin (**OPTIONAL**)

## Setup ROS1 for Ubuntu 18.04 (Automatic)<br>`not recommended`

**Note:** This is not recommended as the manual install guides are kept up-to-date by their respective developers and thus offer a better long-term support. Also knowing each individual setup step enables the user to create an install which fits perfectly to their specific use case.

`cd into /examples/ros1/experiments/vu_battsim_16.04/install and:`

**STEP 1**: run ``install_ros_melodic.sh`` to install the correct ROS version (**MELODIC**)

**STEP 2**: run ``install_turtlebot3_srcs.sh`` to install all necessary turtlebot3 packages (**OPTIONAL**)

**STEP 3**: run ``install_vu_battsim.sh`` to install the Gazebo battery simulation plugin (**OPTIONAL**)


## Test successful install:

### Test ROS1 with VU_BATTSIM installed

From the vu_battsim_1*.04 directory in a terminal:

``roslaunch launch/vu_battsim.launch``

You should now observe:
 - The terminal should launch a ROS master
 - The [VU_BATT] Gazebo plugin should only output in green
 - A Gazebo instance should now be running, showing a Turtlebot3 Burger in a house.
 - The ROS topic `/mobile_base/commands/charge_level` should be available (published)

### Test without VU_BATTSIM installed

To check whether your ROS environment, either ROS1 or ROS2, is correctly installed you can use the command:

```bash
printenv | grep ROS
```

# ROS Device install

**NOTE:** This step is only necessary when you want to perform a **native experiment**. If you are only interested in simulating a ROS device, then your install is already completed if the above described test was successful.



# User guide

In this section, an elaborate user guide will be given as to how to set up any experiment with any combination of use cases. Important aspects will be described such as:

* How to setup a correct experiment for any use case
* How to run a correct experiment for any use case
* How to gather any data from a correct experiment for any use case
* How to process this data and be able to analyze this data



# Technical documentation

This section is only of interest to those that wish to understand the inner-workings of robot-runner, those that wish to expand robot-runner and those that want to see the rationale behind the design decisions made. It is therefore mainly for Software Engineers or those that have such skills. Any copying, alteration or redistribution of this software package (robot-runner) is allowed as it is provided under the MIT license. As long as the original author (**Stan Swanborn**), the supervisor (**Ivano Malavolta**) and the institution for which it was developed (**Vrije Universiteit Amsterdam**) are credited.



