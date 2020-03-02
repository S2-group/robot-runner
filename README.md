# Robot-Runner
This framework enables automated experiment execution on ROS devices, both on physical (**real-life**, hereon after called **native**) robots and in a simulated setting (**Gazebo**, hereon after referred to as **sim**).

Robot-runner is developed to work with both **ROS1**, tested with **Kinetic** and **Melodic**, and **ROS2** , tested with **Eloquent**. Please carefully read and follow this README for the correct setup for your use case.

Robot-runner is developed and tested using **Python 3.6.9**, hereon after referred to as **Python**. Any other version of Python can possibly be used, but this has not been tested nor is it recommended.

Robot-runner is developed specifically for experimenting with a Turtlebot3 Burger, as this is the native ROS device provided by the **Vrije Universiteit Amsterdam**. Therefore example experiments and install guides also involve Turtlebot3 install guides. However, robot-runner is, as said before, developed to be used with **any** ROS device and thus can also be used without any Turtlebot software present. This would however mean that the example experiments provided, with the exception of the native experiments, are unusable. 

Read this README therefore carefully and adjust the install to your own personal needs.

> **Warning:**
>
> Currently **ROS2 Native experiments** are **NOT WORKING**. **ROS2 Sim experiments ARE WORKING** however. ROS2 Native experiments are not working since the recommended image for the Raspberry Pi in combination with ROS2: **Ubuntu 18.04 Server image for ARM processors** has a **known bug** in the Linux kernel. The Raspberry Pi gets too hot under any significant load and tries to throttle the CPU, however this results in a **hardware interrupt** from which the system is **unable to recover**, leading to a **crash**.
>
> **This issue is being worked on.**

# Intro

Robot-runner, has been developed out of a need to be able to perform academic experiments repeatedly and reliably, while preventing as much interference and manual labor as possible. With as end goal that these results can be used to perform academic research and draw scientifically sound conclusions.


Robot-runner has been developed by **Stan Swanborn** in early 2020 under the supervision of **Ivano Malavolta** as part of the course **Individual Systems Practical** as part of the ***Master's degree* Computer Science** at the **Vrije Universiteit Amsterdam**.

Robot-runner is specifically developed to be able to do research into the energy efficiency of architectural tactics for ROS devices (experimented specifically on a **Turtlebot3 Burger**) however, the framework is developed to be versatile and can be used for any sort of experiment or just any automated sequence of actions on any ROS device (native or simulated).

# Table of Contents

This README is an elaborate and therefore somewhat long description of robot-runner. However, not all information might be of interest to all readers. Therefore the information given in this README is presented here in a clear overview, with context to whom this may be of interest.

| Content title                                       | Content                                                      | To whom it may concern                                       |
| --------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| [Short description](#short-description)             | An overview of what is present in this repository and why.<br />A simplified description of robot-runner's workings. | General user of robot-runner.                                |
| [Install guide](#install-guide)                     | An elaborate guide to set up everything needed to start using robot-runner in any use case. It also provides easy verififcation of a successful install. | General user of robot-runner.                                |
| [Native install](#native-install-guide)             | An elaborate guide to set up everything needed to start using robot-runner with native ROS devices. | General user of robot-runner, specifically interested in **native experiments**. |
| [Test successful install](#test-install-guide)      | A set of tests with expected output that help the user determine if the install was correct and successful. | General user of robot-runner.                                |
| [User guide](#user-guide)                           | An elaborate guide as to how to use robot-runner and set up a succesfull experiment for any use case, including native experiments. | General user of robot-runner.                                |
| [Example guide](#example-guide)                     | A guide to understand the provided examples better and to know how to setup the host system environment correctly for experiments. | General user of robot-runner.                                |
| [Technical documentation](#technical-documentation) | Extensive technical documentation of the inner-workings of robot-runner and the rationale behind design decisions. | Any user or developer in need of a technical understanding of robot-runner. |
| [Troubleshooting](#troubleshooting)                 | Common issues and possible fixes.                            | General user of robot-runner.                                |

<a name="short-description"/>

# Short description
## Contents of repository

Everything needed to be able to run robot-runner for any use case is present in this repository. The table below gives a good indication of what is present, why and what it is responsible for.

| Module of interest            | Description                                                  |
| ----------------------------- | ------------------------------------------------------------ |
| **ClientRunner**              | Robot-runner client that needs to run on a native ROS device in case native experiments need to be performed. |
| ClientRunner/Scripts          | Contains scripts used by the robot-runner native client, **not interesting for a general user**. |
| ClientRunner/config.json      | A very simple, two line, config file which specifies a **ROS launch file** or an **entire ROS launch command** which will be executed each run. |
| ClientRunner/\__main__.py     | Run this program on the native ROS device using **Python** 3.6.9. <br> `python3 __main__.py config.json` |
|                               |                                                              |
| **RemoteRunner**              | Robot-runner remote PC module, the **main application** of robot-runner. |
| RemoteRunner/\__main__.py     | Run this program on the remote PC using **Python** 3.6.9. <br/> `python3 __main__.py config.json` |
| RemoteRunner/Controllers      | Contains all controllers used by robot-runner to control experiment execution, run execution and ROS communications. <br />**All application logic** |
| RemoteRunner/Models           | Contains all models needed during execution of robot-runner. <br />**Only model representations, minimal logic** |
| RemoteRunner/Procedures       | Contains all procedures defined in one place. <br />**All code which does not need or cannot be provided as part of a class or object (code that does not adhere to OOP-Principles** |
|                               |                                                              |
| **Examples**                  | Contains experiment examples, **native and sim**, for ROS1 on both Ubuntu 16.04 and Ubuntu 18.04. <br><br>Contains experiment examples, **native and sim**, for ROS2 on Ubuntu 18.04. |
| Examples/**ROS1**/Experiments | Contains the experiment examples both for a native ROS1 experiment and a simulation experiment using Gazebo for both Ubuntu 16.04 and Ubuntu 18.04.<br><br>**NOTE:** For any experiment example; **start with looking at the config.json** with the understanding as given in the [user guide](#user-guide). |
| Examples/**ROS1**/Scripts     | These scripts were used by the developer to gain understanding on the workings of ROS1 and Gazebo. Also how to use these scripts while running robot-runner experiments. They were vital to understand the modularity and node- and topic-based nature of ROS. Therefore they are also provided as part of the robot-runner package. |
| Examples/**ROS2**/Experiments | Contains the experiment examples both for a native ROS2 experiment and a simulation experiment using Gazebo.<br/><br/>**NOTE:** For any experiment example; **start with looking at the config.json** with the understanding as given in the [user guide](#user-guide). |

## General description of robot-runner

Robot-runner is as versatile as it can be, this has as a consequence that it can be used in many different ways, leading to many different use cases. However, this README is written in the context of someone that wants to perform an academic, scientifically sound, automated ROS experiment. Using this context, this README explains to the user how to set up such an experiment. Since there are so many ways a user can start doing that, the most defining factors for any experiment are explained here. The highly detailed, in-depth explanation is given in the [user guide](#user-guide).

Considering these factors use some implementation specific terms which are needed for explaining them, some terms are introduced before they are explained. In any case that this happens, the introduced term links to the section in which it is explained in detail.

This short description is given **before** any install guide, considering a **correct understanding** of the **global workings** of **robot-runner** and its use cases are **vital** for knowing **what** to **install** and **why**.

Any robot-runner experiment can be defined as a combination of these factors:

**<ros_version> | \<sim or native\> | <run_script or not> | \<timed or programmatic stop\>**

| Factor                     | Value                                                        |
| -------------------------- | ------------------------------------------------------------ |
| ros_version                | ROS1 (**Kinetic** on Ubuntu 16.04, **Melodic** on Ubuntu 18.04) <br />ROS2 (**Eloquent**, only Ubuntu 18.04) |
| sim or native              | Running an automated experiment on a simulation or on a native device |
| run_script or not          | Experiment actions defined in a separate, controllable, run script or not<br><br>**NOTE:** If the experiment is not defined in a **run_script** robot-runner **assumes** it is defined somewhere else, which is then **spawned** as part of a **ROS launch file** specified either on the **remote PC** or on the **native device**. |
| timed or programmatic stop | Experiment run ends after a set timeout or when the run_completed ROS topic is published. |

These factors can be set and manipulated by the user in the **config.json** that is described in the [user guide](#user-guide). Here the focus is to give a **global, simplified understanding** of robot-runner. With this simplified knowledge a **general idea** of **combination of factors** needed for **any** specific **use case** can be derived. Considering the user has most **probably** already a **use case** in **mind**, a **correct** **install** and experiment setup can be **chosen** and **followed** using the understanding gained from this short description.

#### ros_version

The ROS version that robot-runner uses is the version that is currently activated on the system robot-runner runs on. Any system can have multiple installs of multiple ROS versions (multiple ROS1 installs, multiple ROS2 installs, all side-by-side), however only one ROS install can be active at a time. This one, active install, sets its version in the environment variable **ROS_VERSION**, the value of this environment variable is used by robot-runner to correctly determine the currently running ROS instance.

This applies to both the RemoteRunner (remote PC) and ClientRunner (native device) modules of robot-runner.

The ROS version is deliberately kept from the **config.json** to prevent **human error**; specifying ROS1 while a ROS2 install is activated.

#### sim or native

How robot-runner will set up any experiment and its communication with the ROS device is directly determined by the user setting in the **config.json**. There, the user is able to set the field: **use_simulator** to either **true** or **false**. When this field is **false**, a **native experiment** will be executed instead. This needs a correct setup and a running **ClientRunner** instance on the native ROS device. A detailed install guide for native devices is given [here](#native-install-guide).

#### run_script or not

Robot-runner is specifically designed to be as versatile as possible and to only reduce the manual overhead needed to run an experiment. Therefore, it leaves any definition of an **experiment** (***the definition of the behavior of the (simulated) ROS device***) to the user. 

The user has many options available as to **where** to **define** the **experiment**, provided natively by ROS. The user can, for example, define a node which manipulates a **basic client** on the ROS device. This node can be programmed in **any language** supported by the **ROS library**, and this **node** and the **basic client** can be launched from a **launch file**. This launch file can then be given to robot-runner in the **config.json**, resulting in robot-runner launching that specified launch file each run of the experiment.

However, when the node is programmed in **Python** it can also be **kept** from the **launch file** and directly specified to robot-runner as the run script. This can be done by giving the path to the file in the **config.json** under the **run_script** field. This option has the advantage that **arguments** can be passed to the run script, these can also be specified in the **config.json**.

It also offers the user the capability of doing additional logic, before launching the **actual** experiment definition. This actual definition could be spawned from the run_script specified to robot-runner.

**Concrete scenario:** when the user runs a native experiment, the experiment can be defined on the (simulated) ROS device as its behavior. However, this moves the variability of experiment definitions to the (simulated) ROS device instead of the remote PC. Instead, the user could choose to develop one client, which can be manipulated by any other ROS node. This node, or these nodes, as specified in a Python file are then considered to be the run script. This script can then be specified to robot-runner. To run different experiments, the user would then only need to specify different run scripts; moving the **added memory** and **changing of experiments** (*changing config files*) to the **remote PC** instead of the (simulated) ROS device.

#### timed or programmatic stop

An experiment consists of a specified number of replications, called runs. Each run needs to be stopped after being started, this can be done using a timeout (a set number of milliseconds in the **config.json** under the field: **duration**)  or using a programmatic stop. The programmatic stop is automatically enabled by robot-runner when the **duration** field is set to **0**.

The **programmatic stop** has robot-runner initialize a ROS node (*poll_run_complete*) which subscribes to the topic: **/robot_runner/run_completed**. When a message of type **std_msgs.msg.Bool** *<u>Bool(True)</u>* is **published** to the **topic**, the run is **automatically** **stopped**. This is **deliberately designed** to be a **topic**, as they are **asynchronous** and **non-blocking**. This means that any experiment node which publishes True does not have to **wait** for robot-runner to respond but can **close immediately**, ensuring a **minimal impact** of the robot-runner **overhead** on any gathered **statistics**.

<a name="install-guide"/>

# Install guide

Robot-runner has been developed mainly using **Ubuntu 18.04**. However, it has also been tested and developed on Ubuntu 16.04, albeit with an older ROS1 version: **Kinetic**, as this is the last ROS1 version to be guaranteed on Ubuntu 16.04. The install options available are thus:

| Ubuntu           | ROS1 Latest version | ROS2 Latest version | Use case                                                     |
| ---------------- | ------------------- | ------------------- | :----------------------------------------------------------- |
| Ubuntu 16.04     | Kinetic             | NONE                | In case any software used is dependent on Ubuntu 16.04. This install should be considered **Legacy support**. |
| **Ubuntu 18.04** | **Melodic**         | **Eloquent**        | **Recommended install**, ROS1 and ROS2 work with latest versions. Robot-runner is mainly developed and tested on this Ubuntu distribution. |

> **WARNING:**
>
> Robot-runner comes with a set of example experiments. Since robot-runner is specifically designed to experiment with a **Turtlebot3 Burger**, these examples need **Turtlebot3 packages** to be installed with it. This is mentioned in each install guide, if you do not need or want these examples to work, then only follow the ROS install guide. 
>
> Considering the Turtlebot3 simulation does not provide any **battery simulation**, a **Gazebo plugin** is provided with each example. This plugin is developed by the **CMU** (Carnegie Mellon University), and used and modified for this project under the **MIT license**.
>
> Examples that use this battery plugin are called VU_BATTSIM_1\*.04 (where ***** = 6 or 8, (16.04 or 18.04)). Please note that only **Ubuntu 16.04 fully supports** the original plugin with motor power taken into account in the battery discharge simulation. Ubuntu 18.04 does not provide this functionality, it thus only provides **linear discharge**, as **legacy software** used is not supported on 18.04.
>
> This is however of **no, or slight, importance** to the user, the examples are only just that. The recommended install is therefore still Ubuntu 18.04.
>
> **NOTE: The VU_BATTSIM battery simulation is only available for ROS1.**

### Recommended setup guides:

**ROS1 (Melodic) install guide**: http://wiki.ros.org/melodic/Installation/Ubuntu

​	If **Turtlebot3 packages** are required for **your use case**, follow the ROBOTIS guide as well: (**PC Setup**)

​	Follow from step **1.3**:

​	http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc-setup

**Important:** To be able to run the **VU_BATTSIM** example; go into the correct **VU_BATTSIM** experiment folder, corresponding to your Ubuntu version, and go into the install folder and run: `install_vu_battsim.sh`

**ROS2 (Eloquent) install guide**: https://index.ros.org/doc/ros2/Installation/Eloquent/

​	If **Turtlebot3 packages** are required for **your use case**, follow the ROBOTIS guide as well: (**PC Setup**)

​	Follow from step **1.3**:

​	http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/

### An example of a correct ~/.bashrc for a remote PC

```bash
# General	
# Remove this line when you are not using a Turtlebot3
# Or change it to the Turtlebot3 model you are using
export TURTLEBOT3_MODEL=burger

## ROS 1
source /opt/ros/melodic/setup.bash
source /home/stanswanborn/catkin_ws/devel/setup.bash # Source any ROS1 packages (OPTIONAL)

## ROS 1 - VU_BATTSIM Env. Variables (OPTIONAL)
export VU_BATTSIM=/home/'<user>'/robot-runner/examples/ros1/experiments/vu_battsim_18.04
export GAZEBO_PLUGIN_PATH=:/home/'<user>'/catkin_ws/src/vu_gazebo_battery/build/devel/lib

## ROS 2
export ROS_DOMAIN_ID=30 #TURTLEBOT3, remove when you are not using a Turtlebot3 (OPTIONAL)
source /opt/ros/eloquent/setup.bash

# ROS2 - Turtlebot3 model sourcing (OPTIONAL, remove when not using a Turtlebot)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
```

<a name="native-install-guide"/>

# Native install guide

**NOTE:** This **step is only necessary** when you want to perform a **native experiment**. If you are only interested in **simulating** a ROS device, then your install is already complete. That is if the [tests](#test-install-guide), that apply to your use case, were successful.

**NOTE:** As the author was provided a **Turtlebot3 Burger**, featuring a Raspberry Pi 3 model B+, this guide is written in the context of using that specific robot.

To be **able** to perform automated experiments using **robot-runner**, the native ROS device has to run **Python 3.6.9** and a supported **ROS** version. From this point on, the install guide will make recommendations on what to do and what will fail based on the aforementioned Turtlebot3 Burger install experience. Therefore, if your device is in any way significantly different, **this guide might be useless to you**.

**In that case**, know that the **only requirements** to be able to perform automated experiments using robot-runner, is to be able to **run** the **ClientRunner** module, which requires **Python 3.6.9**. and have a **supported ROS version** installed on the native device. If you make sure these requirements are met, then you can proceed to the [user guide](#user-guide).

> **WARNING:**
>
> As described before, ROS2 native experiments are not supported nor working at this time. This is the case as for this project the native ROS device consists of a **Turtlebot3 Burger.** To be able to successfully use this robot, **Turtlebot3 packages** have to be build and installed next to ROS2. It is at this step that most installs failed on most images, but using the recommended image by ROBOTIS the build succeeded.
>
> However, while running ROS2 on this recommended image, the **Raspberry Pi 3 model B+** **overheats** and has to throttle down the CPU. This triggers a **hardware interrupt** from which the system **does not recover**, resulting in a **crash**.
>
> **This issue is being worked on.**

## Choosing the right image

Choosing the right image for your use case is of high importance to guarantee a successful setup, which can run as smooth as possible. When using a Turtlebot3, a set of recommended images are provided by ROBOTIS, the creators of Turtlebot. For this guide however, multiple images have been tested, also images that have not been mentioned by ROBOTIS to see if they would work. Based on these findings the following table has been created:

| Raspberry Pi image                                           | ROS1 Install      | ROS1 Running | ROS2 Install | ROS2 Running | Turtlebot3 packages installed | Recommended for use with             |
| ------------------------------------------------------------ | ----------------- | ------------ | ------------ | ------------ | ----------------------------- | ------------------------------------ |
| [ROBOTIS Raspbian](http://www.robotis.com/service/download.php?no=1738) | **PRE-INSTALLED** | **YES**      | **NO**       | **NO**       | **ROS1 YES<br />ROS2 NO**     | **RECOMMENDED FOR ROS1**             |
| [Ubuntu MATE 32-bit (Ubuntu recommended)](https://ubuntu-mate.org/download/) | YES               | YES          | NO           | NO           | ROS1 YES<br />ROS2 NO         | Can be used with ROS1                |
| [Ubuntu MATE 64-bit (Ubuntu experimental)](https://ubuntu-mate.org/download/) | YES               | YES          | YES          | YES          | ROS1 YES<br />ROS2 NO         | Can be used with either ROS1 or ROS2 |
| [Ubuntu 18.04.4 Server image (arm64)](https://ubuntu.com/download/server/arm) | **YES**           | **YES**      | **YES**      | **YES**      | **ROS 1 YES<br />ROS2 YES**   | **RECOMMENDED FOR ROS2**             |

To further elaborate on the table's information:

- **Raspbian** is recommended for use with **ROS1** as it is the most lightweight, best running image on the Raspberry Pi 3 model B+ and comes with all ROBOTIS Turtlebot3 packages pre-installed.
  - In case you do not need the Turtlebot3 packages; Raspbian is still **strongly recommended**, but then downloaded and installed as a clean install from https://www.raspberrypi.org/downloads/.
  - **ROS2 Requires 64-bit** and is therefore not supported on any Raspbian distribution.
- **Ubuntu MATE 32-bit** can be important to those users that want **debian package manager support (dpkg)**. In that case, it can be successfully used with **ROS1**.
- **Ubuntu MATE 64-bit** can be used with either **ROS1 or ROS2**. 
  - May experience issues installing and building the Turtlebot3 packages.
  - If the image is not used with ROS2 but with ROS1, it is **strongly discouraged** to use this image. This is because running a 64-bit images proves to be hard for the Raspberry Pi 3 model B+ and the general performance of the image was significantly lower than that of Raspbian.
- **Ubuntu 18.04.4 Server image (arm64)** can be used with either **ROS1 or ROS2**.  
  - This is the **recommended image** for ROS2 by ROBOTIS, the creators of Turtlebot. 
  - Running ROS2 on the image proved impossible as a hardware interrupt to throttle the CPU crashes the system. 
  - It remains the **"recommended for ROS2"** image, as it is recommended by ROBOTIS and considering the fact that ROS2 was not successfully running on any image.

## Installing ROS

When the right image for your use case has been successfully installed on your ROS device, you are ready to start installing ROS.

### ROS1

Follow this setup guide for installing **ROS1** (**Melodic**): http://wiki.ros.org/melodic/Installation/Ubuntu

​	If **Turtlebot3 packages** are required for **your use case**, follow the **ROBOTIS** guide as well:

​	http://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup

​    In this guide, follow **SBC Setup** and **OpenCR Setup**.

#### ROS1 - An example of a correct ~/.bashrc for native ROS device

After installing ROS1 on the native device, a correct ~/.bashrc file should look like this:

```bash
# ROS1
source /opt/ros/melodic/setup.bash
source /home/'<user>'/catkin_ws/devel/setup.bash
```

### ROS2

**Not supported at this time.**

<a name="test-install-guide"/>

# Test successful install:

## Test any ROS install for correct installation:

To check whether your ROS environment, either ROS1 or ROS2, is correctly installed you can use the command:

```bash
printenv | grep ROS

# Indication of correct ROS1 Output:
ROS_VERSION=1
ROS_PYTHON_VERSION=2
ROS_DISTRO=melodic

# Indication of correct ROS2 Output:
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=eloquent
```

## Test a successful sim install (remote PC):

**NOTE:** By default there are no test scripts provided with any ROS1 or ROS2 install to test if the Gazebo simulator in combination with the ROS install is working correctly. These examples are only provided by the **Turtlebot3 packages**. Therefore this section describes how to test a successful sim install with these Turtlebot3 packages.

#### ROS1:

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

A successful Gazebo launch showing an empty world with a Turtlebot3 present should now be displayed. When running `rosnode list` or `rostopic list` Turtlebot3 nodes and topics should be present. To check if any data is being published by using `rostopic echo /topic_name`

#### ROS2:

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

A successful Gazebo launch showing an empty world with a Turtlebot3 present should now be displayed. When running `ros2 node list` or `ros2 topic list` Turtlebot3 nodes and topics should be present. To check if any data is being published by using `ros2 topic echo /topic_name <topic_data_type>`

## Test a successful native install (remote PC & Native ROS device):

#### ROS1:

Follow these steps too ensure a correct working and install of both the remote PC and the native device.

**On the remote PC**, run roscore:

```bash
roscore
```

**On the native ROS device**, <u>run a simple node publishing some data to a topic</u>. This can be any node, publishing any kind of data. **When using a Turtlebot3, a simple example is already provided. Use the following command:**

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

**Now on the remote PC**, open another terminal and run these two commands:

```
rostopic list
rosnode list
```

In the output a Turtlebot3 node should be seen and Turtlebot3 topics, like **/battery_state** should be observed and publishing. To check wether a topic is being published, run the following command:

```
rostopic echo /topic_name
```

#### ROS2:

**Unsupported at this time.**

## Test ROS1 with VU_BATTSIM for correct functionality:

From the vu_battsim_1*.04 directory in a terminal:

``roslaunch launch/vu_battsim.launch``

You should now observe:
 - The terminal should launch a ROS master
 - The [VU_BATT] Gazebo plugin should only output in green
 - A Gazebo instance should now be running, showing a Turtlebot3 Burger in a house.
 - The ROS topic `/mobile_base/commands/charge_level` should be available (published)

<a name="user-guide"/>

# User guide

In this section, an elaborate user guide will be given as to how to set up any experiment with any combination of use cases. Important aspects will be described such as:

* How to setup, run and collect data from a correct experiment for any use case
* How to process this data and be able to analyze this data

## Running robot-runner

To start, robot-runner consists of two main modules: **RemoteRunner** and **ClientRunner**.

**NOTE:** Robot-runner blocks most of the output of its subprocesses. It does so because the terminal would otherwise be flooded with info messages which are not of any particular value when all goes well. 

The error messages, if anything goes wrong, should be outputted to the terminal. However, that might not always be the case. When anything goes wrong **directly** related to **robot-runner**, it **will always output** the error messages, but subprocesses might not show all their **info / warning / error** messages.

These messages, however, can be crucial for solving the issue. Therefore robot-runner offers a **verbose** mode, in which all subprocesses output their `stdout` to the robot-runner terminal. In order to use the **verbose** mode, run either the **RemoteRunner** or **ClientRunner** module with the following command:

```bash
python3 __main__.py --verbose /path/to/config.json
```

### Running RemoteRunner

**RemoteRunner** must be ran on the remote PC, overseeing any experiment. This can either be an experiment executed on the remote PC itself in the form of a simulation, or an experiment involving a native ROS device. In the latter case the name 'remote PC' makes the most sense.

When speaking about robot-runner, this is to module that is considered. It is the **main application**.

This program can be run by going into the **RemoteRunner** directory and running:

```bash
python3 __main__.py /path/to/config.json
```

Where the only argument given is the path to the config.json file in which all user-defined settings reside.

### Running ClientRunner

**ClientRunner** must be ran on the native ROS device. This lightweight robot-runner module is only necessary when native experiments need to be performed and makes sure that each experiment run is cleanly shutdown and restarted on each replication. It will use a **smaller config.json** file which is explained in further detail later in this section. In order to run the **ClientRunner** module use the following command while in the **ClientRunner** directory.

```bash
python3 __main__.py /path/to/native_config.json
```

## Explaining the config file

Any experiment starts with a **config.json** file, which will be given to the **RemoteRunner** module of robot-runner. This config.json file has to specifically follow the following format:

```json
# EXAMPLE OF A CONFIG FILE FOR A 
# ROS2 SIMULATION EXPERIMENT
{
  "use_simulator": true,
  "name": "ros2_test",
  "replications": 3,
  "duration": 5000,
  "launch_file_path": "/home/stanswanborn/robot-runner/examples/ros2/experiments/house_world/launch/turtlebot3_house.launch.py",
  "run_script": {
    "path": "",
    "args": {
    }
  },
  "nodes_must_be_available": [
    "/gazebo"
  ],
  "topics_must_be_available": [
    "/clock"
  ],
  "output_path": "/home/stanswanborn/experiment_output",
  "topics_to_record": [
    "/clock"
  ],
  "time_between_run": 1000
}
```

Each field has been given a clear name. However, to give a more formal and clear overview the fields above are represented in the table below with expected data types, what these expected values have as a consequence (if any) and a requirement specification.

| Field name               | Expected value(s)                                            | Requirement      |
| ------------------------ | ------------------------------------------------------------ | ---------------- |
| use_simulator            | **boolean**<br /><br />Either True or False. When **true** the simulator is used, when **false** the native ROS device is expected to be running the **ClientRunner** module at the time that robot-runner is started. | **NOT_NULL**     |
| name                     | **string**<br /><br />The name of the experiment, used in naming the experiment output folder as described in the **output_path** field explanation. | **NOT_NULL**     |
| replications             | **int**<br /><br />The number of experiment runs             | **NOT_NULL**     |
| duration                 | **int**<br /><br />A set timeout after which a run is killed.<br /><br />**When set to: 0** runs indefinitely, until the ROS topic: **/robot_runner/run_completed** is published **TRUE** | **NOT_NULL**     |
| launch_file_path         | **string**<br /><br />The path to the .launch (ROS 1) or .launch.py (ROS 2) launch file which will spawn all necessary nodes for the experiment.<br /><br />**Can be empty** (If left empty, robot-runner assumes a run_script has been specified from which the user can manually launch the necessary nodes for the experiment OR when running a **native experiment**, robot-runner expects the native ROS device to launch the necessary nodes for manipulation.) | **not required** |
| run_script               | **array**<br /><br />The array containing the **path** and the possible **args (arguments)** for running a user-defined, Python 3.6.9, .py run_script. | **not required** |
| run_script/path          | **string**<br /><br />The path to the .py file of the run_script, which will be run each experiment run. Here the user can specify custom logic.<br /><br />**Can be empty** (If left empty, robot-runner will not run a run_script and will not consider any run_script arguments that are possibly given.) | **not required** |
| run_script/args          | **array of strings**<br /><br />An array of strings, each string is a new argument. Arguments should be specified in the order in which they are required by the run_script.<br /><br />**Can be empty** (If left empty, robot-runner will run the run_script (if present) without any arguments.) | **not required** |
| nodes_must_be_available  | **array of strings**<br /><br />An array of strings, each string is a ROS node_name which is required to be present. If one of these node names is not present, robot-runner will not start the experiment run.<br /><br />**Can be empty** (If left empty, robot-runner will not wait for any nodes to be present before starting the experiment run.) | **not required** |
| topics_must_be_available | **array of strings**<br /><br />An array of strings, each string is a ROS topic_name which is required to be present (being published). If one of these topic names is not present (being published), robot-runner will not start the experiment run.<br /><br />**Can be empty** (If left empty, robot-runner will not wait for any topics to be present before starting the experiment run.) | **not required** |
| output_path              | **string**<br /><br />The path in which, for each experiment, a folder is created using the given **name** from the **name field**. Following the format: **name-DAY_MONTH_YEAR-HOUR_MINUTE_SECOND**. | **NOT_NULL**     |
| topics_to_record         | **array of strings**<br /><br />An array of strings, each string is a ROS topic will be recorded by robot-runner using **rosbag or ros2bag**, corresponding to the used ROS version. These recorded topics will then be stored in the experiment output folder as mentioned above. | **NOT_NULL**     |
| time_between_run         | **int**<br /><br />The time in milliseconds that robot-runner sleeps between runs (can be 0). | **NOT_NULL**     |

## Setup an experiment for your use case

Using the knowledge gained from following this README up to this point will make setting up an experiment very easy. Since robot-runner is so versatile it is difficult to list every possible combination of settings, the most common and useful combinations are set out here to help you on your way:

### Setting up a simulation experiment

A simulation experiment always needs the field **use_simulator** set to the value **true**. It needs some way to start a Gazebo simulation. This can be using the launch file as specified by the **launch_file_path** field, or when this is left empty, robot-runner expects a run_script to be present as specified by the **run_script** object containing the **path** and **optional arguments**.

When a launch file, launching the Gazebo simulation, is not given, the run_script is given the responsibility to start the Gazebo simulation. This enables the user to perform custom logic using **Python** before launching running the experiment, instead of only being able to specify launch nodes in a launch file.

Launching the experiment can, in case a run_script is used, be done by yet launching the launch file, or by launching each node / controller / file specified in the launch file by hand using **Python**. 

This gives the user a lot of variability, control and ease-of-use. Launching an experiment can be made as complex as necessary.

**All other settings can be set according to your very specific use case, as long as they adhere to the requirements as set out above.**

### Setting up a native experiment

A native experiment always needs the field **use_simulator** set to the value **false**. It also does not need any launch file specified in the **launch_file_path** field. In case you as a user need to launch specific nodes you can do so by launching them from the run_script or launching a .launch file from the run_script.

A native experiment does not necessarily need a **run_script** specified. It does however need a run_script specified, if the experiment consists of performing custom logic on the remote PC side, or of manipulating the native ROS device.

**Important:**

An extra step for native experiments is setting up the experiment on the native ROS device and running the **ClientRunner** module of robot-runner.

The robot-runner repository needs to be cloned on the native ROS device in its totality, as the ClientRunner module is dependent on some modules present in the **RemoteRunner** module.

When running the `__main__.py` main application file, of the **ClientRunner** module, a config file needs to be specified. This config file follows a different, very simple format:

```json
# EXAMPLE OF A CONFIG FILE FOR A 
# ROS1 NATIVE EXPERIMENT
# CONFIG FILE ON NATIVE DEVICE

{
  "launch_command": "roslaunch turtlebot3_bringup turtlebot3_robot.launch",
  "launch_file_path": ""
}
```

**NOTE:** Only one field should be given in the config file, the other field should always remain empty: `""`.

The user can specify either an entire launch_command: consisting of a `roslaunch` or `rosrun` command, launching a .launch or python file. This file should launch (a) node(s) which will be responsible for listening to manipulations by the remote PC or which perform their pre-programmed experiment behavior without any or little manipulation. This is entirely up to the user's use case, everything is possible.

This / these node(s) are then also responsible for publishing the ROS topics that are being recorded by the **RemoteRunner** module of robot-runner, which uses the **standard, longer config.json** as specified in the beginning of the user-guide.

When the user has specified a **duration** of **0** in the standard config.json (**RemoteRunner config**), then either the run_script on the remote PC or the native ROS device is responsible for stopping the experiment run. This is done by publishing the message: **std_msgs.Bool(True)** to the topic: **/robot_runner/run_completed**. In the next section of this user-guide, the programmatic run stop is explained in high detail.

### The timeout or programmatic run stop

During any experiment run robot-runner needs to know when the run has ended or needs to end. This can be done after a certain timeout, if the use case needs the experiment just to run but be terminated after a certain time. This can be done by setting the **duration** field of the **RemoteRunner config.json** to any value **> 0**. This value needs to be given in **milliseconds**.

When the **duration** field is set to **0**, robot-runner will subscribe, listen and wait on topic **/robot_runner/run_completed** to be published with the message **std_msgs.Bool(True)**. After receiving such a message, robot_runner will stop the run and perform the next (if there is a next run, defined by the field **replications**).

**In any case**, robot-runner will cleanly exit all nodes, simulators etc. A clean shutdown of an experiment run is therefore carried out by robot-runner and is not the responsibility of the user.

**An example of how to publish a programmatic run stop message can be found in: */examples/ros1/experiments/vu_battsim_18.04/scripts/test_run.py***.

Also given here:

```python
## ROS 1 EXAMPLE
## STOPPING ROBOT_RUNNER EXPERIMENT RUN

# stop turtlebot
rospy.loginfo("Stop TurtleBot")
# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
self.cmd_vel.publish(Twist())

stop_robot_runner = rospy.Publisher('/robot_runner/run_completed', Bool, queue_size=1)
rospy.sleep(1)

while True:
    if stop_robot_runner.get_num_connections() > 0:
        stop_robot_runner.publish(True)
        break

# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
rospy.sleep(1)
sys.exit(0)
```

## Processing the gathered data

Rosbags are used to gather the data as published in the topics that are set to be recorded by robot-runner. For ROS1 the library; **rosbag** is used and for ROS2 the library; **ros2bag**. These libraries allow for easy recording, playback and processing of any data published in any topic.

After each experiment run, a folder with the following format: **run{RUN_NUMBER}** is created (f.e. **run1**). In case **ROS1** is used, a **topics.bag** file is created following the format: **topics-ros{ROS_VERSION}.bag**. In case **ROS2** is used, a sub-folder is created following the above mentioned naming format. This sub-folder then contains the **3** files that are recorded as part of the **ros2bag** library.

### ROS1

In order to play the data in the rosbag back, open a terminal in the folder in which the .bag file is present and use the following command:

First, run the following command in a terminal:

```bash
roscore
```

Then, run this command in a different terminal:

```bash
rosbag play topics.bag

# Expected output:
[ INFO] [1583169576.974043901]: Opening topics-ros1.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [RUNNING]  Bag Time: 1581605322.272591   Duration: 4.090819 / 4.149297               
Done.
```

Now, when you run `rostopic list` in yet another terminal, you can see the recorded topic being played back. This can be checked using the command `rostopic echo /topic_name`. The recorded data should then echo to the terminal.

**Converting to R**

In case you want to easily convert the rosbag data into a .txt file for data processing (e.g. **R Studio**), use the following command:

```
rostopic echo -b topics.bag -p /mobile_base/commands/charge_level > data.txt
```

The generated .txt file should now contain the recorded data and can be used for data processing.

### ROS2

In order to play the data in the ros2bag files back, open a terminal in the folder in which the 3 ros2bag files are present and use the following command:

```bash
ros2 bag play topics-ros2_0.db3

# Expected output:
[INFO] [rosbag2_storage]: Opened database 'topics-ros2_0.db3' for READ_ONLY.
```

After the expected output is shown, you can use the command `ros2 topic list` and `ros2 topic echo /topic_name /topic_data_type` to see the topics recorded with ros2bag being published again. This allows for playback of the exact data without having to run the experiment again. Using this you can do real-time measurements over and over again, simulating the experiment.

**Converting to R**

In case you want to easily convert the ros2bag data into a .txt file for data processing (e.g. **R Studio**), use the following command while playing the ros2bag file with the aforementiond command:

```bash
ros2 topic echo /topic_name /topic_data_type > data.txt
```

The generated .txt file should now contain the recorded data and can be used for data processing.

<a name="example-guide"/>

# Example guide

Robot-runner comes with multiple examples, as has been apparent from this README. In this section these examples are elaborated on and explained what each specific example demonstrates and what the user can learn from them.

## ROS Sourcing

**IMPORTANT: Correct ROS sourcing, as given below, applies to ALL examples.**

For each example it is essential that the correct ROS version is sourced for them to be able to run. Let's say, for example, that an example requires **ROS 1 to be activated on the host system**. 

This can be done and checked by looking at the ~/.bashrc file and checking that all **ROS 2 sourcing**, if present, is **commented out**. And that all **ROS 1 sourcing** is enabled. This file then needs to be sourced by the terminal, the best way to do this is to launch an entirely **new** terminal instance. As old instances might still **retain old source settings** from previously sourced ~/.bashrc files.

```bash
## ROS 1 (ACTIVATED)
source /opt/ros/melodic/setup.bash
export VU_BATTSIM=/home/stanswanborn/robot-runner/examples/ros1/experiments/vu_battsim_18.04
export GAZEBO_MODEL_PATH=:/model
export GAZEBO_PLUGIN_PATH=:/home/stanswanborn/catkin_ws/src/vu_gazebo_battery/build/devel/lib
source /home/stanswanborn/catkin_ws/devel/setup.bash

## ROS 2 (COMMENTED OUT)
#export ROS_DOMAIN_ID=30 #TURTLEBOT3
#source /opt/ros/eloquent/setup.bash
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
```

**NOTE:** Changing from ROS 1 to ROS 2 is of course the same process but then reversed. For each example you need to make sure the correct ROS version is activated on your system.

## ROS1 / Experiments / vu_battsim_16.04 						&& ROS1 / Experiments / vu_battsim_18.04

This example experiment should be used only on the corresponding Ubuntu version. If the [install guide](#install-guide) was correctly followed and the vu_battsim plugin has been successfully installed, this example can be used.

This example demonstrates the following significant aspects:

* Using the simulator (**Gazebo**)
* 2 replications
* Launches a **custom .launch file** from within the vu_battsim_1*.04 directory.
  * Please check this file to see what a ROS 1 .launch file could look like.
* Launches a **run_script** from within the directory.
  * This example does not use any **duration** but uses the **programmatic run stop**.
* **No nodes** have to be present to start the run
* **One certain topic** must be available to start the run.
  * **/mobile_base/commands/charge_level**
    * This is the topic published by the vu_battsim plugin.
* The above mentioned topic is also the topic being **recorded to a rosbag (.bag) file.**

After sourcing and activating the correct ROS version, the example can be easily run with the following command from the directory **RemoteRunner**:

```bash
python3 __main__.py ../examples/ros1/experiments/vu_battsim_16.04/config.json
```

## ROS1 / Experiments / native_experiment

 This experiment can be easily run and only needs a config.json without any .launch file specified. The .launch file for this experiment should reside on the native ROS device, which is being launched by the **ClientRunner** module of robot-runner, running on the native ROS device.

**NOTE:** This example requires the **native ROS device** to have the **Turtlebot3 Packages** installed as specified in the [native install guide](#native-install-guide).

**NOTE:** The correct ROS version (**ROS 1**) needs to be sourced and activated on the **native ROS device** as described above.

**NOTE:** The correct ROS version (**ROS 1**) needs to be sourced and activated on the **remote PC** as described above.

To start this experiment:

1. Run from the ClientRunner directory on the **native ROS device** the following command:
   1. `python3 __main__.py config_robot.json`
2. Run from the RemoteRunner directory on the **remote PC** the following command:
   1. `python3 __main__.py ../examples/ros1/experiments/native_experiment/config.json`

The experiment should now be running and complete automatically. This example does not contain any special behavior, like moving the robot. It is however a nice test to see if on the remote PC the ROS nodes and topics can now be observed by running:

```bash
rostopic list
rosnode list
```

This example demonstrates the following significant aspects:

* Using the native ROS device
* Using a duration (of 5000ms) after which the experiment run is stopped
* Using **NO** launch file
* Using **NO** run_script, as a programmatic stop is not required.
  * A programmatic stop could still be used, but then the **native ROS device** would have the responsibility of programmatically stopping the experiment run.
* The node **/Turtlebot3_core** needs to be present before the run will start.
* The topic **/battery_state** needs to be present before the run will start.
* The wait time between runs is 5000ms.

## ROS2 / Experiments / house_world

This experiment demonstrates a simplified version of the VU_BATTSIM example but then for ROS 2. It does not use the VU_BATTSIM plugin as that only supports ROS 1. However, it demonstrates how to setup an experiment which launches a specific world file with a robot model present. It can be used to learn how to set this up for ROS 2 and what differs from the ROS 1 example. 

**NOTE:** The correct ROS version (**ROS 2**) needs to be sourced and activated as described above.

To run this example run the following command from the **RemoteRunner** directory:

```
python3 __main__.py ../examples/ros2/experiments/house_world/config.json
```

This example demonstrates the following significant aspects:

* Using ROS 2 and the simulator (Gazebo)
* Spawning a robot model in a world with a house model
* Stopping the experiment run with a duration of 5000ms
* Launching a .launch.py file (ROS 2)
  * Please check this file to see what a .launch.py file could look like.
* **NO** run_script is specified
* The **/gazebo** node needs to be present before an experiment run is started.
* The **/clock** topic needs to be present before an experiment run is started.
* The **/clock** topic is recorded to a **ros2bag** file (**.db3**)

## ROS2 / Experiments / native_experiment

**NOT SUPPORTED AT THIS TIME.**

<a name="technical-documentation"/>

# Technical documentation

This section is only of interest to those that wish to understand the inner-workings of robot-runner or wish to expand robot-runner. It is therefore mainly for Software Engineers or those that have such skills. Any copying, alteration or redistribution of this software package (robot-runner) is allowed as it is provided under the MIT license. As long as the original author (**Stan Swanborn**), the supervisor (**Ivano Malavolta**) and the institution for which it was developed (**Vrije Universiteit Amsterdam**) are credited.

This section offers a very short, general but powerful overview of the structure of robot-runner and what each section is responsible for. The code-base itself contains **very elaborate comments** to aid programmers in navigating the application and adding any logic in the correct place. This section can be considered formal as the elaborate comments in the code-base will be enough for most.

An example of the elaborate comments, as present in the code-base:

```python
###     =========================================================
###     |                                                       |
###     |                RobotRunnerController                  |
###     |       - Init ExperimentController with                |
###     |         correct configmodel loaded from path          |
###     |                                                       |
###     |       - Public function available for parent          |
###     |         to start experiment                           |
###     |                                                       |
###     |       * Add non-experiment related overhead           |
###     |         here, like validating config variables        |
###     |                                                       |
###     =========================================================
```

Any addition to the robot-runner codebase must adhere to this comment style, which is present at the top of each **Python** file, just below the **imports**. 

A ( **-** ) should be used to explain any significant **responsibility** of the module in question.
A ( ***** ) should be used to explain any significant **category of addition** which must then be added to the module in question.

## General overview

![](/home/stanswanborn/Downloads/Screenshot from 2020-03-02 19-13-13.png)

Here an overview is given, showing the various parts of robot-runner. Each component is further elaborated on below:

| Component                          | Description                                                  |
| ---------------------------------- | ------------------------------------------------------------ |
| RobotRunnerController              | This is the main entry point for robot-runner. This controller handles all application overhead (non-experiment overhead) like validating the config file / config model. |
|                                    |                                                              |
| ExperimentController               | This is the main Experiment controller, this controls the experiment execution and spawning and despawning of **RunControllers**. Based on the given **ConfigModel** it decides what kind of experiment run will be ran (sim or native).<br /><br />All experiment related overhead can be added here. |
|                                    |                                                              |
| IRunController                     | The abstract parent class for all variants of **RunControllers**. As of writing there are only two types:<br /><br />**NativeRunController** which handles a successful native experiment run according to the given **ConfigModel**.<br /><br />**SimRunController** which handles a successful sim experiment run according to the given **ConfigModel**.<br /><br />**All shared logic between these two types must be added in this abstract class.** |
| IRunController/NativeRunController | Inherits **IRunController**, this controller is responsible, and only responsible, for a successful **native** experiment run.<br /><br />Any native run specific logic should be added here. |
| IRunController/SimRunController    | Inherits **IRunController**, this controller is responsible, and only responsible, for a successful **sim** experiment run. <br /><br />Any sim run specific logic should be added here. |
|                                    |                                                              |
| Run/Scripts                        | In this folder all scripts needed for ROS communication during an experiment run should be added. These scripts are needed as separate **Python** files since the current ROS libraries: **rospy (ROS1)** and **rclpy (ROS2)** only support one initialization per spawned process.<br /><br />However, robot-runner closes all ROS instances after every run, also closing its own instances. After this the library doesn't allow for a re-initialization. Therefore separate scripts are needed, spawned on each run, which then adhere to the '**one initialization per process**' rule. |
| Run/Scripts/PollRunCompletion.py   | A Python script which, based on the activated ROS version, automatically uses the correct ROS library and polls if the run has completed.<br /><br />**This script is spawned when the programmatic run stop is set up (duration: 0 in *config.json*).** |
| Run/Scripts/PollSimRunning.py      | A Python script which, based on the activated ROS version, automatically uses the correct ROS library and polls if the Gazebo simulator is running.<br /><br />**This script is only used when use_simulator=true in *config.json.*** |
|                                    |                                                              |
| Models                             | This Python module contains all runtime models needed during robot-runner execution. |
| Models/ConfigModel                 | A Python representation of the **config.json** with minimal model-logic; loading in and verifying the **config.json** file. |
| Models/RunScriptModel              | A Python representation of the **run_script** object with minimal model-logic; easy, in-one-place defined launch command with the specified **args (arguments)**. |
|                                    |                                                              |
| IROSController                     | The abstract parent class for all ROS versions. As of writing only two ROS versions are supported:<br /><br />**ROS 1 (Kinetic and Melodic)**<br />**ROS 2 (Eloquent)**<br /><br />The correct ROS version and thus correct **IROSController** **child** is selected as robot-runner uses the **ROS_VERSION** **environment variable** as set by any correct ROS installation.<br />**All shared logic between any ROS versions must be added in this abstract class.** |
| IROSController/ROS1Controller      | Inherits **IROSController**, this controller is responsible for communicating with an activated **ROS1** install on the host system.<br /><br />Any **ROS1** specific logic must be added here. |
| IROSController/ROS2Controller      | Inherits **IROSController**, this controller is responsible for communicating with an activated **ROS2** install on the host system.<br /><br />Any **ROS2** specific logic must be added here. |
|                                    |                                                              |
| Procedures                         | This Python module contains all logic that does not need, cannot or would be easier to use when it does not adhere to the **OOP (Object-Oriented Programming) principles** used throughout this project. |
| Procedures/OutputProcedure         | This procedure contains all **static** methods that are used throughout robot-runner to **output** in a **coherent fashion** to the **terminal**.<br /><br />**All output changes or additions must be made or added here.** |
| Procedures/ProcessProcedure        | This procedure contains all **static** methods that are used throughout robot-runner to **(de)spawn** any **subprocess** or do any kind of **process manipulation**.<br /><br />**All logic involving any kind of system process(es) must be added here.** |

<a name="troubleshooting"/>

# Troubleshooting

In this section any known problems encountered during development and use of robot-runner are given, as well as a possible solution.

| Known problem                                                | Solution                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| IntelliJ not detecting any robot-runner modules at the import section. | Mark the RemoteRunner directory as source root by <u>right-clicking</u> **>** <u>mark directory as</u> **>** <u>sources root</u>. |
| After changing the activated ROS version, this error message is displayed: `This might be a ROS 1 message type but it should be a ROS 2 message  type. Make sure to source your ROS 2 workspace after your ROS 1  workspace.` | 1. Check your ~/.bashrc file to make sure you are not sourcing both ROS installations.<br />2. Close your terminal and open a new terminal, sourcing an altered ~/.bashrc sometimes results in a termporary source merge with the old ~/.bashrc in that specific terminal instance. |

