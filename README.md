# Robot-Runner
This framework enables automated experiment execution on ROS devices both in a real world setting as in a simulated setting.

# Install
Robot-Runner is developed and tested on two Operating Systems: <strong>Ubuntu 16.04 and Ubuntu 18.04</strong>.
The corresponding 'latest stable' ROS releases for both Operating Systems: <strong>Kinetic and Melodic</strong>
respectfully, were used and tested. The setup scripts present in this repo for each operating system take care of 
all the necessary tools, packages and correct versions for that Operating System.

After cloning, Robot-Runner is thus not immediately able to run successfully. It relies on installed tools and packages 
to be available, which can be easily installed using the correct setup scripts for your Operating System (``Ubuntu 16.04 or Ubuntu 18.04``):

``vu_battsim_16.04 || vu_battsim_18.04``
The setup scripts are present in the <strong>install</strong> folder in each of the corresponding example packages.

## Setup for Ubuntu 16.04
FOR MANUAL INSTALL FOLLOW: http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/

<strong>When choosing manual install, still perform ``STEP 4`` of Automatic install.</strong>

### Automatic install:

cd into /examples/vu_battsim_16.04/install and:

<strong>STEP 1</strong>: ``chmod 755`` on ``install_ros_kinetic.sh``, ``install_ros_pkgs.sh`` and ``install_vu_battsim.sh``

<strong>STEP 2</strong>: run ``install_ros_kinetic.sh`` to install the correct ROS version (<strong>KINETIC</strong>)

<strong>STEP 3</strong>: run ``install_ros_pkgs.sh`` to install all necessary packages

<strong>STEP 4</strong>: run ``install_vu_battsim.sh`` to install everything necessary to launch the test environment

## Setup for Ubuntu 18.04


## LAUNCH TEST ENVIRONMENT:

from the vu_battsim_16.04 directory in a terminal:

``roslaunch launch/vu_battsim.launch``

You should now see:
 - The terminal launch a ROS master
 - The [VU_BATT] Gazebo plugin should only output in green
 - A Gazebo instance should now be running, showing a Turtlebot3 Burger in a house.