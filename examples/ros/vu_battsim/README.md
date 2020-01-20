# SETUP
To setup the correct environment variables a special setup.sh bash script is present in the /setup folder. When you run this setup script all necessary VU_BATTSIM environment variables are set so that the example can run.

FOR MANUAL INSTALL FOLLOW: http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/
-- When choosing manual install, still perform STEP 4 of Automatic install.

## FOR AUTOMATIC INSTALL:

cd into /setup and:

STEP 1: chmod 755 on install_ros_kinetic.sh, install_ros_pkgs.sh and install_vu_battsim.sh

STEP 2: run install_ros_kinetic.sh to install the correct ROS version

STEP 3: run install_ros_pkgs.sh to install all necessary pkgs

STEP 4: run install_vu_battsim.sh to install everything necessary to launch the test environment

## LAUNCH TEST ENVIRONMENT:

from the vu_battsim directory in a terminal:
``roslaunch launch/vu_battsim.launch"
