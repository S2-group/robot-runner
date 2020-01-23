source ~/.bashrc
[ -d ~/catkin_ws ] && echo "~/catkin_ws already exists" || mkdir -p ~/catkin_ws
[ -d ~/catkin_ws/src ] && echo "~/catkin_ws/src already exists" || mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws/ && catkin_make
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
cd ~/catkin_ws/src
sudo cp -r turtlebot3 /opt/ros/melodic/share
sudo cp -r turtlebot3_msgs /opt/ros/melodic/share
sudo cp -r turtlebot3_simulations /opt/ros/melodic/share
