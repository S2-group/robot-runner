export VU_BATTSIM=$(dirname $(dirname $(readlink -f "$0")))
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(dirname $(dirname $(readlink -f "$0")))/model

sudo rm -r ~/catkin_ws/src/vu_gazebo_battery
cp -r $VU_BATTSIM/vu_gazebo_battery ~/catkin_ws/src/vu_gazebo_battery
cd ~/catkin_ws/src/vu_gazebo_battery
mkdir build
cd build
cmake ../
make
sudo make install
cd ~/catkin_ws && catkin_make
