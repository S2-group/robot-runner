echo "[VU_INSTALL] >> Installing dependency: libignition-math2-dev"
sudo apt install libignition-math2-dev

echo "[VU_INSTALL] >> Setting environment variables"
echo export VU_BATTSIM=$(dirname $(dirname $(readlink -f "$0"))) >> ~/.bashrc		 # SET VU_BATTSIM to the vu_battsim folder in robot-runner
source ~/.bashrc
echo export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$VU_BATTSIM/model >> ~/.bashrc	
source ~/.bashrc

echo "[VU_INSTALL] >> Copying vu_gazebo_battery plugin to catkin_ws"
[ -d ~/catkin_ws/src/vu_gazebo_battery ] && sudo rm -r ~/catkin_ws/src/vu_gazebo_battery # If the directory exists, remove it, copy fresh copy in.
cp -r $PWD/vu_gazebo_battery ~/catkin_ws/src/vu_gazebo_battery
cd ~/catkin_ws/src/vu_gazebo_battery

echo "[VU_INSTALL] >> Building plugin"
mkdir build
cd build
cmake ../
make
sudo make install

echo "[VU_INSTALL] >> Catkin_make..."
cd ~/catkin_ws && catkin_make

echo export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/src/vu_gazebo_battery/build/devel/lib >> ~/.bashrc
echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc
source ~/.bashrc

