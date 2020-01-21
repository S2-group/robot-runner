echo "[VU_INSTALL] >> Installing dependency: libignition-math2-dev"
sudo apt install libignition-math2-dev

echo "[VU_INSTALL] >> Setting environment variables"
export VU_BATTSIM=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(dirname $(dirname $(readlink -f "$0")))/model
source ~/.bashrc

echo "[VU_INSTALL] >> Copying vu_gazebo_battery plugin to catkin_ws"
[ -d $VU_BATTSIM/vu_gazebo_battery ] && sudo rm -r ~catkin_ws/src/vu_gazebo_battery
cp -r $VU_BATTSIM/vu_gazebo_battery ~/catkin_ws/src/vu_gazebo_battery
cd ~/catkin_ws/src/vu_gazebo_battery
echo "[VU_INSTALL] >> Building plugin"
mkdir build
cd build
cmake ../
make
sudo make install
echo "[VU_INSTALL] >> Catkin_make..."
cd ~/catkin_ws && catkin_make
