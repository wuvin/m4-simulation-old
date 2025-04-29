# M4 model URDF model and gazebo_uneven_terrain package

just put repo in any ros workspace and do catkin build or catkin_make 

different roslaunch files are setup for turtlebot,husky and M4 (first 2 models need to be installed before operation)

2D differential drive with realsense camera from gazebo.launch of m4assembly package 

3D control in progress  
## Installing Ardupilot and MAVProxy For 3D control of M4 with UBUNTU 20.04 and ROS noetic 

## Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

## Install dependencies:
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

reload profile
```
. ~/.profile
```

## Checkout Latest Copter Build
```
git checkout Copter-4.0.4
git submodule update --init --recursive
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

***You can also do this to build the repo if you are facing problems***
```
./waf configure --board sitl
./waf copter
```

After the build is successful you can check if the SITL is working using the following commands
```
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --map --console
```

## Install Gazebo plugin for APM (ArduPilot Master) :
```
cd ~
git clone --recurse https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

## Run Simulator

***NOTE the iris_arducopter_runway is not currently working in gazebo11.***

In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2)(use Ctrl+Shift+T to open another terminal), run SITL:
```
cd ~/ardupilot/ArduCopter/
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map

or 
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console

```

# Installing QGroundControl on Ubuntu Linux 20.04 LTS

On a new terminal enter
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```
Logout and login again to enable the change to user permissions.
***You can use this command if you are a command line nerd (remember to save any current progress as this instantly logs you out).***
```
loginctl terminate-user $USER
```

Download the latest QGroundControl.AppImage (you can check https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html for the latest version)
```
cd ~ # Or download the Image in any directory of your choice
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```
Change permissions and run 
```
chmod +x ./QGroundControl.AppImage 
./QGroundControl.AppImage  (or double click)
```

## Run SITL and connect with Q Ground

In another terminal run SITL and QGround will automatically connect.
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py
```

## MAVROS and MAVLINK

Install `mavros` and `mavlink` from source:
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```

Install geographiclib dependancy 
```
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

