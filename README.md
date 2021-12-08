# Autoware_v2x

Using Autoware_v2x with Autoware.iv.

## Installation

Basically, you should be able to install Autoware.iv by following the Installation Steps in the [Autoware.proj Repository README](https://github.com/tlab-wide/autoware.proj-v1.0.0).

Otherwise, follow the steps here.


1. Set up Autoware Repository
```
mkdir -p ~/workspace
cd ~/workspace
git clone git@github.com:tlab-wide/autoware.proj-v1.0.0.git
cd autoware.proj-v1.0.0
```

2. Run the set up script
```
./setup_ubuntu20.04.sh
```

3. Build source code
```
source /opt/ros/foxy/setup.bash
colcon build
```

## Running

1. Start Autoware.iv (for example planning_simulator)
```
. install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=[ADD PATH TO MAP HERE] vehicle_model:=lexus sensor_model:=aip_xx1
```

2. Start Autoware_V2X as root
```
sudo su
. install/setup.bash
ros2 launch autoware_v2x v2x.launch.xml
```

### Contact

Yu Asabe (yuasabe[at]hongo.wide.ad.jp)