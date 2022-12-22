# Source Installation

## Prerequisites

- OS
    - Ubuntu 20.04
    - Ubuntu 22.04
- ROS
    - ROS2 Galactic

## Installing Autoware

Refer to the [Official Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) for the newest installation procedures. In a nutshell, you can run the following commands:

```bash
# Clone repository
git clone https://github.com/autowarefoundation/autoware.git
cd autoware

# Install dependencies using Ansible
./setup-dev-env.sh

# Use vcstool to import more repositories
mkdir src
vcs import src < autoware.repos

# Install dependent ROS packages
source /opt/ros/galactic/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Adding AutowareV2X

1. Edit the `autoware.repos` file and replace the following repositories.
```
repositories:
  core/autoware.core:
    type: git
    url: https://github.com/autowarefoundation/autoware.core.git
    version: 6bafedfb24fb34157ed65bfe3f6f4c1ed0fbc80b
  core/autoware_adapi_msgs:
    type: git
    url: https://github.com/autowarefoundation/autoware_adapi_msgs.git
    version: 9679b5a7a1f4cfff2fa50b80d2759d3937f2f953
  core/autoware_common:
    type: git
    url: https://github.com/autowarefoundation/autoware_common.git
    version: 6916df26fafe6749db4b1d5bd6636a92444fc48d
  core/autoware_msgs:
    type: git
    url: https://github.com/autowarefoundation/autoware_msgs.git
    version: 4f13d4b8b465ed7f424fce9af17882dbe1752875
  core/external/autoware_auto_msgs:
    type: git
    url: https://github.com/tier4/autoware_auto_msgs.git
    version: 6b5bc4365f9a2fc913bc11afa74ec21ffa2dbf32
  launcher/autoware_launch:
    type: git
    url: https://github.com/autowarefoundation/autoware_launch.git
    version: e4abe673667a8d4f2d783ed22edacbf5d4784b8f
  param/autoware_individual_params:
    type: git
    url: https://github.com/autowarefoundation/autoware_individual_params.git
    version: 79cff0ba014808050be6f5cb3b4764ba2c96c21c
  sensor_component/external/sensor_component_description:
    type: git
    url: https://github.com/tier4/sensor_component_description.git
    version: 475857daeb4c4883ab0295336713364b326e8278
  sensor_component/external/tamagawa_imu_driver:
    type: git
    url: https://github.com/tier4/tamagawa_imu_driver.git
    version: 28ad3cd4fb043e5f92353a540c3531cd4cb7bef3
  sensor_component/external/velodyne_vls:
    type: git
    url: https://github.com/tier4/velodyne_vls.git
    version: baeafaf9a376c5798f7b67a77211890c33900f84
  sensor_kit/external/awsim_sensor_kit_launch:
    type: git
    url: https://github.com/RobotecAI/awsim_sensor_kit_launch.git
    version: d9022ee9bbfd958c239b673cfbb230eea50607be
  sensor_kit/sample_sensor_kit_launch:
    type: git
    url: https://github.com/autowarefoundation/sample_sensor_kit_launch.git
    version: 03decbd31bb954eb9f52daaf3a3fa2b921dbb0c3
  universe/autoware.universe:
    type: git
    url: https://github.com/autowarefoundation/autoware.universe.git
    version: febbc135b8e09e993ed345ee6d3cd7e65b6c1d68
  universe/external/morai_msgs:
    type: git
    url: https://github.com/MORAI-Autonomous/MORAI-ROS2_morai_msgs.git
    version: 6fd6a711e4bbf8a9989b54028e8074acabbbce6f
  universe/external/muSSP:
    type: git
    url: https://github.com/tier4/muSSP.git
    version: c79e98fd5e658f4f90c06d93472faa977bc873b9
  universe/external/ndt_omp:
    type: git
    url: https://github.com/tier4/ndt_omp.git
    version: f59e1667390fe66d72c5c3aa0b25385b5b6dd8cf
  universe/external/pointcloud_to_laserscan:
    type: git
    url: https://github.com/tier4/pointcloud_to_laserscan.git
    version: 948a4fca35dcb03c6c8fbfa610a686f7c919fe0b
  universe/external/tier4_ad_api_adaptor:
    type: git
    url: https://github.com/tier4/tier4_ad_api_adaptor.git
    version: 5084f9c8eaf03458a216060798da2b1e4fa96f28
  universe/external/tier4_autoware_msgs:
    type: git
    url: https://github.com/tier4/tier4_autoware_msgs.git
    version: a360ee9f5235a0d426427813f26e43027e32139d
  vehicle/external/pacmod_interface:
    type: git
    url: https://github.com/tier4/pacmod_interface.git
    version: b5ae20345f2551da0c6e4140a3dc3479d64efd1f
  vehicle/sample_vehicle_launch:
    type: git
    url: https://github.com/autowarefoundation/sample_vehicle_launch.git
    version: 157238ca77de7b0a59f71a0b28f456741fab3ca2
  v2x/autowarev2x:
    type: git
    url: https://github.com/tlab-wide/AutowareV2X.git
    version: 48a1f2d3db6ae59e92febb93aad7cde760f4f3ec
  v2x/vanetza:
    type: git
    url: https://github.com/yuasabe/vanetza.git
    version: cfffe9afda177297c59bbb804d3e8f66120c8453
```

!!! Note
    If you want to　follow the latest ver, edit the `autoware.repos` file and add the following two repositories to the end.

```
v2x/autowarev2x:
  type: git
  url: https://github.com/tlab-wide/AutowareV2X.git
  version: cpm-tr
v2x/vanetza:
  type: git
  url: https://github.com/yuasabe/vanetza.git
  version: master

```

2. Update the repository
```
vcs import src < autoware.repos
vcs pull src
```

3. Install dependent ROS packages
```bash
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

4. Build the workspace
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
