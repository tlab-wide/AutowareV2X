# Installing AutowareV2X

AutowareV2X is used as an add-on module to the open-source autonomous driving stack called [Autoware](https://autowarefoundation.github.io/autoware-documentation/main/). Therefore, in order to properly use AutowareV2X, Autoware must first be installed on the system.

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

1. Edit the `autoware.repos` file and add the following two repositories to the end.
```
v2x/autoware_v2x:
  type: git
  url: git@github.com:tlab-wide/autoware_v2x.git
  version: main
v2x/vanetza:
  type: git
  url: git@github.com:tlab-wide/vanetza.git
  version: socktap-cpm-tr103562

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
