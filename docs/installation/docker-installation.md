# Docker Installation

In order to run the simulations explained in the [Tutorials](/tutorials) section, you will need to proceed with the Docker installation.

!!! Note
    Also refer to [Autoware's Docker Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/) for the Docker-based installation of Autoware.universe.

## Installing Autoware (Docker version)

For the newest documentation for the Docker installation of Autoware, see their [official documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/).

In a nutshell, the following commands should work:

```bash
git clone https://github.com/autowarefoundation/autoware.git autoware_docker
cd autoware_docker

# Install dependencies using Ansible
./setup-dev-env.sh docker

# Make directory to store maps
mkdir ~/autoware_map

# Launch Autoware container (with NVIDIA GPU)
rocker --nvidia --x11 --user --volume $HOME/autoware_docker --volume $HOME/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda

# Launch Autoware container (without NVIDIA GPU)
rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware_docker --volume $HOME/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

## Adding AutowareV2X

From here, run commands inside the container.

1. Move into `autoware_docker` directory.
```bash
cd autoware_docker
```

2. Edit the `autoware.repos` file and add the following two repositories to the end.
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

3. Update the repository
```
mkdir src
vcs import src < autoware.repos
vcs pull src
```

4. Install dependent ROS packages
```bash
sudo apt update
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

5. Build the workspace
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
