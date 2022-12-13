# Docker Installation

In order to run the simulations explained in the [Tutorials](/tutorials) section, you will need to proceed with the Docker installation.

!!! Note
    Also refer to [Autoware's Docker Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/) for the Docker-based installation of Autoware.universe.

## Installing Autoware (Docker version)

For the newest documentation for the Docker installation of Autoware, see their [official documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/).

### Setup

```bash
mkdir -p ~/workspace && cd ~/workspace
git clone https://github.com/autowarefoundation/autoware.git autoware_docker
cd autoware_docker

# Install dependencies using Ansible
./setup-dev-env.sh docker

# Make directory to store maps
mkdir -p ~/data/maps
```


### Launch container
```
# Launch Autoware container (with NVIDIA GPU)
rocker --nvidia --x11 --user --privileged --volume $HOME/workspace/autoware_docker --volume $HOME/data -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda

# Launch Autoware container (without NVIDIA GPU)
rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --privileged --volume $HOME/workspace/autoware_docker --volume $HOME/data -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

## Adding AutowareV2X

!!! Note
    From here, run commands inside the container.

1. Move into `autoware_docker` directory.
```bash
cd ~/workspace/autoware_docker
```

2. Edit the `autoware.repos` file and add the following two repositories to the end.
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
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -r
```

5. Build the workspace
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
