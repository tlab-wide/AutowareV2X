# Tutorials

Simulations can be an easy way of verifying the functionality of AutowareV2X before an actual field test.
Here, since we want to test both the sending and receiving of information through AutowareV2X, we will need at least two AutowareV2X instances. For this, we will use a Docker-based environment.

## Using CPM in a simulation-based environment

1. Create Docker networks
```
$ docker network create --driver=bridge --subnet=10.0.0.0/16 v2x_net -o com.docker.network.bridge.name="v2x_net"
```
# Launch Autoware container
rocker --nvidia --x11 --user --volume $HOME/autoware_docker --volume $HOME/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda

$ cd autoware_docker
$ . install/setup.bash
$ ros2 launch autoware_v2x v2x.launch.xml network_interface:=eth1
```