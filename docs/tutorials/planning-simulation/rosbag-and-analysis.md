# Recording Rosbag and Tcpdump for Analysis

## Record both rosbag and tcpdump

In `autoware_1`:
```
cd ~/workspace/autoware_docker
ros2 bag record -o test_sender_rosbag /perception/object_recognition/objects /tf
tcpdump -i eth0 -w test_sender_tcpdump
```

In `autoware_2`:
```
cd ~/workspace/autoware_docker
ros2 bag record -o test_receiver_rosbag /v2x/cpm/objects /tf
tcpdump -i eth0 -w test_receiver_tcpdump
```

## Analyze in JupyteLab

### Plot the x,y coordinates of objects in test_sender_rosbag

1. Export the necessary topics of the Rosbag to a CSV file