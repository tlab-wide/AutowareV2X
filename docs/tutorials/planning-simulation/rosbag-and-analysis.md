# Recording Rosbag and Tcpdump for Analysis

## Record both rosbag and tcpdump

In `autoware_1`:
```
docker exec -it autoware_1 bash
cd ~/workspace/autoware_docker
ros2 bag record -o test_sender_rosbag /perception/object_recognition/objects /tf

sudo apt update
sudo apt install tcpdump
sudo tcpdump -i eth0 -w test_sender_tcpdump.pcap
```

In `autoware_2`:
```
docker exec -it autoware_2 bash
cd ~/workspace/autoware_docker
ros2 bag record -o test_receiver_rosbag /v2x/cpm/objects /tf

sudo apt update
sudo apt install tcpdump
sudo tcpdump -i eth0 -w test_receiver_tcpdump.pcap
```

## Open PCAP file in Wireshark

![](./wireshark_awv2x.png)

## Analyze in JupyteLab

### Plot the x,y coordinates of objects in test_sender_rosbag

1. Export the necessary topics of the Rosbag to a CSV file