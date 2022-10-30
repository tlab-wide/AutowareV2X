# Network Emulation

The `tc` command can be used within the Docker container to emulate various network parameters.

In `autoware_1` for example, use `tc` to add a delay of 100ms.
```
sudo tc qdisc add dev eth0 root netem delay 100ms
```

To remove delay, simply:
```
sudo tc qdisc delete dev eth0 root netem delay 100ms
```

To show all qdisc:
```
sudo tc qdisc show
```

Documentation about [tc-netem](https://man7.org/linux/man-pages/man8/tc-netem.8.html).