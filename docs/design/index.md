# Design

!!! Warning
    More to come

A V2X communication software stack called [Vanetza](https://github.com/riebl/vanetza)  is integrated into the standalone autonomous driving software stack, [Autoware](https://github.com/autowarefoundation/autoware). The V2X stack and the autonomous driving stack can be decoupled, allowing other applications to utilize the V2X router as well. A high-level overview of the architecture is shown below. 

![AutowareV2X Architecture](../architecture.png)

Autoware is responsible for the perception task, while Autoware\_V2X manages the transmission and reception of messages over the V2X channel. Services that are necessary for the integration of Vanetza into Autoware were newly developed. 

The V2XApp is responsible for managing the various facilities such as DENM, CAM, CPM, while the V2XNode handles the conversion of information between the V2X messages and ROS2 messages.