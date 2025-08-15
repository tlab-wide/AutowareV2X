# AutowareV2X
[![Collective_Perception_between_RSU_and_Vehicle_using_Autoware (2)](https://github-production-user-asset-6210df.s3.amazonaws.com/3140505/253303730-8662b65b-99af-4793-8c4c-62d457fc9e62.gif)](https://youtu.be/57fx3-gUNxU)

AutowareV2X is an open-source module that can be added onto the newest Autoware.universe to enable V2X communication.

## Prerequisites

- **Vanetza**: This project requires Vanetza (ETSI C-ITS protocol stack) to be installed on your system. Vanetza provides the V2X communication stack used for sending and receiving CPM messages.
- **Autoware 0.45.1+**: Compatible with modern Autoware versions (tested with 0.45.1)

## Compatibility Note

This version has been updated to work with modern Autoware (0.45.1+) which uses `autoware_perception_msgs` instead of the older `autoware_auto_perception_msgs`. If you find this code useful in your research, please consider citing :

    @inproceedings{Asabe2023b,
        title = {AutowareV2X: Reliable V2X Communication and Collective Perception for Autonomous Driving},
        author = {Yu Asabe and Ehsan Javanmardi and Jin Nakazato and Manabu Tsukada and Hiroshi Esaki},
        year = {2023},
        date = {2023-06-20},
        booktitle = {The 2023 IEEE 97th Vehicular Technology Conference (VTC2023-Spring)},
        address = {Florence, Italy},
    }

It utilizes Vanetza as the protocol suite for ETSI C-ITS standards.

We have also provided a working example of a CPM application where Collective Perception Messages can be used to exchange perception information in Autoware.





![AutowareV2X Architecture](./docs/figs/autowarev2x_architecture_v2.png)

See the [official documentation](https://tlab-wide.github.io/AutowareV2X/)
