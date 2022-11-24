# Design

!!! Warning
    More to come

A V2X communication software stack called [Vanetza](https://github.com/riebl/vanetza)  is integrated into the standalone autonomous driving software stack, [Autoware](https://github.com/autowarefoundation/autoware). The V2X stack and the autonomous driving stack can be decoupled, allowing other applications to utilize the V2X router as well. A high-level overview of the architecture is shown below. 

![AutowareV2X Architecture diagram](../figs/autowarev2x_architecture_v2.png)

Autoware is responsible for the perception task, while AutowareV2X manages the transmission and reception of messages over the V2X channel. Services that are necessary for the integration of Vanetza into Autoware were newly developed.

The V2XApp is responsible for managing the various facilities such as DENM, CAM, CPM, while the V2XNode handles the conversion of information between the V2X messages and ROS2 messages.

## V2XNode

The V2XNode launches a ROS2 Node for AutowareV2X. Its main purpose is to act as the bridge interface between Autoware and AutowareV2X. Information that is to be utilized in V2X Applications are retreived from Autoware in the form of ROS2 topics. Similarily, information that is received by AutowareV2X through V2X communications is published as ROS2 topics in order to feed it back into Autoware. 

### Input

| Name                 | Type                            | Description      |
| -------------------- | ------------------------------- | ---------------- |
| `/perception/object_recognition/objects` | `autoware_auto_perception_msgs::msg::PredictedObjects` | Perceived Objects by Autoware |
| `/tf` | `tf2_msgs::msg::TFMessage` | Pose of Ego Vehicle |

### Output

| Name                 | Type                            | Description      |
| -------------------- | ------------------------------- | ---------------- |
| `/v2x/cpm/objects` | `autoware_auto_perception_msgs::msg::PredictedObjects` | Objects received by CPMs |

### Functions

| Name                 | Description      |
| -------------------- | ---------------- |
| `objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)` | Call `V2XApp::objectsCallback` |
| `tfCallback` | Call `V2XApp::tfCallback` |
| `publishObjects(std::vector<CpmApplication::Object> *objectsStack, int cpm_num)` |  |
| `publishCpmSenderObject` | Not used now |


## V2XApp

## CPM Facility