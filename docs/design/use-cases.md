# V2X Use Cases

## Blindspot VRU Detection using Collective Perception

``` mermaid
sequenceDiagram
  autonumber
  participant VRU
  box RSU
  participant RSU Sensing Unit
  participant RSU V2X Unit
  end
  box CAV
  participant CAV V2X Unit
  participant CAV Perception Stack
  participant CAV Planning Stack
  end
  VRU->>RSU Sensing Unit: Detected by RSU
  loop
    RSU Sensing Unit->>RSU Sensing Unit: Object detection and recognition
  end
  RSU Sensing Unit->>RSU V2X Unit: Selection of objects to send
  RSU V2X Unit-->>VRU: Notification of detection
  loop
    RSU V2X Unit->>RSU V2X Unit: CPM Packet Generation
  end
  RSU V2X Unit->>CAV V2X Unit: CPM Broadcast and Transmission
  CAV V2X Unit->>CAV V2X Unit: CPM Dissection
  CAV V2X Unit->>CAV Perception Stack: PerceivedObjects sent via CPM
  loop
    CAV Perception Stack->>CAV Perception Stack: Fusion with locally detected objects
  end
  CAV Perception Stack->>CAV Planning Stack: Object information that will <br>influence planning decisions
```

### Other Considerations

- Where and how to conduct sensor (information) fusion when dealing with locally percevied objects and CPM-shared objects. The same can be said for traffic light V2I scenarios as well.
- Depending on the V2X standard and the types of information sent via V2X, the applied sensor fusion methods or available features can differ. For example, there's a big difference between sending raw pointcloud information (early fusion) and just bounding-box object information (late-fusion).

## CAMs betweens CAVs


``` mermaid
sequenceDiagram
  autonumber
  box CAV1
    participant CAV1 Localization
    participant CAV1 V2X Unit
  end
  box CAV2
    participant CAV2 V2X Unit
    participant CAV2 LDM
  end
  box CAV3
    participant CAV3 V2X Unit
  end
  CAV1 Localization->>CAV1 V2X Unit: Current pose, kinematics, acceleration
  loop
    CAV1 V2X Unit->>CAV1 V2X Unit: CAM Packet Generation
  end
  par CAV1 to CAV2
    CAV1 V2X Unit->>CAV2 V2X Unit: CAM Broadcast and Transmission
    CAV2 V2X Unit->>CAV2 LDM: Send CAM Info
    CAV2 LDM->>CAV2 LDM: Store CAM Info
    loop
      CAV2 LDM->>CAV2 LDM: Use CAM Info
    end
  and CAV1 to CAV3
    CAV1 V2X Unit->>CAV3 V2X Unit: CAM Broadcast and Transmission
  end
  
```

## I2V Traffic Light Information (TLI) Sharing

``` mermaid
sequenceDiagram
  autonumber
  box Traffic Light
    participant TL Main Unit
    participant TL V2X Unit
  end
  box CAV
    participant CAV V2X Unit
    participant TLDC as CAV TL Detector and Classifier
    participant CAV TL Fusion Module
    participant CAV Planning Stack
  end
  TL Main Unit->>TL V2X Unit: Send TLI
  Note right of TL Main Unit: Current phase, <br>Min/MaxTimeToChange, <br>Phase Schedule
  loop
    TL V2X Unit->>TL V2X Unit: SPaT Packet Generation
  end
  TL V2X Unit->>CAV V2X Unit: Broadcast/Unicast SPaT Packet
  CAV V2X Unit->>CAV V2X Unit: Dissect SPaT Packet
  CAV V2X Unit->>CAV TL Fusion Module: Send TLI
  TLDC->>CAV TL Fusion Module: Send locally perceived TLI
  loop
    CAV TL Fusion Module->>CAV TL Fusion Module: TLI Fusion of SPaT and local perception
  end
  CAV TL Fusion Module->>CAV Planning Stack: Send TLI
  Note right of CAV Planning Stack: TL-conscious Planning
  
```