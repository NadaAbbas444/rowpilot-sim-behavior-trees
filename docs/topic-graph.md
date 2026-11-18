# Processes, topics and messages

Generated from the running simulation by `scripts/topic_graph.py` (the simulation's rqt_graph).
Boxes are processes; rounded boxes are topics or links, with their message type.

```mermaid
flowchart LR
    n0["Gazebo (physics, sensors)"]
    n1["imu_synth"]
    n2("/gh_firefly/d435i_imu
gz.msgs.IMU")
    n0 --> n2
    n2 --> n1
    n3["rowpilot (behaviour tree)"]
    n4("/gh_firefly/depth
gz.msgs.Image")
    n0 --> n4
    n4 --> n3
    n5["vio_bridge (OpenVINS)"]
    n6("/gh_firefly/infra1
gz.msgs.Image")
    n0 --> n6
    n6 --> n5
    n7("/gh_firefly/infra2
gz.msgs.Image")
    n0 --> n7
    n7 --> n5
    n8["(plant images: saved on the real drone)"]
    n9("/gh_firefly/picam
gz.msgs.Image")
    n0 --> n9
    n9 --> n8
    n10("/model/gh_firefly/pose
gz.msgs.Pose_V")
    n0 --> n10
    n10 --> n1
    n11("/world/greenhouse/clock
gz.msgs.Clock")
    n0 --> n11
    n11 --> n3
    n12("/world/greenhouse/pose/info
gz.msgs.Pose_V")
    n0 --> n12
    n12 --> n3
    n13("/gh_firefly/d435i_imu_kin
gz.msgs.IMU")
    n1 --> n13
    n13 --> n5
    n14("/rowpilot/vio/odometry
gz.msgs.Odometry")
    n5 --> n14
    n14 --> n3
    n15["ArduCopter SITL"]
    n16("MAVLink tcp:5760
ODOMETRY, SET_POSITION_TARGET_LOCAL_NED, COMMAND_LONG")
    n3 --> n16
    n16 --> n15
    n17("MAVLink tcp:5760
HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS")
    n15 --> n17
    n17 --> n3
    n18("JSON UDP 9002
motor PWM")
    n15 --> n18
    n18 --> n0
    n19("JSON UDP 9002
IMU, pose, velocity (lock-step)")
    n0 --> n19
    n19 --> n15
```

| Publisher | Topic / link | Message | Subscriber |
|---|---|---|---|
| Gazebo (physics, sensors) | `/gh_firefly/d435i_imu` | gz.msgs.IMU | imu_synth |
| Gazebo (physics, sensors) | `/gh_firefly/depth` | gz.msgs.Image | rowpilot (behaviour tree) |
| Gazebo (physics, sensors) | `/gh_firefly/infra1` | gz.msgs.Image | vio_bridge (OpenVINS) |
| Gazebo (physics, sensors) | `/gh_firefly/infra2` | gz.msgs.Image | vio_bridge (OpenVINS) |
| Gazebo (physics, sensors) | `/gh_firefly/picam` | gz.msgs.Image | (plant images: saved on the real drone) |
| Gazebo (physics, sensors) | `/model/gh_firefly/pose` | gz.msgs.Pose_V | imu_synth |
| Gazebo (physics, sensors) | `/world/greenhouse/clock` | gz.msgs.Clock | rowpilot (behaviour tree) |
| Gazebo (physics, sensors) | `/world/greenhouse/pose/info` | gz.msgs.Pose_V | rowpilot (behaviour tree) |
| imu_synth | `/gh_firefly/d435i_imu_kin` | gz.msgs.IMU | vio_bridge (OpenVINS) |
| vio_bridge (OpenVINS) | `/rowpilot/vio/odometry` | gz.msgs.Odometry | rowpilot (behaviour tree) |
| rowpilot (behaviour tree) | `MAVLink tcp:5760` | ODOMETRY, SET_POSITION_TARGET_LOCAL_NED, COMMAND_LONG | ArduCopter SITL |
| ArduCopter SITL | `MAVLink tcp:5760` | HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS | rowpilot (behaviour tree) |
| ArduCopter SITL | `JSON UDP 9002` | motor PWM | Gazebo (physics, sensors) |
| Gazebo (physics, sensors) | `JSON UDP 9002` | IMU, pose, velocity (lock-step) | ArduCopter SITL |
