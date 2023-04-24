## vrpn_client_node
ros node to get optitrack data, and then publish to mavlink topic which received by pixhawk.

System Architecture:
![image](config/sys_arch2.png)
![image](config/mocap-ros.png)
-QGC or offboard control node running on the linux running machine.
-Companion computer is XU4. we can use raspberry pi instead of XU4.
The `mavros` and `vrpn_client_node` running on Companion computer, so as the master node of ros.


## run vrpn_client_node
connect to the same wifi with motive computer(IP: 192.168.3.252), and then run

>192.168.3.252 is the computer ip which running motive 
```bash
roslaunch vrpn_client_ros sample.launch server:=192.168.3.252 
```

## result
we can see rviz, and the frame is ENU.

print the topic of ros:

```bash
rostopic echo /mavros/vision_pose/pose
```
the terminal output isL

```Console
---
header: 
  seq: 9390
  stamp: 
    secs: 1682315199
    nsecs: 620635421
  frame_id: "world"
pose: 
  position: 
    x: 1.7827200889587402
    y: -1.8732807636260986
    z: 0.8786203861236572
  orientation: 
    x: -0.0005116735119372606
    y: -0.0013188595185056329
    z: -0.05677390471100807
    w: 0.998386025428772
---

```

finally, install [mavros](https://docs.px4.io/main/en/ros/mavros_installation.html) and run it by follow command, then the data transfer to pixhawk. the mavros can 

## run mavros
ROS uses ENU frames by convention. Assume the Optitrack system have set `Up Axis` to `Z Up`, and the data obtained by using the vrpn_client_node node is ENU frame. Through topic remapping, mavros/vision_pose/pose is obtained. MAVROS is responsible for converting the ENU frame of mavros/vision_pose/pose into the NED frame used by px4.

>fcu_url is the usb dev, gcs_url is the QGC(ground control station, gcs) computer IP.

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600 gcs_url:=udp://@192.168.3.190
```


