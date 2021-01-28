# Raspberry+ROS Image

This is a customized Ubuntu Bionic (18.04) image for running ROS on Raspberry Pi devices. By default, it supports Turtlebot3 and DJ Tello. You can enable other robots according to your needs.

Automate them with [Robot-Runner].

> It has been run on Raspberry Pi 3 Model B+ and Raspberry Pi 4 Model B.
---
# Package
  - Ubuntu 18.04
  - ROS2 (Eloquent)
  - Turtlebot3 ROS Modules
  - easyTello Python Library + Runner Script
---

# Log In

* Username: ubuntu
* Password: s2group

## Remote (SSH)
```bash
$ ssh s2group@192.168.200.200
```

## Network/WiFi Configuration

Edit the configuration file.
```bash
$ sudo nano /etc/netplan/50-cloud-init.yaml
```

Set up WiFi parameters:

```file
...
network:
    ethernets:
        eth0:
            dhcp4: false
            addresses: [192.168.200.200/24]
    version: 2
    wifis:
        wlan0:
            optional: true
            access-points:
                "YOUR-SSID-NAME":
                    password: "YOUR-NETWORK-PASSWORD"
            dhcp4: true
```

... for Open WiFi (No Password)

```
    wifis:
        wlan0:
            optional: true
            access-points:
                "YOUR-SSID-NAME": {}
            dhcp4: true
```

Apply your changes.
```bash
$ sudo netplan apply
```

---
# Howto

## Setup the ROS1 environment:

Setup environment variables:

```bash
$ bash /opt/ros/melodic/setup.bash
```

Check the variables:

```bash
$ printenv | grep -i ROS
```

You should see this:

```
ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ROS_ROOT=/opt/ros/melodic/share/ros
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=1
ROS_PYTHON_VERSION=2
ROS_PACKAGE_PATH=/opt/ros/melodic/share
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=melodic
```

Run master node (if you run on standalone mode)...

```bash
$ roscore &
```

otherwise, you set up ROS master node?

```bash
export ROS_MASTER_URI=http://<master_host>:<master_port>
```

Run a simple example:

```bash
$ rosrun rospy_tutorials talker chatter:=/wg/chatter
```

You should get this:

```
[INFO] [1611755100.147060]: hello world 1611755100.15
[INFO] [1611755100.258993]: hello world 1611755100.26
[INFO] [1611755100.347802]: hello world 1611755100.35
[INFO] [1611755100.447843]: hello world 1611755100.45
[INFO] [1611755100.547811]: hello world 1611755100.55
[INFO] [1611755100.647949]: hello world 1611755100.65
[INFO] [1611755100.747753]: hello world 1611755100.75
...
```
## Setup the ROS2 environment:

If ROS1 is running, kill the master node.

```bash
$ pkill roscore
```

Setup environment variables:

```bash
$ bash /opt/ros/eloquent/setup.bash
```

Check the variables:

```bash
$ printenv | grep -i ROS
```

You should see this:

```
ROS_VERSION=2
ROS_LOCALHOST_ONLY=0
ROS_PYTHON_VERSION=3
ROS_DISTRO=eloquent
```

Run a simple example:

```bash
$ ros2 topic pub /chatter std_msgs/String "{data: 'Hello there'}"
```

You should get this:

```
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hello there')

publishing #2: std_msgs.msg.String(data='Hello there')

publishing #3: std_msgs.msg.String(data='Hello there')
```

## Check DJ Tello (TELLO > WIFI -- ETHERNET < ROS)

Raspberry connects to TELLO WiFi, while ROS runs on ethernet network. So far, you must write your own Python scripts by using [TelloPy library] to interact to Tello, and manage to link [rospy] (for ROS communicaton) and TelloPy. Not a big deal at all!

Anyway, we'll be dropping here a simple example.

1) Turn the drone ON and wait for the yellow blinking light;
2) Connect to its WiFi network (see the WiFi configuration section);
3) Run the takeoff and landing script:

```bash
$ ~/check-tello.sh
```
The drone will take off, hover for a few seconds, and then land.

Raspberry terminal should report something like the following:

```
Tello: 16:51:56.579:  Info: start video thread
Tello: 16:51:56.579:  Info: send connection request (cmd="conn_req:9617")
Tello: 16:51:56.579:  Info: video receive buffer size = 425984
Tello: 16:51:56.582:  Info: state transit State::disconnected -> State::connecting
ALT:  0 | SPD:  0 | BAT: 39 | WIFI:  0 | CAM:  0 | MODE:  1
Tello: 16:51:56.636:  Info: connected. (port=9617)
Tello: 16:51:56.636:  Info: send_time (cmd=0x46 seq=0x01e4)
Tello: 16:51:56.636:  Info: state transit State::connecting -> State::connected
Tello: 16:51:56.637:  Info: set altitude limit 30m
Tello: 16:51:56.637:  Info: takeoff (cmd=0x54 seq=0x01e4)
ALT:  0 | SPD:  0 | BAT: 39 | WIFI:  0 | CAM:  0 | MODE:  1
ALT:  0 | SPD:  0 | BAT: 39 | WIFI:  0 | CAM:  0 | MODE: 41
ALT:  0 | SPD:  0 | BAT: 39 | WIFI:  0 | CAM:  0 | MODE: 41
ALT:  0 | SPD:  0 | BAT: 39 | WIFI:  0 | CAM:  0 | MODE: 41
ALT:  0 | SPD:  0 | BAT: 39 | WIFI: 90 | CAM:  0 | MODE: 41
ALT:  0 | SPD:  0 | BAT: 39 | WIFI: 90 | CAM:  0 | MODE: 41
ALT:  0 | SPD:  0 | BAT: 39 | WIFI: 90 | CAM:  0 | MODE: 41
...
```

### DJ Tello Example

Not available yet. TODO by Michel.

## Check Turtlebot

TODO by Ivano/VU Team
...

---
# Todo
 - DJ Tello and ROS;
 - TurtleBot and ROS;
 - Customize splash (fancy stuff).

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [Robot-Runner]: <https://github.com/S2-group/robot-runner>
   [TelloPy library]: <https://github.com/hanyazou/TelloPy>
   [rospy]: <http://wiki.ros.org/rospy>
