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

Set the right WiFi parameters.

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

Apply your changes.
```bash
$ sudo netplan apply
```

---
# Howto

## Setup the ROS environment:

Already in the .bashrc

```bash
$ bash /opt/ros/eloquent/setup.bash
```

## Check DJ Tello

1) Turn the drone ON and wait for the yellow blinking light;
2) Connect to its WiFi network (see the WiFi configuration section);
3) Run the takeoff and landing script:

```bash
$ python3 ~/check-tello.py
```

## Check Turtlebot

...

---
# Todo

 - Add ROS1 support;
 - Write some ROS tests (rostest) for validation;
 - Test with TurtleBot3.

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [Robot-Runner]: <https://github.com/S2-group/robot-runner>

