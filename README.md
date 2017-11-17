ATI Force/Torque sensor
===================

[![Build Status](https://travis-ci.org/kuka-isir/ati_sensor.svg?branch=master)](https://travis-ci.org/kuka-isir/ati_sensor)

This package contains ROS and OROCOS components for getting Force and Torque mesurement out of the ATI NET F/T sensor.

#### Usage
```bash
roslaunch ati_sensor ft_sensor.launch ip:=192.168.100.103 frame:=/ati_ft_link
```
