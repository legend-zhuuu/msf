#!/bin/bash

sudo setcap 'CAP_SYS_NICE=ep' devel/lib/lqc-ros/lqc_node
sudo ldconfig /opt/ros/noetic/lib
