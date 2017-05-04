# OpenNI2 Tracker

The OpenNI2 Tracker broadcasts the OpenNI skeleton frames using tf. For more information checkout the ROS Wiki: http://ros.org/wiki/openni2_tracker.

OpenNI2 Tracker is the new version of the http://github.com/ros-drivers/openni_tracker using OpenNI2 and Nite2.

# How to install NiTE2

1. put NiTE-Linux-x64-2.0.0.tar.bz2.gz under ~/Downloads

2. download [primesense-nite2-nonfree_2.0.0-3_amd64.deb.gz](https://github.com/ros-drivers/openni2_tracker/files/287720/primesense-nite2-nonfree_2.0.0-3_amd64.deb.gz), `gunzip` it and run `dpkg-i`

3. run `dpkg -i /var/cache/primesense-nite2-nonfree/openni-module-primesense-nite2-nonfree_2.0.0.0-1_amd64.deb`

# How to run demo

1. To run openni2 driver and openni2 tracker at once

`$ roslsaunch openni2_tracker openni2_tracker.launch` will start

2. To run openni2 driver and openni2 tracker separately

`$ roslsaunch openni2_tracker openni2_launch openni2.launch`

`$ ROS_NAMESPACE=camera roslaunch  openni2_tracker_standalone.launch manager:=camera_nodelet_manager is_standalone:=false`



# Maintainer tips

https://github.com/ros-drivers/openni2_tracker/issues/4#issuecomment-221925076

https://github.com/k-okada/primesense-nite2-nonfree-2.0/blob/master/update-primesense-nite2-nonfree

https://github.com/k-okada/primesense-nite2-nonfree-2.0/blob/master/primesense-nite2-nonfree-make-deb
