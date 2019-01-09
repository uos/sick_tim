sick_tim
========

For documentation, please see [sick_tim at the ROS wiki](http://wiki.ros.org/sick_tim).

Setting up udev rules
---------------------

**Note:** The following steps are only required when installing the package
from source. When installing a binary debian package of `sick_tim` >= 0.0.14,
the udev rules are set up automatically. Also, this is only required if
connecting to the scanner via USB.

To give all members of the plugdev group write access to the Sick TiM devices, run the following
commands from the root of the `sick_tim` repository:

```bash
sudo cp debian/udev /etc/udev/rules.d/81-sick-tim3xx.rules
sudo udevadm control --reload-rules
```

Make sure that your current user is a member of the plugdev group by running
`groups`. If not, add the user to the group and login again.

Now unplug your USB cable and plug it in again. This will allow you to
communicate with the laser scanner without running the node as root and fix the
following error:

```
LIBUSB - Cannot open device (permission denied?); please read sick_tim/README.md
```


Travis - Continuous Integration
-------------------------------

| Indigo |
|--------|
| [![Build Status](https://travis-ci.org/uos/sick_tim.svg?branch=indigo)](https://travis-ci.org/uos/sick_tim) |


ROS Buildfarm
-------------

|           | binary deb | source deb | devel | doc |
|-----------|------------|------------|-------|-----|
| indigo | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__sick_tim__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__sick_tim__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__sick_tim__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__sick_tim__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__sick_tim__ubuntu_trusty_amd64)](http://build.ros.org/job/Idev__sick_tim__ubuntu_trusty_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idoc__sick_tim__ubuntu_trusty_amd64)](http://build.ros.org/job/Idoc__sick_tim__ubuntu_trusty_amd64) |
| kinetic | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__sick_tim__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__sick_tim__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__sick_tim__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__sick_tim__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__sick_tim__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__sick_tim__ubuntu_xenial_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdoc__sick_tim__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__sick_tim__ubuntu_xenial_amd64) |
| lunar | [![Build Status](http://build.ros.org/buildStatus/icon?job=Lbin_uX64__sick_tim__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Lbin_uX64__sick_tim__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Lsrc_uX__sick_tim__ubuntu_xenial__source)](http://build.ros.org/job/Lsrc_uX__sick_tim__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ldev__sick_tim__ubuntu_xenial_amd64)](http://build.ros.org/job/Ldev__sick_tim__ubuntu_xenial_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ldoc__sick_tim__ubuntu_xenial_amd64)](http://build.ros.org/job/Ldoc__sick_tim__ubuntu_xenial_amd64) |
| melodic | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__sick_tim__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__sick_tim__ubuntu_bionic_amd64__binary) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__sick_tim__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__sick_tim__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__sick_tim__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__sick_tim__ubuntu_bionic_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdoc__sick_tim__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdoc__sick_tim__ubuntu_bionic_amd64) |
