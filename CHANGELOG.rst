^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_tim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix memory leak (`#56 <https://github.com/uos/sick_tim/issues/56>`_)
  s should be deleted before returning.
* mrs1000: Make output REP-117 compliant (invalid = +inf)
  This is a port of e964fb4c to the MRS-1000.
* sick mrs1000 driver (`#55 <https://github.com/uos/sick_tim/issues/55>`_)
  This commit adds SICK MRS-1000 support. The initialization of the device
  has to be different, due to that I have made the methods for initialization
  virtual and now the mrs1000 driver runs different init code. Also the
  support for PointCloud2 is new.
* Contributors: Sebastian Pütz, Jochen Sprickerhof, Martin Günther

0.0.11 (2017-12-21)
-------------------
* Make output REP-117 compliant (`#54 <https://github.com/uos/sick_tim/issues/54>`_)
  The laser scan topic now encodes invalid measurements as +inf instead of 0.
  This makes costmap2d treat all invalid measurements as out of range
  measurements and correctly clearing obstacles even when there is no valid
  measurement behind.  This can lead to some obstacles being incorrectly
  cleared (when the "0" returne by the SICK TiM actually means "invalid
  measurement" or "too close to measure" instead of "out of range"), but this
  happens much less frequently in practice than the problem of non-cleared
  obstacles.
* .travis.yml: Add fix for `travis-ci/travis-ci#8048 <https://github.com/travis-ci/travis-ci/issues/8048>`_
* Contributors: Martin Günther

0.0.10 (2017-01-07)
-------------------
* Automatically reboot scanner if it reports an error code. (`#44 <https://github.com/uos/sick_tim/issues/44>`_)
* Update strtok logic. Fixes `#42 <https://github.com/uos/sick_tim/issues/42>`_ (`#43 <https://github.com/uos/sick_tim/issues/43>`_)
* Contributors: Derek King, Jochen Sprickerhof, Martin Guenther

0.0.9 (2016-09-09)
------------------
* timestamp diagnostics must take into account time_offset (`#41 <https://github.com/uos/sick_tim/issues/41>`_)
* Choose one of multiple connected scanners
* Contributors: Christian Reinhard, procopiostein

0.0.8 (2016-04-25)
------------------
* First release into kinetic
* Remove dependency on driver_base
  The driver_base package is end-of-life, and we only needed it because of
  one enum.
* Contributors: Martin Guenther

0.0.7 (2016-04-15)
------------------
* Check for firmware versions without range output
  Closes `#36 <https://github.com/uos/sick_tim/issues/36>`_ .
* Use intensity param in TiM 551/571 parser
  Also print a warning when the intensity param is set, but RSSI is not
  enabled on the scanner. See `#32 <https://github.com/uos/sick_tim/issues/32>`_.
* Other minor changes
* Contributors: Martin Günther

0.0.6 (2015-11-13)
------------------
* First release into Jade
* Create sick_tim571_2050001.launch
  This launch file can be used directly to connect to TIM571 devices.
  See `#28 <https://github.com/uos/sick_tim/issues/28>`_.
* Contributors: Martin Günther, sacuar

0.0.5 (2015-05-06)
------------------
* Auto retry USB and TCP connections due to any reason; see `#25 <https://github.com/uos/sick_tim/issues/25>`_
* Parameterized TCP timeout
* Contributors: Chad Rockey, Martin Günther, Jochen Sprickerhof, Jeff Schmidt

0.0.4 (2015-03-16)
------------------
* Tim561: Make sick_tim551 node work with TiM561.

  - The TiM561 has a angular resolution of 0.33°, which leads to 811 points per scan.
  - Add warning if time_increment is inconsistent. This happens on the TiM561,
    which reports an incorrect measurement frequency.
  - Add ros params to override a few values, including time_increment
    (`#24 <https://github.com/uos/sick_tim/issues/24>`_ ).

* All scanners: Split datagrams up before handing them to parse_datagram.
  This finally fixes the warning on datagrams of invalid length
  each time multiple datagrams are read. (`#21 <https://github.com/uos/sick_tim/issues/21>`_)
* All scanners: add subscribing to datagram topic.
  If subscribe_datagram is set, all nodes will now process the datagrams
  published on the datagram topic instead of reading from the physical
  device. Useful for debugging.
* Contributors: Jochen Sprickerhof, Martin Günther, Michael Ferguson, Michael Görner

0.0.3 (2015-01-09)
------------------
* Merge pull request `#20 <https://github.com/uos/sick_tim/issues/20>`_ from jspricke/fix_19
  Fixes for `#19 <https://github.com/uos/sick_tim/issues/19>`_
* Increase get_datagram timout to 1 second, Closes: `#19 <https://github.com/uos/sick_tim/issues/19>`_
* Add ROS param for TCP port (defaults to 2112)
* fix dependencies in CMakeLists
  All non-catkin things that we expose in our headers should be added to
  the DEPENDS, so that packages which depend on our package will also
  automatically link against it.
  Also see: http://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/`#58593 <https://github.com/uos/sick_tim/issues/58593>`_
* Contributors: Jochen Sprickerhof, Martin Günther

0.0.2 (2014-09-01)
------------------
* Merge pull request `#15 <https://github.com/uos/sick_tim/issues/15>`_ from v4hn/libusb-pkgconfig
  use libusb's pkgconfig support
* Merge pull request `#16 <https://github.com/uos/sick_tim/issues/16>`_ from v4hn/hydro_catkin_fixup
  fixup hydro catkinize
* mark libsick_tim_3xx as exported
* export headers related to libsick_tim3xx
  As far as I know nobody uses them right now,
  but as we install the library, why not provide the interface?
* install meshes & urdf
* add missing external dependencies
  make sure msg headers are built before sick_tim_3xx
* use libusb's pkgconfig support
  This streamlines sick_tim's libusb detection.
* updated URDF: restructuring, add TiM 511
* renamed sick_tim.stl
* added mesh for tim551
* completed rename: sick_tim3xx -> sick_tim
  perl -e 's/sick_tim3xx/sick_tim/g' -pi $(git ls-files)
  perl -e 's/SickTim3xx/SickTim/g' -pi $(git ls-files)
  rename 's/sick_tim3xx/sick_tim/g' $(git ls-files)
  rename 's/SickTim3xx/SickTim/g' $(git ls-files)
* Rename sick_tim3xx -> sick_tim
* Merge pull request `#13 <https://github.com/uos/sick_tim/issues/13>`_ from MadEgg/hydro_improved_tim551
  Hydro improved tim551
* Modified SICK TIM551 parser to accept reduced scanning range and to read out and publish intensity data. Also fixes breaking when a device name has been set.
  Replace hacky bare socket handling with proper boost::asio socket handling in sick_tim3xx_common_tcp.cpp. Introduces dependency on boost::asio 1.46
  Fully functional and tested on tim551.
* omit libusb-1.0 prefix
  automatically configured by CMake
* sick_tim551 launch: add example snippet for enabling TCP
* CMakeLists: rename libsick -> libsick_tim_3xx
  libsick.so was too generic and may cause name conflicts later on
* package.xml: fix incorrect build_depend + run_depend
  build_depend and run_depend can only use either the name of a ros
  package, or something listed in `rosdep db`.
* CMakeLists: add missing catkin_depends, fix depends
* CMakeLists: don't export include dir / libary
  The previous configuration was incorrect: we exported an include path
  that we didn't install. One way to fix this would have been to install
  the headers, but since we don't expect any package outside of
  sick_tim3xx to be using our library, it's better not to install it at
  all.
* Merge pull request `#12 <https://github.com/uos/sick_tim/issues/12>`_ from efernandez/hydro_catkin
  sets dependencies and linking in the library
* sets dependencies and linking in the library
* Merge pull request `#11 <https://github.com/uos/sick_tim/issues/11>`_ from efernandez/hydro_catkin
  renames libsick to sick, so we have libsick.so
* renames libsick to sick, so we have libsick.so
* package.xml: update email addresses, remove .gitignore
* catkinizes sick_tim3xx
* updated manifest.xml
  closes `#8 <https://github.com/uos/sick_tim/issues/8>`_
* common_usb: increase USB_TIMEOUT from 500 to 1000 ms
  This is necessary to make the tim310 work. It (strangely) only publishes
  with 1.875 Hz = one message every 533 ms, so a timeout of 500 ms always
  caused a LIBUSB_ERROR_TIMEOUT.
  This closes `#7 <https://github.com/uos/sick_tim/issues/7>`_.
* fix node name in launch files
* urdf: removed box_inertial_with_origin xacro macro
  this conflicted with a new macro of the same name in
  uos_common_urdf/common.xacro
* fixed warning message
* add select() calls before reading in TCP mode.
  Now diagnostics won't go stale when the device is unplugged but report
  missing data errors correctly. The driver reconnects when the cable is
  plugged again.
* adjusted parameters from real scanner
* Added diagnostics support.
* ~hostname determines if TCP or USB is used.
  Also removed sick_tim3xx_common_tcp from Tim3xx binaries.
* add TCP connection
* prepare option for TCP on sick_tim551_2050001
* split sick_tim3xx_common into common and usb specific stuff
* merged fix from diamondback branch
* updated stack.xml
* add driver for SICK TiM551
* include -> xacro:include
* Don't publish message if there was a parsing error
* more verbose warning when using wrong node
* add launch files for new nodes
* new node sick_tim310_1130000m01 (experimental)
* new node sick_tim310 (experimental)
* renamed sick_tim3xx node to sick_tim310s01
* add test node: sick_tim3xx_datagram_test
* refactoring: split parse_datagram() into own class
* refactoring: split out common code into sick_tim3xx_common
* refactoring: extract function parse_datagram()
* when receiving more fields than expected, print number of fields
* add optional datagram publishing (for debug)
* check return code of init_usb(), exit on failure
* Change udev rule from MODE to GROUP
  User needs to be a member of the plugdev group!
  New udev releases contain a 91-permissions.rules which overwrites the
  MODE="0666". An other workaround would be to move the
  81-sick-tim3xx.rules after the 91-permissions.rules. This patch
  implements a proper fix, which is to use the plugdev group instead.
* update URDF to be compatible with Gazebo 1.5
  In the ros-groovy-simulator-gazebo update to 1.7.12, Gazebo was switched
  over to 1.5, which breaks compatibility with old-style URDFs. This
  commit updates to the new version.
* modified rosdep dependency for compatibility with fuerte
* updated .gitignore
* fix max_angle calculation
* add support for dynamic_reconfigure parameters
* don't dump scans to rosout on error
  usually, this happens when we're lagging behind due to a different
  error; printing the stuff to rosout slows down the whole process enough
  so we never catch up.
* changed default laser_frame to "laser", made xacro macro
* adjust time stamp
  - last scan point = now  ==>  first scan point = now - 271 * time increment
  - also just assume 0.001 s USB latency between scanner and PC for now
  this avoids TF ExtrapolationExceptions (cannot project into future)
* fixed frame name in gazebo URDF
* URDF: renamed changed box_inertial
  ... because it doesn't play well with our other URDFs in
  kurt_description
* URDF: introduced xacro properties for constants
* add launch file
* add URDF file and mesh for scanner
* shift angle_min and angle_max by -PI/2
  now angle_min = -135° and angle_max = +135°
* turned everything into a class
  reason: this allows us to call all the cleanup code from the destructor,
  so we can make sure it's called every time we exit
* properly exit on error, improved logging
* change default frame name to fully qualified /laser_link
* fix illegal write detected by valgrind
* updated udev README
* working implementation
* copy SICK example code, start conversion to ROS
* description in manifest
* add includes, rosdep dependency on libusb
* add BSD license header
* add udev rules
* add code skeleton for node
* add roscpp dependency
* initial commit
* Contributors: Christian Dornhege, Egbert van der Wal, Jochen Sprickerhof, Martin Günther, Michael Görner, enriquefernandez, v4hn
