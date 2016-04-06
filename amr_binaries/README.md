Summary
=======

This package contains precompiled binaries of several ROS nodes. In order to
make the usage of binaries architecture-transparent (i.e. so that the user
does not have to care about which version of executable he has to launch
depending on his processor architecture) this package provides "launcher"
scripts for each node. A "launcher" script automatically starts appropriate
binary program (either x32 or x64) depending on the processor architecture.
