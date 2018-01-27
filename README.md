# RPLIDAR Landmark Tracking

## Prerequisites
* RPLIDAR SDK [[website](http://www.slamtec.com/en/Support#rplidar-a1)]
* SDL2 for linux (from repository)

## Steps to compile for linux
1. Install the RPLIDAR SDK from their [website](http://www.slamtec.com/en/Support#rplidar-a1).
2. Compile the SDK on the machine by going into the top-level 'sdk' folder and typing `make`.
3. Move the 'sdk/output/Release/Linux/librplidar_sdk.a' static object file to this library directory i.e. next to 'core.cpp.'
4. Rename 'librplidar_sdk.a' to 'lidar.a'
5. Type `make` in this library folder.
6. Run the 'lidar' program created in the 'bin' folder.
7. Jump with joy. (IMPORTANT)
