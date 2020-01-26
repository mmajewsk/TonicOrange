#Tonic Orange

This repository contains scripts and source code used to create
map with [ORB_SLAM2]().

## Dependencies

Those scripts are dependant on some custom versions of some libraries.
You should use this repo [TonicSlamDunk](https://github.com/mmajewsk/TonicSlamDunk) to install them.

You can use:
 - **install.sh** - for ubuntu (should be ok on 16.04)
 - or you can use **docker** image that i have prepared [mwmajewsk/tonic_slam_dunk](https://hub.docker.com/repository/docker/mwmajewsk/tonic_slam_dunk).
 
 

## basic_usage.py

- **vocab_path** this vocabulary file needed by OrbSlam2
- **settings_path**  - this is settings fiel needed by the OrbSlam2. 
The file contains settings for the camera and its callibration.
- **images_path** - path to folder with images
- **save_name** - where to save the map


This script will save the map created by OrbSlam2 using protobuff file. 

```
Usage: ./orbslam_mono_tum path_to_vocabulary path_to_settings path_to_sequence

positional arguments:
  vocab_path            A path to voabulary file from orbslam. E.g:
                        ORB_SLAM/Vocabulary/ORBvoc.txt
  settings_path         A path to configurational file, E.g: TUM.yaml
  images_path           Path to the folder with images
  save_name             a path name to save map to

optional arguments:
  -h, --help            show this help message and exit
  --frame_start FRAME_START
                        Number of starting frame

```
