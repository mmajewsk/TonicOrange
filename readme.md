#Tonic Orange

This repository contains scripts and source code used to create
map with [ORB_SLAM2](https://github.com/mmajewsk/ORB_SLAM2), [osmap](https://github.com/mmajewsk/osmap), [ORB_SLAM2](https://github.com/mmajewsk/ORB_SLAM2-PythonBindings).
Im using my [Tonic](https://github.com/mmajewsk/Tonic) project to acquire images and data, and then I can process it 
to be able to create map of the environment and also to localise in that map.


## Dependencies

Those scripts are dependant on some custom versions of some libraries.
You should use this repo [TonicSlamDunk](https://github.com/mmajewsk/TonicSlamDunk) to install them.

You can use:
 - **install.sh** - for ubuntu (should be ok on 16.04 and 18.04)
 - or you can use **docker** image that i have prepared [mwmajewsk/tonic_slam_dunk](https://hub.docker.com/repository/docker/mwmajewsk/tonic_slam_dunk).
 
## basic_usage.py

![](https://imgur.com/oA3ERWN.gif)

This script can: 
 - create new ORB_SLAM2 map from images and write it to file
 - read ORB_SLAM2 map from file and extend it with additional data from new set of images, and then save it to file

Parameters:
- **vocab_path** this vocabulary file needed by OrbSlam2
- **settings_path**  - this is settings file needed by the OrbSlam2. 
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

## live_coords.py
@TODO

## direction_finding.py
@TODO

## Developer journal

###26.01.2020
I need to add osmap for python to be generated.
Tt seems that I have lost changes done to the libraries.
One thing that i need to get back is get_keyframe_list, that returns dict of mnid and timestamp
Brought up an issue here:
https://github.com/AlejandroSilvestri/osmap/issues/15
Osmap messes up path management.


### 03.06.2020
So i created a mock for video streeming, I remember having problems with using both path finding andstreaming.