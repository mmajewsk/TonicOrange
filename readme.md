This repository contains scripts and source code used to create map with [ORB_SLAM2](https://github.com/mmajewsk/ORB_SLAM2), [osmap](https://github.com/mmajewsk/osmap), [ORB_SLAM2](https://github.com/mmajewsk/ORB_SLAM2-PythonBindings).
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

### 22.06.2020

Scripts are running as they should. lets see again where's the problem.
So i added an possiblity of writing a log to direction finding.

### 30.06.2020

I added a DumbDriver in `direction_finding.py` Its not finished, todos in method
`direction_finding.py:238`. Do them.

### 09.07.2020
\
Use 
```
sudo lsof -i:8080
```

To find busy port
test in hackerspace
the batteries seems to be ok
trying to record some paths in hardroom

First run Tonic/src/pc:

```
python run.py -v -s --dump_video ~/repositories/Tonic/data_intake4/09_07_2020_hackerspace_v0.01
```

to take data, then create rgb file with

```
python tools.py ~/repositories/Tonic/data_intake4/09_07_2020_hackerspace_v0.01/
```


```python basic_usage.py /home/mwm/repositories/slam_dunk/builds/data/ORBvoc.txt /home/mwm/repositories/os2py_applied/TUM-MINE-wide.yaml /home/mwm/repositories/Tonic/data_intake4/17_07_2020_hackerspace_v0.01/```
then use the algorithm to create map

### 17.07.2020

After the battery change, and added battery monitor, I will try to run
everything again. 

`python basic_usage.py /home/mwm/repositories/slam_dunk/builds/data/ORBvoc.txt /home/mwm/repositories/os2py_applied/TUM-MINE-wide.yaml /home/mwm/repositories/Tonic/data_intake4/17_07_2020_hackerspace_v0.03/ --start 250 --end 400 `


doing stuff at home:

So there is aproblem with steering it seems stuck or doeasnt go in "rest" state
(no speed) at all.

### 19.07.2020

For whatever the reason, the image in she slam is not getting there fast enough.
This might be due to the way that it ic received, or slowness of the SLAM.

### 23.07.2020

Attempting to profile this problem with mock.

### 24.07.2020

All the profilers are failing me, because of the exit bug.
So the get_current_pose is the choke-point.
    I think it's beacuse of the conversion to opencv mat.
Ok it is not. Thats a mess.
I need to compare basic usage wiht path finder.
Problem solved, mainly those were mine sleeps (usleep in python wrapper!).
Sort this mess out, and commit this later.

### 26.07.2020

The Tonic/pathfinder branch needs merge in the future, also needs to be hcecked
out against the code on the device.

### 12.08.2020

It works perfectly, with the exception of the steering. Something is wrong, I need better visualisation online (preferably as 2d map )
