import json
import sys
import cv2
import os
import orbslam2
import argparse

def load_images(path_to_association):
    rgb_filenames = []
    timestamps = []
    with open(os.path.join(path_to_association, 'rgb.txt')) as times_file:
        for i, line in enumerate(times_file):
            if i < 3:
                # why ? i dont know, but its in the xample so do it
                continue
            if len(line) > 0 and not line.startswith('#'):
                t, rgb = line.rstrip().split(' ')[0:2]
                rgb_filenames.append(rgb)
                timestamps.append(float(t))
    return rgb_filenames, timestamps


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Usage: ./orbslam_mono_tum path_to_vocabulary path_to_settings path_to_sequence')
    parser.add_argument('vocab_path', help="A path to voabulary file from orbslam. \n"
                                           "E.g: ORB_SLAM/Vocabulary/ORBvoc.txt")
    parser.add_argument('settings_path', help="A path to configurational file, E.g: TUM.yaml")
    parser.add_argument('images_path', help="Path to the folder with images")
    parser.add_argument('save_name', help="a path name to save map to")
    parser.add_argument('--frame_start', type=int, default=0, help="Number of starting frame")
    args = parser.parse_args()
    vocab_path = args.vocab_path
    settings_path = args.settings_path
    filenames, timestamps = load_images(args.images_path)
    data_tuple = [(cv2.imread(os.path.join(sys.argv[3],filename)), float(timestamp)) for filename, timestamp in zip(filenames, timestamps)]
    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
    slam.set_use_viewer(True)
    slam.initialize()
    slam.osmap_init()
    frameno=args.frame_start
    first_datapoint = data_tuple[frameno]
    slam.process_image_mono(first_datapoint[0], first_datapoint[1])
    #slam.map_load(sys.argv[4]+".yaml", False, False)
    #old_timestamps = [kf['mTimeStamp'] for kf in slam.get_keyframe_list()]
    #old_ids = [kf['mnId'] for kf in slam.get_keyframe_list()]
    new_ids = []
    for i, (im, ts) in enumerate(data_tuple[frameno+1:]):
        slam.process_image_mono(im, ts)
    for skf in slam.get_keyframe_list():
        fil_ind = timestamps.index(skf['mTimeStamp'])
        new_ids.append((skf['mnId'], skf['mTimeStamp'], filenames[fil_ind]))
    slam.map_save(args.save_name+"proc_2D", False)
    with open('proc2_assoc_im_kf.json', 'w') as f:
        json.dump( dict(keyframes=new_ids, data_path=args.images_path, map_name=parser.save_name), f)
