from argparse import ArgumentParser
import cv2
from common.pose import Pose3Dto2D, get_poses, data_iterator
from logic.mapper import Mapper2
from logic.calibration import VideoCalibration
from logic.inverse_perspective import InversePerspective

if __name__ == '__main__':
    ## @MentalMap
    # Calibration is needed to undistort
    # Perspective reverses the image
    parser = ArgumentParser()
    parser.add_argument('calibration', help="path to json calibration file")
    parser.add_argument('map_path', help="path to maps yaml file")
    parser.add_argument('images_path', help="path to folder with images")
    #'/home/mwm/Desktop/datadumps/01-07-19-drive-v1_22'
    parser.add_argument('assoc_file', help="path to assoc.json file")
    #'./assets/maps/testmap1/assoc.json'
    args = parser.parse_args()
    img_shape = 320,240
    calibration = VideoCalibration(args.calibration, img_shape, new_shape=img_shape)
    mapper = Mapper2(map_shape=(800,800,3), type='additive')
    perspective = InversePerspective(img_shape=(250, 250), desired_shape=(100, 130), marigin=0.25)
    id_and_pose = get_poses(args.map_path)
    xy_transformation = Pose3Dto2D([idp[1] for idp in id_and_pose])
    for kf_id, pose, ts, filename, img in data_iterator(id_and_pose, args.images_path, args.assoc_file):
        if True:
            cv2.imshow('img',img)
            undi = calibration.undistort(img)
            undi[:100,:,:] = 0
            img = perspective.inverse(undi)
            cv2.imshow('undi', img)
            cv2.waitKey(0)
        new_pose = xy_transformation.transform.dot(pose)
        new_angles = transform.Rotation.from_dcm(new_pose[:3,:3]).as_euler('xyz')
        position = new_pose[:2,3]
        offset_position = np.array((-position[0],position[1]))
        mapper.update(img, angle=(90-np.rad2deg(new_angles[2])), position=offset_position)
        cv2.imshow('map', mapper.map)
        cv2.waitKey(0)
