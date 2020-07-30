import os
from scipy.spatial import transform
from argparse import ArgumentParser
import numpy as np
from logic.calibration import VideoCalibration
import json
from slam_map import OsmapData

from logic.mapper import Mapper2
import pathlib
import cv2
from logic.inverse_perspective import InversePerspective

class Pose3Dto2D:
    def __init__(self, poses):
        coords = self.poses_to_coords(poses)
        plane_normal_vector = self.calculate_plane(coords)
        self.rotation = self.rotation_vector_to_z(plane_normal_vector)
        pose_transform = np.eye(4)
        pose_transform[:3,:3] = self.rotation.as_dcm()
        self.transform = pose_transform


    def poses_to_coords(self, poses):
        coords = [pose[:3,3] for pose in poses]
        return np.array(coords)

    def calculate_plane(self, coords):
        """

        :param coords: np array [[x1,y1,z1], ..., [xn,yn,zn]]
        :return: normal vector to the plane
        """
        G = coords.sum(axis=0) / coords.shape[0]
        u, s, vh = np.linalg.svd(coords - G)
        u_norm = vh[2, :]
        return u_norm

    def rotation_vector_to_z(self, normal_vector) -> transform.Rotation:
        xrot = np.zeros_like(normal_vector)
        yrot = np.zeros_like(normal_vector)
        # first step, get rid of x axis, and look only at y,z
        # calculate the angle between the 2D vector in that plane
        # and the vector pointing up in z
        desired_vector = [0, 1]
        # this is because dot product of v1 * v2 = ||v1||*||v2||*cos(alpha)
        # and ||v1|| in both cases are almost 1
        yz_vector = normal_vector[1:]
        xrot[0] = np.arccos((yz_vector.dot(desired_vector))/np.linalg.norm(yz_vector))  # np.arctan(-normal[1]/normal[2])
        # -xrot, because we want to reverse this operation
        rotx = transform.Rotation.from_euler('xyz', -xrot)
        newax = rotx.as_dcm().dot(normal_vector)
        xz_vector = newax[[0, 2]]
        yrot[1] = np.arccos((xz_vector.dot([0, 1])) / np.linalg.norm(xz_vector))
        tot_rot = -xrot-yrot
        return transform.Rotation.from_euler('xyz', tot_rot)

def get_poses(map_yaml_path):
    my_osmap= OsmapData()
    map_path = pathlib.Path(map_yaml_path)
    my_osmap.read_map(map_path.parent, map_path.stem)
    id_and_pose = my_osmap.id_and_pose()
    return id_and_pose

def ts_fname_association_to_id(assoc_path):
    with open(assoc_path, 'r') as f:
        json_img_assoc = json.loads(f.read())
    img_assoc_dict = { t[0]: (t[1],t[2]) for t in json_img_assoc['keyframes']}
    return img_assoc_dict

def data_iterator(id_and_pose, images_path, assoc_path):
    img_assoc_dict = ts_fname_association_to_id(assoc_path)
    data_path = pathlib.Path(images_path)
    for kf_id, pose in id_and_pose:
        ts, filename = img_assoc_dict[kf_id]
        img = cv2.imread(str(data_path/filename))
        yield kf_id, pose, ts, filename, img



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
