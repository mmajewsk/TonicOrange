import os
from scipy.spatial import transform
import numpy as np
import json
import cv2
import pathlib
from common.slam_map import OsmapData

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

class Transform3Dto2D:
    def __init__(self, my_osmap):
        self.pose_transformer = Pose3Dto2D(my_osmap.poses_reshaped())

    def transform(self, pose):
        new_pose = self.pose_transformer.transform.dot(pose)
        new_angles = transform.Rotation.from_dcm(new_pose[:3, :3]).as_euler('xyz')
        position = new_pose[:2, 3]
        # !!! new_angles[2] + np.pi !!!!
        return position, new_angles[2]

def calculate_desired_angle(current_pose2D, destination2D):
        (x1, y1), phi1 = current_pose2D
        (x2, y2), phi2 = destination2D
        phi1_d = phi1 + np.pi
        a = np.abs(x1 - x2)
        b = np.abs(y1 - y2)
        p1_p2_angle = np.arctan(b / a)
        return p1_p2_angle
