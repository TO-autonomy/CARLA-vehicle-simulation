import numpy as np
import urdf_parser_py.urdf as urdf
from scipy.spatial.transform import Rotation 

class URDFParser:
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.robot = urdf.URDF.from_xml_file(urdf_file)
        self.root = self.robot.get_root()

    def compute_chain_transform(self, chain):
        transform = np.eye(4)
        
        for joint in chain:
            if joint not in self.robot.joint_map:
                continue
            
            joint_info = self.robot.joint_map[joint]
            rpy = joint_info.origin.rpy
            xyz = joint_info.origin.xyz
            rotation = Rotation.from_euler('xyz', rpy).as_matrix()
            translation = np.array(xyz)
            T = self.build_transform_matrix(rotation, translation)
            transform = np.dot(transform, T)
        
        return transform

    def get_T_from_to(self, start_frame, end_frame):
        chain_1 = self.robot.get_chain(self.root, start_frame)
        chain_2 = self.robot.get_chain(self.root, end_frame)
        T1 = self.compute_chain_transform(chain_1)
        T2 = self.compute_chain_transform(chain_2)
        return np.dot(np.linalg.inv(T1), T2)
    
    def build_transform_matrix(self, rotation, translation):
        m = np.eye(4)
        m[:3, :3] = rotation
        m[:3, 3] = translation
        return m