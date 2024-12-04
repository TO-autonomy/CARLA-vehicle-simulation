import numpy as np

def compute_camera_matrix(K, transformation_matrix):
    R_c2w = transformation_matrix[:3, :3]
    t_c2w = transformation_matrix[:3, 3]
    R_w2c = R_c2w.T
    t_w2c = -R_w2c @ t_c2w
    extrinsic_matrix = np.hstack((R_w2c, t_w2c.reshape(3, 1)))  # Shape: (3, 4)
    P = K @ extrinsic_matrix  # Shape: (3, 4)
    return P

def compute_camera_matrix_4x4(K, transformation_matrix):
    P = compute_camera_matrix(K, transformation_matrix)  # 3x4 matrix
    P_4x4 = np.vstack((P, [0, 0, 0, 1]))  # Shape: (4, 4)
    return P_4x4

class Camera():
    def __init__(self, intrinsics_matrix, extrinsics_matrix, name="unknown"):
        self.name = name
        self.intrinsics_matrix = intrinsics_matrix
        self.extrinsics_matrix = extrinsics_matrix

    def get_fov(self, in_degrees=True):
        if in_degrees:
            return self.get_fov_degrees()
        return self.get_fov_radians()

    def get_fov_radians(self):
        image_width = self.get_native_image_width()
        focal_length = self.get_focal_length()
        fov_radians = 2 * np.arctan(image_width / (2 * focal_length))
        fov_degrees = np.degrees(fov_radians)
        return fov_degrees

    def get_name(self):
        return self.name
    
    def get_intrinsics_matrix(self):
        return self.intrinsics_matrix
    
    def get_projection_matrix(self):
        intrinsics_matrix = self.get_intrinsics_matrix()
        intrinsics_matrix = intrinsics_matrix[:3, :3]
        extrinsics_matrix = self.get_extrinsic_matrix()
        return compute_camera_matrix(intrinsics_matrix, extrinsics_matrix)
    
    def get_extrinsic_matrix(self):
        return self.extrinsics_matrix
    
    def get_fx(self):
        return self.intrinsics_matrix[0, 0]
    
    def get_fy(self):
        return self.intrinsics_matrix[1, 1]
    
    def get_focal_length(self):
        return (self.intrinsics_matrix[0, 0] + self.intrinsics_matrix[1, 1]) / 2
    
    def get_ppx(self):
        return self.intrinsics_matrix[0, 2]
    
    def get_ppy(self):
        return self.intrinsics_matrix[1, 2]
    
    def get_native_image_width(self):
        return self.intrinsics_matrix[0, 2] * 2
    
    def get_native_image_height(self):
        return self.intrinsics_matrix[1, 2] * 2
    
    def get_distortion(self):
        return self.intrinsics_matrix[2, :]
    
    def get_position(self):
        return self.extrinsics_matrix[:3, 3]
    
    def get_rotation(self):
        return self.extrinsics_matrix[:3, :3]
    
    def get_pitch(self):
        return np.arcsin(-self.get_rotation()[2, 0])
    
    def get_yaw(self):
        return np.arctan2(self.get_rotation()[1, 0], self.get_rotation()[0, 0])
    
    def get_roll(self):
        return np.arctan2(self.get_rotation()[2, 1], self.get_rotation()[2, 2])