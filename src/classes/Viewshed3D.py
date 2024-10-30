import numpy as np
from numpy.linalg import svd

def bresenham_3d_numba(start, end):
    x1, y1, z1 = start
    x2, y2, z2 = end

    # x1 = int64(x1)
    # y1 = int64(y1)
    # z1 = int64(z1)
    # x2 = int64(x2)
    # y2 = int64(y2)
    # z2 = int64(z2)

    dx, dy, dz = abs(x2 - x1), abs(y2 - y1), abs(z2 - z1)
    sx = 1 if x2 > x1 else -1
    sy = 1 if y2 > y1 else -1
    sz = 1 if z2 > z1 else -1

    path = []

    if dx >= dy and dx >= dz:
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while x1 != x2:
            path.append((x1, y1, z1))
            x1 += sx
            if p1 >= 0:
                y1 += sy
                p1 -= 2 * dx
            if p2 >= 0:
                z1 += sz
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
    elif dy >= dx and dy >= dz:
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while y1 != y2:
            path.append((x1, y1, z1))
            y1 += sy
            if p1 >= 0:
                x1 += sx
                p1 -= 2 * dy
            if p2 >= 0:
                z1 += sz
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
    else:
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while z1 != z2:
            path.append((x1, y1, z1))
            z1 += sz
            if p1 >= 0:
                y1 += sy
                p1 -= 2 * dz
            if p2 >= 0:
                x1 += sx
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx

    # Include the last point
    path.append((x2, y2, z2))

    return path

class Viewshed3D:
    def __init__(self, voxel_centroids, voxel_size, voxel_grid_dims=None):
        self.voxel_centroids = voxel_centroids  # Occupied voxels in the world
        self.voxel_size = voxel_size
        self.build_voxel_grid(voxel_grid_dims)
        self.build_occupancy_grid()

    def build_voxel_grid(self, grid_dims):
        # Create a mapping from voxel indices to occupancy status
        if grid_dims is None:
            self.x_min, self.y_min, self.z_min = self.voxel_centroids.min(axis=0) - self.voxel_size / 2
            self.x_max, self.y_max, self.z_max = self.voxel_centroids.max(axis=0) + self.voxel_size / 2

            # Compute grid dimensions
            self.grid_dims = np.ceil(np.array([
                self.x_max - self.x_min,
                self.y_max - self.y_min,
                self.z_max - self.z_min
            ]) / self.voxel_size).astype(int)
        else:
            self.grid_dims = np.array(grid_dims)
            x_min_grid, y_min_grid, z_min_grid = -self.grid_dims // 2
            self.x_min, self.y_min, self.z_min = x_min_grid * self.voxel_size, y_min_grid * self.voxel_size, z_min_grid * self.voxel_size
            x_max_grid, y_max_grid, z_max_grid = self.grid_dims // 2
            self.x_max, self.y_max, self.z_max = x_max_grid * self.voxel_size, y_max_grid * self.voxel_size, z_max_grid * self.voxel_size  

        print(self.x_min, self.y_min, self.z_min)   
        
    def build_occupancy_grid(self):
        self.occupancy_grid_array = np.zeros(self.grid_dims, dtype=bool)

        # Map occupied voxels to grid indices
        for centroid in self.voxel_centroids:
            idx = self.world_to_grid_index(centroid)
            x_idx, y_idx, z_idx = idx
            if (0 <= x_idx < self.grid_dims[0] and
                0 <= y_idx < self.grid_dims[1] and
                0 <= z_idx < self.grid_dims[2]):
                self.occupancy_grid_array[x_idx, y_idx, z_idx] = True

    def world_to_grid_index(self, point):
        idx = ((point - np.array([self.x_min, self.y_min, self.z_min])) / self.voxel_size).astype(int)
        return tuple(idx)

    def grid_index_to_world(self, idx):
        point = np.array(idx) * self.voxel_size + np.array([self.x_min, self.y_min, self.z_min]) + self.voxel_size / 2
        return point

    def compute_visible_voxels(self, camera_matrix, image_width, image_height):
        homogenous_voxels = np.hstack((self.voxel_centroids, np.ones((self.voxel_centroids.shape[0], 1))))  # (N, 4)
        projected_voxels_homogeneous = (camera_matrix @ homogenous_voxels.T).T  # (N, 4)
        projected_voxels = projected_voxels_homogeneous[:, :2] / projected_voxels_homogeneous[:, 2:3]  # (N, 2)

        in_front = projected_voxels_homogeneous[:, 2] > 0
        u = projected_voxels[:, 0]
        v = projected_voxels[:, 1]
        in_image = (u >= 0) & (u < image_width) & (v >= 0) & (v < image_height)
        valid_voxels_mask = in_front & in_image

        visible_voxel_indices = np.where(valid_voxels_mask)[0]
        P = camera_matrix[:3, :]
        _, _, Vt = svd(P)
        C_homogeneous = Vt[-1]
        C_homogeneous = C_homogeneous / C_homogeneous[-1]  
        camera_position = C_homogeneous[:3]

        combined_centroids = []
        for idx in visible_voxel_indices:
            voxel = self.voxel_centroids[idx]
            # Instead of computing direction and max_range, use start and end points
            start_point = camera_position
            end_point = voxel

            voxel_indices_along_ray = self.traverse_ray(start_point, end_point)
            for voxel_idx in voxel_indices_along_ray:
                # Check bounds
                if not ((0 <= voxel_idx[0] < self.grid_dims[0]) and
                        (0 <= voxel_idx[1] < self.grid_dims[1]) and
                        (0 <= voxel_idx[2] < self.grid_dims[2])):
                    break  # Out of bounds

                if self.occupancy_grid_array[voxel_idx]:
                    # Occupied voxel found
                    combined_centroids.append(self.grid_index_to_world(voxel_idx))
                    break  # Stop traversing this ray
                else:
                    # Free space voxel
                    combined_centroids.append(self.grid_index_to_world(voxel_idx))

        return np.array(combined_centroids)

    def traverse_ray(self, start_point, end_point):
        # Convert world coordinates to grid indices
        start_idx = self.world_to_grid_index(start_point)
        end_idx = self.world_to_grid_index(end_point)

        # Use the 3D Bresenham algorithm to get the voxels along the line
        voxel_indices = bresenham_3d_numba(start_idx, end_idx)

        return voxel_indices