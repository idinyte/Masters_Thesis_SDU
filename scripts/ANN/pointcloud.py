import numpy as np
import open3d as o3d
import pybullet as p
from tqdm import tqdm
import random

class PointCloud():
    def __init__(self):
        # Camera parameters
        self.camera_position = [0, -0.5, 2]
        self.camera_target = [0, -0.5, 1]
        self.camera_resolution_width = 640
        self.camera_resolution_height = 480
        self.up_vector = [0, -1, 0]
        self.fov = 60
        self.aspect = self.camera_resolution_width / self.camera_resolution_height
        self.near = 0.01
        self.far = 1000

        # http://ksimek.github.io/2013/08/13/intrinsic/
        self.view_matrix = p.computeViewMatrix(self.camera_position, self.camera_target, self.up_vector)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.near, self.far)
      
    def export_to_pcd(self, point_cloud, filename):
      o3d.io.write_point_cloud(filename, point_cloud)
      
    def voxel_grid_process(self, input_cloud, voxel_grid_size):
      # Downsample the pointcloud using a Voxel Grid Filter.
      return input_cloud.voxel_down_sample(voxel_grid_size)
    
    def crop_point_cloud_aabb(self, point_cloud, min_bound, max_bound):
      aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
      cropped_cloud = point_cloud.crop(aabb)
      return cropped_cloud
    
    def get_point_cloud(self):
      image_arr = p.getCameraImage(width=self.camera_resolution_width, height=self.camera_resolution_height, viewMatrix=self.view_matrix, projectionMatrix=self.projection_matrix)
      depth_np_arr = image_arr[3]
    
      point_cloud_NDC = []
      projectionMatrix = np.asarray(self.projection_matrix).reshape([4,4], order='F') # Projects points from the camera’s view space into NDC
      viewMatrix = np.asarray(self.view_matrix).reshape([4,4], order='F') # Transforms points from world space to the camera’s view space.
      ncd_to_world = np.linalg.inv(projectionMatrix @ viewMatrix)
      for h in range(self.camera_resolution_height):
          for w in range(self.camera_resolution_width):
            
              # Convert to normalized device coordinates NDC range [-1, 1]
              x = (2*w - self.camera_resolution_width)/self.camera_resolution_width
              y = -(2*h - self.camera_resolution_height)/self.camera_resolution_height
              z = 2*depth_np_arr[h,w] - 1
              point_cloud_NDC.append([x, y, z, 1])

      
      point_cloud_world = ncd_to_world @ np.array(point_cloud_NDC).T
      point_cloud_world = point_cloud_world[:3, :] / point_cloud_world[3, :]
      point_cloud_world = point_cloud_world.T

      pcd = o3d.geometry.PointCloud()
      pcd.points = o3d.utility.Vector3dVector(point_cloud_world)
      return pcd
    
    def get_point_cloud_from_object(self, object, num_points):
      mesh = o3d.io.read_triangle_mesh(object)
      point_cloud = mesh.sample_points_poisson_disk(number_of_points=num_points)
      return point_cloud
    
    def global_alignment(self, scene, object):
      ransac_iterations = 200
      inlier_threshold = 0.01**2

      # Feature extraction
      object.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(10))
      scene.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(10))

      object_features = o3d.pipelines.registration.compute_fpfh_feature(object, search_param=o3d.geometry.KDTreeSearchParamRadius(0.1))
      scene_features = o3d.pipelines.registration.compute_fpfh_feature(scene, search_param=o3d.geometry.KDTreeSearchParamRadius(0.1))

      object_features = np.asarray(object_features.data).T
      scene_features = np.asarray(scene_features.data).T

      # Feature matches
      correspondences = o3d.utility.Vector2iVector()
      feature_correspondence_step = 2
      for j in tqdm(range(0, object_features.shape[0], feature_correspondence_step), desc='Feature matches'):
          dist = np.sum((object_features[j] - scene_features)**2, axis=-1)
          correspondences.append((j, np.argmin(dist)))

      # RANSAC
      tree = o3d.geometry.KDTreeFlann(scene)
      inliers_best = 0
      for i in tqdm(range(ransac_iterations), desc='RANSAC'):   
          correspondence_sample = o3d.utility.Vector2iVector(random.choices(correspondences, k=3))
          
          # Estimate transformation
          est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
          T = est.compute_transformation(object, scene, correspondence_sample)
          
          # Apply pose
          object_aligned = o3d.geometry.PointCloud(object)
          object_aligned.transform(T)
          
          # Count inliers
          inliers = 0
          for j in range(len(object_aligned.points)):
              k, idx, dist = tree.search_knn_vector_3d(object_aligned.points[j], 1)
              if dist[0] < inlier_threshold:
                  inliers += 1

          # Update result
          if inliers > inliers_best:
              inliers_best = inliers
              pose = T
              print(f'{inliers}/{len(object_aligned.points)} inliers')
      
      return pose