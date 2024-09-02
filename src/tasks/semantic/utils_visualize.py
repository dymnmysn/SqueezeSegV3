import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def visualize_point_cloud_with_segmentation(point_cloud, segmentation_classes, num_classes=3):
    # Reshape the point cloud to (N, 3)
    points = point_cloud.reshape(-1, 3)
    
    # Reshape segmentation classes to (N,)
    class_indices = segmentation_classes.reshape(-1)
    
    # Generate distinct colors for each class
    cmap = plt.get_cmap('tab20', num_classes)
    colors = cmap(class_indices % num_classes)[:, :3]  # Map classes to colors

    # Create an Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Visualize the point cloud with colors based on segmentation classes
    o3d.visualization.draw_geometries([pcd])

# Example usage:
# Assuming 'point_cloud' is your (64, 2650, 3) numpy array
# Assuming 'segmentation_classes' is your (64, 2650) numpy array containing class indices

#visualize_point_cloud_with_segmentation(point_cloud, segmclasses)

def visualize_point_cloud(point_cloud):
    # Reshape the point cloud to a (N, 3) shape where N is the number of points
    points = point_cloud.reshape(-1, 3)
    
    # Create an Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    
    # Set the points to the PointCloud object
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])

# Assuming 'point_cloud' is your (64, 2650, 3) numpy array
#visualize_point_cloud(point_cloud)

