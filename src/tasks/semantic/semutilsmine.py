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

def generate_azimuth_angles(num_horizontal_points):
    # Generate azimuth angles evenly spaced from 0 to 360 degrees
    return np.linspace(-180, 0, num_horizontal_points, endpoint=False)

def convert_range_image_to_point_cloud(range_image, beam_inclination, horizontal_fov=360.0):
    # Get the shape of the range image
    num_beams, num_horizontal_points = range_image.shape
    
    # Generate azimuth angles
    azimuth_angles = generate_azimuth_angles(num_horizontal_points)
    
    # Convert angles from degrees to radians
    azimuth_rad = np.radians(azimuth_angles)
    #elevation_rad = np.radians(beam_inclination)
    elevation_rad = beam_inclination
    # Initialize arrays for point cloud
    x_coords = np.zeros_like(range_image)
    y_coords = np.zeros_like(range_image)
    z_coords = np.zeros_like(range_image)
    
    # Compute Cartesian coordinates for each point
    for i in range(num_beams):
        x_coords[i, :] = range_image[i, :] * np.cos(elevation_rad[i]) * np.cos(azimuth_rad)
        y_coords[i, :] = range_image[i, :] * np.cos(elevation_rad[i]) * np.sin(azimuth_rad)
        z_coords[i, :] = range_image[i, :] * np.sin(elevation_rad[i])
    
    # Stack the coordinates to form a point cloud
    point_cloud = np.stack([x_coords, y_coords, z_coords], axis=-1)
    
    return point_cloud


#point_cloud = convert_range_image_to_point_cloud(range_image, beam_inclination)