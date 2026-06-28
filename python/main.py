import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def generate_synthetic_tabletop():
    print("1. Generating synthetic dataset...")
    
    # --- TABLE ---
    table_x = np.random.uniform(-1, 1, 20000)
    table_y = np.random.uniform(-1, 1, 20000)
    table_z = np.random.normal(0, 0.010, 20000) 
    table_points = np.vstack((table_x, table_y, table_z)).T

    # --- OBJECT 1 (BOX) ---
    box_x = np.random.uniform(-0.6, -0.2, 5000)
    box_y = np.random.uniform(-0.3, 0.3, 5000)
    box_z = np.random.uniform(0, 0.4, 5000)
    box_points = np.vstack((box_x, box_y, box_z)).T

    # --- OBJECT 2 (CYLINDER) ---
    cyl_r = np.random.uniform(0, 0.2, 5000)
    cyl_theta = np.random.uniform(0, 2*np.pi, 5000)
    cyl_x = 0.5 + cyl_r * np.cos(cyl_theta)
    cyl_y = 0.2 + cyl_r * np.sin(cyl_theta)
    cyl_z = np.random.uniform(0, 0.5, 5000)
    cyl_points = np.vstack((cyl_x, cyl_y, cyl_z)).T

    # Combine everything into a single raw point cloud
    all_points = np.vstack((table_points, box_points, cyl_points))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)
    return pcd

def main():
    pcd = generate_synthetic_tabletop()

    # --- RANSAC (Finding the table) ---
    print("2. RANSAC: Separating the table from objects...")
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
    
    table_cloud = pcd.select_by_index(inliers)
    objects_cloud = pcd.select_by_index(inliers, invert=True)

    # --- DBSCAN (Clustering objects) ---
    print("3. DBSCAN: Finding object boundaries...")
    labels = np.array(objects_cloud.cluster_dbscan(eps=0.08, min_points=20, print_progress=False))
    max_label = labels.max()
    print(f"Found independent objects: {max_label + 1}")

    # --- COLORING ---
    table_cloud.paint_uniform_color([0.5, 0.5, 0.5]) 
    colors = plt.get_cmap("tab10")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0 
    objects_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # --- EXTRACTING COORDINATES FOR THE ROBOT ---
    print("\n--- GRASPING DATA ---")
    
    # Create a list of geometries to render
    geometries_to_draw = [table_cloud, objects_cloud]
    
    # Find all unique object labels (0 and 1), ignoring noise (-1)
    unique_labels = set(labels)
    unique_labels.discard(-1)

    # Start loop: iterate through each found object
    for obj_id in unique_labels:
        # Filter the array, keeping points only for the CURRENT object
        obj_indices = np.where(labels == obj_id)[0]
        single_obj_cloud = objects_cloud.select_by_index(obj_indices)
        
        # Calculate the mathematical center (Centroid)
        center = single_obj_cloud.get_center()
        print(f"Object #{obj_id}: Send robot to -> X: {center[0]:.2f}, Y: {center[1]:.2f}, Z: {center[2]:.2f}")
        
        # Calculate the bounding box
        aabb = single_obj_cloud.get_axis_aligned_bounding_box()
        aabb.color = (1, 0, 0)
        geometries_to_draw.append(aabb)

    # --- BUILT-IN VISUALIZATION ---
    print("\nOpening the built-in Open3D 3D window...")
    o3d.visualization.draw_geometries(geometries_to_draw, window_name="Coordinates and Dimensions")

if __name__ == "__main__":
    main()