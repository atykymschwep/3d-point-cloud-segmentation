#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include <algorithm>
#include <set>
#include <open3d/Open3D.h>

using namespace open3d;

// our custom "Tab10" colormap from Matplotlib
std::vector<Eigen::Vector3d> get_tab10_colors() {
    return {
        {0.121, 0.466, 0.705}, // Blue
        {1.0, 0.498, 0.054},   // Orange
        {0.172, 0.627, 0.172}, // Green
        {0.839, 0.152, 0.156}, // Red
        {0.580, 0.403, 0.741}  // Purple
    };
}

std::shared_ptr<geometry::PointCloud> generate_synthetic_tabletop() {
    std::cout << "1. Generating synthetic dataset..." << std::endl;
    auto pcd = std::make_shared<geometry::PointCloud>();
    
    // C++ random number generator setup
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> xy_dist(-1.0, 1.0);
    std::uniform_real_distribution<> box_x_dist(-0.6, -0.2);
    std::uniform_real_distribution<> box_y_dist(-0.3, 0.3);
    std::uniform_real_distribution<> box_z_dist(0.0, 0.4);
    
    std::uniform_real_distribution<> r_dist(0.0, 0.2);
    std::uniform_real_distribution<> theta_dist(0.0, 2 * M_PI);
    std::uniform_real_distribution<> cyl_z_dist(0.0, 0.5);
    
    std::normal_distribution<> z_dist(0.0, 0.010);

    // --- TABLE ---
    for (int i = 0; i < 20000; ++i) {
        pcd->points_.push_back(Eigen::Vector3d(xy_dist(gen), xy_dist(gen), z_dist(gen)));
    }

    // --- OBJECT 1 (BOX) ---
    for (int i = 0; i < 5000; ++i) {
        pcd->points_.push_back(Eigen::Vector3d(box_x_dist(gen), box_y_dist(gen), box_z_dist(gen)));
    }

    // --- OBJECT 2 (CYLINDER) ---
    for (int i = 0; i < 5000; ++i) {
        double r = r_dist(gen);
        double theta = theta_dist(gen);
        double x = 0.5 + r * std::cos(theta);
        double y = 0.2 + r * std::sin(theta);
        double z = cyl_z_dist(gen);
        pcd->points_.push_back(Eigen::Vector3d(x, y, z));
    }

    return pcd;
}

int main() {
    auto pcd = generate_synthetic_tabletop();

    // --- RANSAC ---
    std::cout << "2. RANSAC: Separating the table from objects..." << std::endl;
    // RANSAC in C++ returns a std::tuple (plane equation and point indices)
    auto plane_tuple = pcd->SegmentPlane(0.05, 3, 1000);
    std::vector<size_t> inliers = std::get<1>(plane_tuple); // Extract indices (inliers)
    
    // Separate clouds (true = invert selection)
    auto table_cloud = pcd->SelectByIndex(inliers);
    auto objects_cloud = pcd->SelectByIndex(inliers, true);

    // --- DBSCAN ---
    std::cout << "3. DBSCAN: Finding object boundaries..." << std::endl;
    std::vector<int> labels = objects_cloud->ClusterDBSCAN(0.08, 20, false);
    
    if (labels.empty()) {
        std::cout << "No objects found." << std::endl;
        return 0;
    }

    // Find the maximum label
    int max_label = *std::max_element(labels.begin(), labels.end());
    std::cout << "Found independent objects: " << max_label + 1 << std::endl;

    // --- COLORING ---
    table_cloud->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    objects_cloud->colors_.resize(objects_cloud->points_.size()); // Allocate memory for colors
    auto palette = get_tab10_colors();

    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] < 0) {
            objects_cloud->colors_[i] = Eigen::Vector3d(0.0, 0.0, 0.0); // Noise to black
        } else {
            // Take color from the palette by index (with array bounds protection)
            objects_cloud->colors_[i] = palette[labels[i] % palette.size()];
        }
    }

    // --- EXTRACTING COORDINATES ---
    std::cout << "\n--- GRASPING DATA ---" << std::endl;
    
    // Create a vector of geometries to draw
    std::vector<std::shared_ptr<const geometry::Geometry>> geometries_to_draw;
    geometries_to_draw.push_back(table_cloud);
    geometries_to_draw.push_back(objects_cloud);
    
    // Get unique labels
    std::set<int> unique_labels(labels.begin(), labels.end());
    unique_labels.erase(-1); // Remove the noise label

    for (int obj_id : unique_labels) {
        // Collect indices of the current object
        std::vector<size_t> obj_indices;
        for (size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] == obj_id) {
                obj_indices.push_back(i);
            }
        }
        auto single_obj_cloud = objects_cloud->SelectByIndex(obj_indices);
        
        // Calculate the centroid
        Eigen::Vector3d center = single_obj_cloud->GetCenter();
        printf("Object #%d: Send robot to -> X: %.2f, Y: %.2f, Z: %.2f\n", 
               obj_id, center.x(), center.y(), center.z());
        
        // Calculate the Axis-Aligned Bounding Box (AABB)
        auto aabb = std::make_shared<geometry::AxisAlignedBoundingBox>(single_obj_cloud->GetAxisAlignedBoundingBox());
        aabb->color_ = Eigen::Vector3d(1.0, 0.0, 0.0); // Red color for the bounding box
        
        geometries_to_draw.push_back(aabb);
    }

    // --- VISUALIZATION ---
    std::cout << "\nOpening the built-in Open3D C++ window..." << std::endl;
    visualization::DrawGeometries(geometries_to_draw, "Coordinates and Dimensions (C++)");

    return 0;
}