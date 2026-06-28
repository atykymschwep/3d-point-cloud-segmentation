![Pipeline Result](screenshot.png)

# 3D Point Cloud Segmentation: From Python Prototype to C++ Production

This repository demonstrates a complete computer vision pipeline for processing 3D point clouds, designed for robotic manipulation tasks. It showcases the industry-standard workflow: rapid prototyping and algorithm design in **Python**, followed by a high-performance, production-ready implementation in **C++**.

The pipeline generates a synthetic scene (a tabletop with parts) and applies machine vision algorithms to isolate objects, filter noise, calculate centroids, and construct precise bounding boxes (AABB) for potential robotic grasping.

## 📁 Repository Structure

* `python/` — The R&D prototype. Fast to modify, easy to debug, and relies on `numpy` and `matplotlib` for matrix operations and color mapping.
* `cpp/` — The production engine. Written in C++17 utilizing the Open3D C++ SDK and `Eigen`. Optimized for raw execution speed and strict memory management.

## 🚀 Pipeline Features

1. **Data Generation:** Creates a synthetic 3D point cloud (table, box, cylinder) simulating standard Gaussian sensor noise.
2. **Plane Segmentation (RANSAC):** Mathematically identifies and extracts the main supporting plane (the table) to isolate the target parts.
3. **Clustering (DBSCAN):** Divides the remaining points into independent objects based on spatial density. 
4. **Advanced Noise Filtering:** Custom logic implemented to drop small clusters (e.g., < 100 points), effectively filtering out optical artifacts and RANSAC plane-fitting leftovers.
5. **Spatial Analysis & Bounding Boxes:** Calculates exact centroid coordinates (X, Y, Z) for each detected object and constructs Axis-Aligned Bounding Boxes (AABB) to define physical grasping boundaries.

## 🐍 Python (R&D Prototype)

### Setup
Ensure your package manager is up to date and install the required libraries:

```bash
sudo apt update
sudo apt install python3-pip
pip3 install open3d numpy matplotlib

### Run

```bash
cd python
python3 main.py

## ⚙️ C++ (Production Engine)

### Setup & Prerequisites
The C++ version uses Clang as the compiler to match the official Open3D binaries and avoid ABI incompatibility issues with standard GCC.
Install the Clang compiler and build tools:

```bash
sudo apt update
sudo apt install clang libc++-dev libc++abi-dev cmake build-essential

Download and extract the Open3D C++ SDK to your system (update the CMAKE_PREFIX_PATH in your build command accordingly).

### Build & Run

```bash
cd cpp
mkdir build && cd build
# Generate build files using Clang
cmake -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_PREFIX_PATH=~/open3d_cpp_lib ..
# Compile the project
make -j$(nproc)
# Execute the pipeline
./robot_vision


## 📚 Tech Stack

    **Languages:** Python 3, C++17
    **Core Library:** Open3D (Python API & C++ SDK)
    **Math & Geometry:** NumPy, Eigen
    **Build System:** CMake, Make