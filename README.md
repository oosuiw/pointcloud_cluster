---
## LiDAR-based Object Detection with NDT/ICP in ROS2 Humble
---

### Introduction

This ROS2 Humble C++ package performs **LiDAR-based object detection** by registering live point cloud scan data with a pre-defined 3D model (in `.ply` format). It leverages either the **Normal Distributions Transform (NDT)** or **Iterative Closest Point (ICP)** algorithm to find the optimal pose of the model within the scan data. Upon successful detection, the package visualizes a **bounding box** around the detected object in RViz2 and displays a **fitness score** indicating the quality of the match.

This tool is ideal for applications requiring precise object localization, such as robotic manipulation, autonomous navigation, or quality inspection.

### Features

* **Model Loading**: Loads a 3D object model from a `.ply` file.
* **NDT/ICP Registration**: Configurable to use either NDT or ICP algorithms for point cloud registration.
* **Real-time Processing**: Subscribes to LiDAR scan data (or any `sensor_msgs/msg/PointCloud2` topic) for continuous object detection.
* **Object Detection Scoring**: Calculates and displays a fitness score to quantify the similarity between the scan data and the registered model.
* **RViz2 Visualization**:
    * Publishes the processed (downsampled) point cloud.
    * Publishes a **3D bounding box** around the detected object.
    * Displays the **detection score as a text overlay** in RViz2.
* **Parameter Configuration**: Easily tune registration and detection parameters via a YAML configuration file.

### Prerequisites

* **ROS2 Humble**: This package is developed and tested on ROS2 Humble.
* **PCL (Point Cloud Library)**: Essential for point cloud processing and registration algorithms.
    * Ensure PCL is installed and configured for ROS2. You can install it via:
        ```bash
        sudo apt update
        sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
        ```
* **Eigen3**: For matrix and quaternion operations (usually installed with ROS/PCL).

### Installation

1.  **Create a ROS2 Workspace (if you don't have one):**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  **Clone the Repository:**
    ```bash
    git clone https://github.com/oosuiw/pointcloud_cluster.git
    ```

3.  **Place your 3D Model:**
    * Put your `.ply` model file (e.g., `my_model.ply`) into the `pointcloud_cluster/models/` directory.

4.  **Build the Package:**
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --packages-select pointcloud_cluster
    source install/setup.bash
    ```

### Usage

1.  **Configure Parameters:**
    Edit `pointcloud_cluster/config/params.yaml` to adjust registration parameters, model path, and the input LiDAR topic.

    ```yaml
    # config/params.yaml
    pointcloud_clusterer:
      ros__parameters:
        model_path: "models/my_model.ply" # Name of your PLY model in the 'models' folder
        registration_method: "ICP" # Choose "ICP" or "NDT"
        output_topic: "processed_pointcloud"
        scan_topic: "/your/lidar/topic" # <--- IMPORTANT: Change this to your actual LiDAR topic (e.g., /velodyne_points, /camera/depth/color/points)
        
        # ICP Parameters
        icp_max_iterations: 100
        icp_transformation_epsilon: 1e-8
        icp_euclidean_fitness_epsilon: 1e-4
        icp_max_correspondence_distance: 0.1
        icp_voxel_leaf_size: 0.02 # Voxel grid filter size for ICP source cloud (e.g., 0.02m)

        # NDT Parameters
        ndt_resolution: 1.0
        ndt_step_size: 0.1
        ndt_transformation_epsilon: 0.01
        ndt_max_iterations: 30
        ndt_voxel_leaf_size: 0.05 # Voxel grid filter size for NDT source cloud (e.g., 0.05m)

        # Object Detection Threshold
        detection_score_threshold: 0.05 # Lower score indicates better match. Adjust this value.
    ```

2.  **Launch the Node:**
    Run the package using the provided launch file. Remember to replace `/your/lidar/topic` with your actual LiDAR topic.

    ```bash
    # Example using ICP
    ros2 launch pointcloud_cluster cluster_node.launch.py scan_topic:=/your/lidar/topic registration_method:=ICP

    # Example using NDT
    ros2 launch pointcloud_cluster cluster_node.launch.py scan_topic:=/your/lidar/topic registration_method:=NDT
    ```

3.  **Visualize in RViz2:**
    Launch RViz2 to visualize the results:
    ```bash
    rviz2
    ```
    In RViz2:
    * Set the **`Fixed Frame`** to the frame ID of your LiDAR (`/your/lidar/topic`'s `header.frame_id`), e.g., `base_link`, `lidar_link`, `camera_link`.
    * Add a **`PointCloud2`** display and subscribe to the `/processed_pointcloud` topic.
    * Add a **`Marker`** display and subscribe to the `/detected_object_bbox` topic. This will show both the bounding box and the score text.
    * (Optional) Add another `PointCloud2` display for your original LiDAR topic (`/your/lidar/topic`) to see the raw input.

### Customization and Tuning

* **Model Accuracy**: The quality of your `.ply` model significantly impacts detection performance. Ensure it's accurate and appropriately scaled.
* **Initial Pose**: ICP and NDT can be sensitive to the initial pose. If your object is not typically in a known approximate location, you might need an initial pose estimation step (e.g., from an approximate bounding box or a prior localization system).
* **Registration Parameters**: Experiment with `icp_max_iterations`, `icp_max_correspondence_distance`, `ndt_resolution`, etc., to find the best settings for your specific model and environment.
* **Voxel Leaf Size**: Adjust `*_voxel_leaf_size` to control the downsampling level of the input scan. Smaller values retain more detail but increase computation.
* **`detection_score_threshold`**: This is crucial for determining what constitutes a "detection." A lower score indicates a better match. You'll need to tune this based on your application's requirements.


### Contributing

Feel free to open issues or submit pull requests if you have suggestions or improvements!
