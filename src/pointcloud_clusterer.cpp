#include "pointcloud_cluster/pointcloud_processor.hpp"

namespace pointcloud_cluster
{

PointCloudProcessor::PointCloudProcessor() : Node("pointcloud_clusterer")
{
  declare_and_get_parameters();

  processed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
  bounding_box_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("detected_object_bbox", 10); // 바운딩 박스 및 텍스트 토픽

  model_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (!load_model_from_ply(model_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load model from %s", model_path_.c_str());
    rclcpp::shutdown(); // 모델 로드 실패 시 노드 종료
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Model loaded successfully from %s with %zu points.", model_path_.c_str(), model_cloud_->points.size());

  // 모델의 AABB (Axis Aligned Bounding Box) 계산
  // PCL 1.12+ 에서는 Eigen::Vector4f를 인자로 받는 getMinMax3D가 일반적입니다.
  // Eigen::Vector3f를 사용하려면 이전에 Eigen::Vector4f로 계산한 뒤 3D 부분만 추출하거나,
  // PCL::getMinMax3D(PointCloud, min_pt, max_pt) 오버로드가 있는 PCL 버전을 사용해야 합니다.
  // 여기서는 Eigen::Vector4f로 계산하여 bbox_length_ 등을 계산하도록 수정합니다.
  Eigen::Vector4f min_pt_4f, max_pt_4f;
  pcl::getMinMax3D(*model_cloud_, min_pt_4f, max_pt_4f);
  model_min_pt_ = Eigen::Vector3f(min_pt_4f.x(), min_pt_4f.y(), min_pt_4f.z());
  model_max_pt_ = Eigen::Vector3f(max_pt_4f.x(), max_pt_4f.y(), max_pt_4f.z());

  bbox_length_ = model_max_pt_.x() - model_min_pt_.x();
  bbox_width_ = model_max_pt_.y() - model_min_pt_.y();
  bbox_height_ = model_max_pt_.z() - model_min_pt_.z();

  RCLCPP_INFO(this->get_logger(), "Calculated model bounding box dimensions: L=%.2f, W=%.2f, H=%.2f",
              bbox_length_, bbox_width_, bbox_height_);

  // 스캔 데이터 구독 시작
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, 10, std::bind(&PointCloudProcessor::scan_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribing to scan data on topic: %s", scan_topic_.c_str());
}

void PointCloudProcessor::declare_and_get_parameters()
{
  this->declare_parameter<std::string>("model_path", "share/pointcloud_cluster/models/my_model.ply");
  this->get_parameter("model_path", model_path_);

  this->declare_parameter<std::string>("registration_method", "ICP");
  this->get_parameter("registration_method", registration_method_);

  this->declare_parameter<std::string>("output_topic", "processed_pointcloud");
  this->get_parameter("output_topic", output_topic_);

  this->declare_parameter<std::string>("scan_topic", "/camera/depth/color/points"); // 기본 스캔 토픽 (예: RealSense)
  this->get_parameter("scan_topic", scan_topic_);

  // ICP 파라미터
  this->declare_parameter<int>("icp_max_iterations", 100);
  this->get_parameter("icp_max_iterations", icp_max_iterations_);
  this->declare_parameter<double>("icp_transformation_epsilon", 1e-8);
  this->get_parameter("icp_transformation_epsilon", icp_transformation_epsilon_);
  this->declare_parameter<double>("icp_euclidean_fitness_epsilon", 1e-4);
  this->get_parameter("icp_euclidean_fitness_epsilon", icp_euclidean_fitness_epsilon_);
  this->declare_parameter<double>("icp_max_correspondence_distance", 0.1);
  this->get_parameter("icp_max_correspondence_distance", icp_max_correspondence_distance_);
  this->declare_parameter<double>("icp_voxel_leaf_size", 0.01); // 1cm 다운샘플링
  this->get_parameter("icp_voxel_leaf_size", icp_voxel_leaf_size_);

  // NDT 파라미터
  this->declare_parameter<double>("ndt_resolution", 1.0);
  this->get_parameter("ndt_resolution", ndt_resolution_);
  this->declare_parameter<double>("ndt_step_size", 0.1);
  this->get_parameter("ndt_step_size", ndt_step_size_);
  this->declare_parameter<double>("ndt_transformation_epsilon", 0.01);
  this->get_parameter("ndt_transformation_epsilon", ndt_transformation_epsilon_);
  this->declare_parameter<int>("ndt_max_iterations", 30);
  this->get_parameter("ndt_max_iterations", ndt_max_iterations_);
  this->declare_parameter<double>("ndt_voxel_leaf_size", 0.05); // 5cm 다운샘플링 (NDT는 좀 더 큰 리솔루션 사용)
  this->get_parameter("ndt_voxel_leaf_size", ndt_voxel_leaf_size_);

  // 객체 검출 스코어 임계값
  this->declare_parameter<double>("detection_score_threshold", 0.05); // 정합 스코어 임계값
  this->get_parameter("detection_score_threshold", detection_score_threshold_);
}

bool PointCloudProcessor::load_model_from_ply(const std::string& filepath)
{
  std::string full_path;
  const char* install_prefix_env = std::getenv("COLCON_CURRENT_PREFIX");
  if (install_prefix_env) {
      full_path = std::string(install_prefix_env) + "/share/pointcloud_cluster/" + filepath;
  } else {
      RCLCPP_WARN(this->get_logger(), "COLCON_CURRENT_PREFIX not found, trying local path. For installed package, please ensure env is sourced correctly.");
      full_path = filepath;
  }

  RCLCPP_INFO(this->get_logger(), "Attempting to load model from: %s", full_path.c_str());

  if (pcl::io::loadPLYFile<pcl::PointXYZ>(full_path, *model_cloud_) == -1) {
    return false;
  }
  return true;
}

void PointCloudProcessor::scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received scan data. Processing...");

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *scan_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(scan_cloud);
  
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
  double fitness_score = -1.0;

  if (registration_method_ == "ICP") {
    sor.setLeafSize(static_cast<float>(icp_voxel_leaf_size_), static_cast<float>(icp_voxel_leaf_size_), static_cast<float>(icp_voxel_leaf_size_));
    sor.filter(*filtered_scan_cloud);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Using ICP with %zu points after voxel filtering.", filtered_scan_cloud->points.size());

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(icp_max_iterations_); // setMaxIterations -> setMaximumIterations
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);

    icp.setInputSource(filtered_scan_cloud);
    icp.setInputTarget(model_cloud_);

    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    icp.align(final_cloud);

    if (icp.hasConverged()) {
      fitness_score = icp.getFitnessScore();
      RCLCPP_INFO(this->get_logger(), "ICP converged. Fitness score: %f", fitness_score);
      transformation_matrix = icp.getFinalTransformation();
    } else {
      RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
    }
  } else if (registration_method_ == "NDT") {
    sor.setLeafSize(static_cast<float>(ndt_voxel_leaf_size_), static_cast<float>(ndt_voxel_leaf_size_), static_cast<float>(ndt_voxel_leaf_size_));
    sor.filter(*filtered_scan_cloud);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Using NDT with %zu points after voxel filtering.", filtered_scan_cloud->points.size());

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(static_cast<float>(ndt_resolution_));
    ndt.setStepSize(static_cast<float>(ndt_step_size_));
    ndt.setTransformationEpsilon(static_cast<float>(ndt_transformation_epsilon_));
    ndt.setMaximumIterations(ndt_max_iterations_);

    ndt.setInputSource(filtered_scan_cloud);
    ndt.setInputTarget(model_cloud_);

    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    ndt.align(final_cloud);

    if (ndt.hasConverged()) {
      fitness_score = ndt.getFitnessScore();
      RCLCPP_INFO(this->get_logger(), "NDT converged. Fitness score: %f", fitness_score);
      transformation_matrix = ndt.getFinalTransformation();
    } else {
      RCLCPP_WARN(this->get_logger(), "NDT did not converge.");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown registration method: %s. No processing done.", registration_method_.c_str());
  }
  
  // 스코어 기반 객체 검출 판단 및 시각화
  if (fitness_score != -1.0 && fitness_score < detection_score_threshold_) {
    RCLCPP_INFO(this->get_logger(), "Object detected! Fitness score %.4f is below threshold %.4f.", fitness_score, detection_score_threshold_);
    publish_bounding_box(transformation_matrix, msg->header, fitness_score);
    publish_pointcloud(filtered_scan_cloud, msg->header.frame_id, msg->header);
  } else if (fitness_score != -1.0) {
    RCLCPP_INFO(this->get_logger(), "No object detected (score %.4f >= threshold %.4f).", fitness_score, detection_score_threshold_);
    publish_bounding_box(Eigen::Matrix4f::Identity(), msg->header, -1.0, true);
  } else {
    RCLCPP_WARN(this->get_logger(), "Registration failed, no fitness score available.");
    publish_bounding_box(Eigen::Matrix4f::Identity(), msg->header, -1.0, true);
  }
}

void PointCloudProcessor::publish_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& frame_id, const std_msgs::msg::Header& original_header)
{
  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.stamp = original_header.stamp;
  ros_cloud.header.frame_id = frame_id;
  processed_cloud_publisher_->publish(ros_cloud);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Published processed point cloud with %zu points to topic '%s'", cloud->points.size(), output_topic_.c_str());
}

// publish_bounding_box 함수 정의 변경!
void PointCloudProcessor::publish_bounding_box(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& original_header, double score, bool is_delete)
{
  visualization_msgs::msg::Marker bbox_marker;
  bbox_marker.header.frame_id = original_header.frame_id;
  bbox_marker.header.stamp = original_header.stamp;
  bbox_marker.ns = "detected_objects";
  bbox_marker.id = 0; // 바운딩 박스 ID
  bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
  bbox_marker.action = is_delete ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;

  // 바운딩 박스 자세
  Eigen::Vector3f model_center_local = (model_min_pt_ + model_max_pt_) / 2.0f;
  Eigen::Vector4f transformed_center_4f = transform * Eigen::Vector4f(model_center_local.x(), model_center_local.y(), model_center_local.z(), 1.0f);
  bbox_marker.pose.position.x = transformed_center_4f.x();
  bbox_marker.pose.position.y = transformed_center_4f.y();
  bbox_marker.pose.position.z = transformed_center_4f.z();

  Eigen::Matrix3f rotation_matrix = transform.block<3,3>(0,0);
  Eigen::Quaternionf q(rotation_matrix);
  bbox_marker.pose.orientation.x = q.x();
  bbox_marker.pose.orientation.y = q.y();
  bbox_marker.pose.orientation.z = q.z();
  bbox_marker.pose.orientation.w = q.w();

  // 바운딩 박스 크기
  bbox_marker.scale.x = bbox_length_;
  bbox_marker.scale.y = bbox_width_;
  bbox_marker.scale.z = bbox_height_;

  // 바운딩 박스 색상 (녹색, 투명도 50%)
  bbox_marker.color.r = 0.0f;
  bbox_marker.color.g = 1.0f;
  bbox_marker.color.b = 0.0f;
  bbox_marker.color.a = 0.5f;
  bbox_marker.lifetime = rclcpp::Duration(0, 0); // 마커를 계속 유지

  bounding_box_publisher_->publish(bbox_marker);

  // 텍스트 마커 발행
  if (!is_delete) {
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = original_header.frame_id;
    text_marker.header.stamp = original_header.stamp;
    text_marker.ns = "detected_object_scores";
    text_marker.id = 1; // 텍스트 마커는 다른 ID 사용
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // 항상 카메라를 바라보도록
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    // 텍스트 위치 (바운딩 박스 상단에 표시)
    text_marker.pose.position.x = transformed_center_4f.x();
    text_marker.pose.position.y = transformed_center_4f.y();
    text_marker.pose.position.z = transformed_center_4f.z() + bbox_height_ / 2.0 + 0.1; // 바운딩 박스 위 10cm

    text_marker.pose.orientation.w = 1.0; // 텍스트는 회전 없음 (TEXT_VIEW_FACING이므로 중요하지 않음)

    // 텍스트 내용 (스코어)
    std::ostringstream ss;
    ss << "Score: " << std::fixed << std::setprecision(4) << score;
    text_marker.text = ss.str();

    // 텍스트 크기
    text_marker.scale.z = 0.2; // 텍스트 높이 (m)

    // 텍스트 색상 (흰색)
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f; // 불투명

    text_marker.lifetime = rclcpp::Duration(0, 0); // 마커를 계속 유지

    bounding_box_publisher_->publish(text_marker);
  } else {
      // DELETE 액션일 경우, 텍스트 마커도 DELETE 메시지를 발행하여 지워줍니다.
      visualization_msgs::msg::Marker text_marker_delete;
      text_marker_delete.header.frame_id = original_header.frame_id;
      text_marker_delete.header.stamp = original_header.stamp;
      text_marker_delete.ns = "detected_object_scores";
      text_marker_delete.id = 1; // 텍스트 마커의 ID와 일치
      text_marker_delete.action = visualization_msgs::msg::Marker::DELETE;
      bounding_box_publisher_->publish(text_marker_delete);
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Published bounding box and score text for detected object.");
}

} // namespace pointcloud_cluster

// 메인 함수: ROS2 노드 실행
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_cluster::PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}