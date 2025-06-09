#ifndef POINTCLOUD_PROCESSOR_HPP
#define POINTCLOUD_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <sstream> // std::ostringstream 사용을 위해 추가
#include <iomanip> // std::fixed, std::setprecision 사용을 위해 추가

// Eigen 라이브러리 추가 (변환 행렬 사용을 위해)
#include <Eigen/Dense>

// PCL의 getMinMax3D 사용을 위해 필요한 헤더
#include <pcl/common/common.h> // 이 헤더가 getMinMax3D를 포함합니다.

namespace pointcloud_cluster
{

class PointCloudProcessor : public rclcpp::Node
{
public:
  PointCloudProcessor();

private:
  void declare_and_get_parameters();
  bool load_model_from_ply(const std::string& filepath);
  void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& frame_id, const std_msgs::msg::Header& original_header);
  // publish_bounding_box 함수 선언 변경!
  void publish_bounding_box(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& original_header, double score, bool is_delete = false);


  // ROS2 파라미터
  std::string model_path_;
  std::string registration_method_;
  std::string output_topic_;
  std::string scan_topic_;

  // ICP 파라미터
  int icp_max_iterations_;
  double icp_transformation_epsilon_;
  double icp_euclidean_fitness_epsilon_;
  double icp_max_correspondence_distance_;
  double icp_voxel_leaf_size_;

  // NDT 파라미터
  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_transformation_epsilon_;
  int ndt_max_iterations_;
  double ndt_voxel_leaf_size_;

  // 바운딩 박스 크기
  double bbox_length_;
  double bbox_width_;
  double bbox_height_;
  double detection_score_threshold_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bounding_box_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_;

  // 바운딩 박스의 초기 오프셋을 저장할 수 있는 변수
  Eigen::Vector3f model_min_pt_; // <--- 이 부분은 그대로 사용
  Eigen::Vector3f model_max_pt_; // <--- 이 부분은 그대로 사용

};

} // namespace pointcloud_cluster

#endif // POINTCLOUD_PROCESSOR_HPP