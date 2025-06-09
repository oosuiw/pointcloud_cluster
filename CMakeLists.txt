cmake_minimum_required(VERSION 3.8)
project(pointcloud_cluster) # 프로젝트 이름이 pointcloud_cluster로 변경된 것을 반영

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree filters features registration) # PCL 모듈 추가

add_executable(pointcloud_clusterer src/pointcloud_clusterer.cpp)

# 헤더 파일을 찾을 경로 추가
# 이 부분이 'No such file or directory' 오류를 해결합니다.
target_include_directories(pointcloud_clusterer PUBLIC
  # 빌드 시 소스 디렉토리 내의 include 폴더를 찾도록 합니다.
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # 설치 후에는 'include' 폴더를 찾도록 합니다 (ROS2 규칙).
  # PCL 관련 헤더 파일들은 이미 find_package(PCL ...)에서 처리됩니다.
  $<INSTALL_INTERFACE:include>
)

# 실행 파일의 종속성 설정
ament_target_dependencies(pointcloud_clusterer rclcpp sensor_msgs visualization_msgs pcl_conversions pcl_msgs PCL)

# 모델 및 설정 파일 설치 규칙 추가
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS2 패키지로 선언합니다. 이 함수는 패키지를 내보내고 설치 규칙을 처리합니다.
ament_package()