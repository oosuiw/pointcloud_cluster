import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('cluster_package')
    
    # 모델 파일 경로 설정 (절대 경로)
    # config/params.yaml 에서는 "models/my_model.ply"와 같이 상대 경로로 설정하고,
    # 여기 런치 파일에서 실제 절대 경로를 오버라이드 해 주는 것이 좋습니다.
    model_file_path = os.path.join(package_dir, 'models', 'my_model.ply') # YOUR_MODEL_NAME.ply 로 변경

    # 파라미터 파일 경로 설정
    params_file_path = os.path.join(package_dir, 'config', 'params.yaml')

    # 런치 아규먼트 선언
    registration_method_arg = DeclareLaunchArgument(
        'registration_method',
        default_value='ICP', 
        description='Registration method: ICP or NDT'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='processed_pointcloud',
        description='ROS2 topic for publishing processed point cloud'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/livox/lidar', # 기본 스캔 토픽 설정 (YAML에서 오버라이드 가능)
        description='ROS2 topic for input scan data'
    )

    return LaunchDescription([
        registration_method_arg,
        output_topic_arg,
        scan_topic_arg, # 새 아규먼트 추가
        Node(
            package='cluster_package',
            executable='pointcloud_clusterer',
            name='pointcloud_clusterer_node',
            output='screen',
            parameters=[
                params_file_path, # YAML 파일 로드
                {'model_path': model_file_path}, # 런치 파일에서 모델 경로 재정의
                {'registration_method': LaunchConfiguration('registration_method')},
                {'output_topic': LaunchConfiguration('output_topic')},
                {'scan_topic': LaunchConfiguration('scan_topic')} # 스캔 토픽 파라미터 전달
            ],
            remappings=[
                # 여기에 필요하다면 추가적인 리매핑을 정의할 수 있습니다.
            ]
        ),
        # Rviz를 함께 띄우고 싶다면 다음 주석을 해제하세요.
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(package_dir, 'rviz', 'default.rviz')], # Rviz 설정 파일 경로
        #     output='screen'
        # )
    ])