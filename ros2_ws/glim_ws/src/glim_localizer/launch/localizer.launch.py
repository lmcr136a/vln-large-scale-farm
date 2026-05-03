from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        "map_path", default_value="",
        description="Path to GLIM saved map directory")

    map_path = LaunchConfiguration("map_path")

    # map_loader_node: saved map → /map_pointcloud (RViz 시각화용)
    map_loader = Node(
        package="glim_localizer",
        executable="map_loader_node",
        name="map_loader_node",
        parameters=[{
            "map_path": map_path,
            "points_topic": "/map_pointcloud",
            "map_frame": "map",
        }],
        output="screen",
    )

    # localizer_ext + init_pose_viewer 는 GLIM extension module로 로드됨
    # → config_ros.json extension_modules에 추가
    # → ros2 run glim_ros glim_rosnode 실행 시
    #     -p localizer_ext.map_path:=<map_path> 파라미터 전달

    return LaunchDescription([
        map_path_arg,
        map_loader,
    ])