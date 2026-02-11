from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def _extract_camera_blocks(data: dict):
    """
    Returns a list of (block_key, ros__parameters_dict).
    For multi-camera dcw2.yaml like:
      dcw2_1: {ros__parameters: {...}}
      dcw2_2: {ros__parameters: {...}}
    """
    blocks = []
    if not isinstance(data, dict):
        return blocks
    for k, v in data.items():
        if isinstance(v, dict) and isinstance(v.get("ros__parameters"), dict):
            blocks.append((str(k), dict(v["ros__parameters"])))
    return blocks


def _launch_setup(context, *args, **kwargs):
    config_file_path = LaunchConfiguration("config_file_path").perform(context)
    with open(config_file_path, "r") as f:
        data = yaml.safe_load(f) or {}

    blocks = _extract_camera_blocks(data)
    # Backward compatible single-camera formats:
    # 1) {/**: {ros__parameters: {...}}}
    # 2) {ros__parameters: {...}}
    if not blocks:
        if isinstance(data, dict) and isinstance(data.get("/**"), dict) and isinstance(
            data["/**"].get("ros__parameters"), dict
        ):
            blocks = [("/**", dict(data["/**"]["ros__parameters"]))]
        elif isinstance(data, dict) and isinstance(data.get("ros__parameters"), dict):
            blocks = [("ros__parameters", dict(data["ros__parameters"]))]
        else:
            raise RuntimeError(
                f"No camera blocks found in {config_file_path}. "
                "Expected top-level keys like dcw2_1/dcw2_2 with ros__parameters."
            )

    camera_count = len(blocks)

    # If user passes device_num explicitly, keep it; otherwise default to camera_count.
    device_num_str = LaunchConfiguration("device_num").perform(context)
    try:
        device_num_val = int(device_num_str)
    except Exception:
        device_num_val = camera_count
    if device_num_val <= 0:
        device_num_val = camera_count

    composable_nodes = []
    for block_key, params in blocks:
        # Namespace / node name: prefer camera_name in yaml, fallback to block key.
        cam_ns = str(params.get("camera_name") or block_key)
        params["device_num"] = device_num_val

        composable_nodes.append(
            ComposableNode(
                package="orbbec_camera",
                plugin="orbbec_camera::OBCameraNodeDriver",
                name=cam_ns,
                namespace=cam_ns,
                parameters=[params],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return [container]


def generate_launch_description():
    default_config_file = os.path.join("/data", "params", "1dcw2.yaml")

    args = [
        # YAML parameter file. Loaded first, then overridden by launch arguments below.
        DeclareLaunchArgument(
            "config_file_path",
            default_value=default_config_file,
        ),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('depth_registration', default_value='false'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('vendor_id', default_value='0x2bc5'),
        DeclareLaunchArgument('product_id', default_value=''),
        DeclareLaunchArgument('enable_point_cloud', default_value='true'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument('cloud_frame_id', default_value=''),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),
        DeclareLaunchArgument('connection_delay', default_value='100'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('color_fps', default_value='10'),
        DeclareLaunchArgument('color_format', default_value='MJPG'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('flip_color', default_value='false'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('enable_color_auto_exposure_priority', default_value='false'),
        DeclareLaunchArgument('color_exposure', default_value='-1'),
        DeclareLaunchArgument('color_gain', default_value='-1'),
        DeclareLaunchArgument('enable_color_auto_white_balance', default_value='true'),
        DeclareLaunchArgument('color_white_balance', default_value='-1'),
        DeclareLaunchArgument('color_ae_max_exposure', default_value='-1'),
        DeclareLaunchArgument('color_brightness', default_value='-1'),
        DeclareLaunchArgument('color_sharpness', default_value='-1'),
        DeclareLaunchArgument('color_saturation', default_value='-1'),
        DeclareLaunchArgument('color_contrast', default_value='-1'),
        DeclareLaunchArgument('color_gamma', default_value='-1'),
        DeclareLaunchArgument('color_hue', default_value='-1'),
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='400'),
        DeclareLaunchArgument('depth_fps', default_value='10'),
        DeclareLaunchArgument('depth_format', default_value='Y11'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('flip_depth', default_value='false'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),
        # /config/depthfilter/Openni_device.json，need config path.
        DeclareLaunchArgument('depth_filter_config', default_value=''),
        DeclareLaunchArgument('ir_width', default_value='640'),
        DeclareLaunchArgument('ir_height', default_value='400'),
        DeclareLaunchArgument('ir_fps', default_value='10'),
        DeclareLaunchArgument('ir_format', default_value='Y10'),
        # NOTE: Enable IR stream only if you really need it, to reduce CPU/bandwidth by default.
        DeclareLaunchArgument('enable_ir', default_value='false'),
        DeclareLaunchArgument('flip_ir', default_value='false'),
        DeclareLaunchArgument('ir_qos', default_value='default'),
        DeclareLaunchArgument('ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
        DeclareLaunchArgument('ir_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_gain', default_value='-1'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        # NOTE: Software depth filter may increase host CPU usage.
        DeclareLaunchArgument('enable_soft_filter', default_value='false'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        # NOTE: Depth post-process filters can cost CPU; keep noise removal off by default.
        DeclareLaunchArgument('enable_noise_removal_filter', default_value='false'),
        DeclareLaunchArgument('soft_filter_max_diff', default_value='-1'),
        DeclareLaunchArgument('soft_filter_speckle_size', default_value='-1'),
        DeclareLaunchArgument('ordered_pc', default_value='false'),
        # Subsample point cloud by taking 1 point every N points during conversion (unordered point cloud).
        # 1 means no subsampling.
        DeclareLaunchArgument('point_cloud_stride', default_value='1'),
        DeclareLaunchArgument('use_hardware_time', default_value='false'),
        DeclareLaunchArgument('enable_depth_scale', default_value='true'),
        DeclareLaunchArgument('align_mode', default_value='HW'),
        DeclareLaunchArgument('laser_energy_level', default_value='-1'),
        DeclareLaunchArgument('enable_heartbeat', default_value='false'),
        DeclareLaunchArgument('industry_mode', default_value=''),
    ]

    # Multi-camera boot-up count is derived from the YAML file blocks (dcw2_1, dcw2_2, ...).
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
