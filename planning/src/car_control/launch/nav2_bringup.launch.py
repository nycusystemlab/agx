'''
Nav2 導航啟動檔（Base + Override 架構）

參數檔合併流程：
    nav2_params_base.yaml  ← 共用預設
    nav2_params_{real,sim}.yaml  ← 差異 override
    → deep-merge 至 /tmp 暫存 yaml → 傳入 nav2_bringup/bringup_launch.py

預設使用 nav2_params_real.yaml（實體機器人，尚未測試）。
sim 模式請改用 nav2_bringup_sim.launch.py，它會覆寫 params_file。
use_sim_time 由 SetParameter 全域注入，yaml 裡不再寫入。
'''

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def _deep_merge(base, override):
    if isinstance(base, dict) and isinstance(override, dict):
        merged = dict(base)
        for key, value in override.items():
            merged[key] = _deep_merge(merged.get(key), value) if key in merged else value
        return merged
    return override


def _merge_params(base_path, override_path):
    with open(base_path, 'r') as f:
        base = yaml.safe_load(f) or {}
    with open(override_path, 'r') as f:
        override = yaml.safe_load(f) or {}
    merged = _deep_merge(base, override)

    fd, out_path = tempfile.mkstemp(
        prefix='nav2_params_merged_', suffix='.yaml'
    )
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(merged, f, default_flow_style=False, sort_keys=False)
    return out_path


def _launch_setup(context, *args, **kwargs):
    car_control_share = get_package_share_directory('car_control')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    base_path = os.path.join(car_control_share, 'config', 'nav2_params_base.yaml')
    override_path = LaunchConfiguration('params_file').perform(context)
    merged_path = _merge_params(base_path, override_path)

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': merged_path,
        }.items(),
    )

    fox_republisher = Node(
        package='car_control',
        executable='foxglove_pose_republisher.py',
        name='foxglove_pose_republisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return [nav2_launch, fox_republisher]


def generate_launch_description():
    car_control_share = get_package_share_directory('car_control')

    default_map = os.path.join(car_control_share, 'config', 'sim_map.yaml')
    default_override = os.path.join(
        car_control_share, 'config', 'nav2_params_real.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock from /clock (set true for Isaac Sim)',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load',
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_override,
        description='Nav2 override yaml（會與 nav2_params_base.yaml 深度合併）',
    )

    # 全域注入 use_sim_time — 此 Launch 內所有節點皆繼承
    set_sim_time = SetParameter(
        name='use_sim_time', value=LaunchConfiguration('use_sim_time')
    )

    # 縮短 shutdown 超時 — 避免 Nav2 進程卡死
    sigterm_timeout = SetLaunchConfiguration('sigterm_timeout', '5')
    sigkill_timeout = SetLaunchConfiguration('sigkill_timeout', '2')

    return LaunchDescription([
        sigterm_timeout,
        sigkill_timeout,
        declare_use_sim_time,
        declare_map,
        declare_params,
        set_sim_time,
        OpaqueFunction(function=_launch_setup),
    ])
