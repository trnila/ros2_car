load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_library",
)
load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "cpp_ros2_interface_library",
    "ros2_interface_library",
)

DEPS = [
    "@ros2_common_interfaces//:cpp_geometry_msgs",
    "@ros2_common_interfaces//:c_geometry_msgs",
    "@ros2_common_interfaces//:cpp_std_srvs",
    "@ros2_rclcpp//:rclcpp",
    "@message_filters",
    "@ros2_pluginlib//:pluginlib",
    "@ros2_geometry2//:tf2_ros",
    "@ros2_geometry2//:cpp_tf2_msgs",
    "@ros2_geometry2//:cpp_tf2_geometry_msgs",
    "@ros2_common_interfaces//:cpp_nav_msgs",
    "@ros2_rclcpp//:rclcpp_lifecycle",
    "@bond_core//:bondcpp",
]

ros2_interface_library(
    name = "nav2_msgs",
    srcs = glob(["nav2_msgs/msg/*.msg"]),
    visibility = ["//visibility:public"],
)

cpp_ros2_interface_library(
    name = "cpp_nav2_msgs",
    visibility = ["//visibility:public"],
    deps = [":nav2_msgs"],
)

ros2_cpp_library(
    name = "nav2_amcl",
    srcs = glob(
        [
            "nav2_amcl/**/*.hpp",
            "nav2_amcl/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_amcl/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_amcl/include"],
    deps = DEPS + [
        ":cpp_nav2_msgs",
        ":nav2_util",
    ],
)

ros2_cpp_library(
    name = "nav2_behaviors",
    srcs = glob(
        [
            "nav2_behaviors/**/*.hpp",
            "nav2_behaviors/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_behaviors/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_behaviors/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_behavior_tree",
    srcs = glob(
        [
            "nav2_behavior_tree/**/*.hpp",
            "nav2_behavior_tree/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_behavior_tree/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_behavior_tree/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_bringup",
    srcs = glob(
        [
            "nav2_bringup/**/*.hpp",
            "nav2_bringup/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_bringup/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_bringup/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_bt_navigator",
    srcs = glob(
        [
            "nav2_bt_navigator/**/*.hpp",
            "nav2_bt_navigator/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_bt_navigator/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_bt_navigator/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_collision_monitor",
    srcs = glob(
        [
            "nav2_collision_monitor/**/*.hpp",
            "nav2_collision_monitor/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_collision_monitor/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_collision_monitor/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_common",
    srcs = glob(
        [
            "nav2_common/**/*.hpp",
            "nav2_common/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_common/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_common/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_constrained_smoother",
    srcs = glob(
        [
            "nav2_constrained_smoother/**/*.hpp",
            "nav2_constrained_smoother/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_constrained_smoother/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_constrained_smoother/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_controller",
    srcs = glob(
        [
            "nav2_controller/**/*.hpp",
            "nav2_controller/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_controller/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_controller/include"],
    deps = DEPS + [
        ":nav2_core",
        ":nav2_costmap_2d",
    ],
)

ros2_cpp_library(
    name = "nav2_core",
    srcs = glob(
        [
            "nav2_core/**/*.hpp",
            "nav2_core/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_core/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_core/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_costmap_2d",
    srcs = glob(
        [
            "nav2_costmap_2d/**/*.hpp",
            "nav2_costmap_2d/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_costmap_2d/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_costmap_2d/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_docking",
    srcs = glob(
        [
            "nav2_docking/**/*.hpp",
            "nav2_docking/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_docking/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_docking/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_dwb_controller",
    srcs = glob(
        [
            "nav2_dwb_controller/**/*.hpp",
            "nav2_dwb_controller/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_dwb_controller/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_dwb_controller/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_graceful_controller",
    srcs = glob(
        [
            "nav2_graceful_controller/**/*.hpp",
            "nav2_graceful_controller/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_graceful_controller/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_graceful_controller/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_lifecycle_manager",
    srcs = glob(
        [
            "nav2_lifecycle_manager/**/*.hpp",
            "nav2_lifecycle_manager/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_lifecycle_manager/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_lifecycle_manager/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_loopback_sim",
    srcs = glob(
        [
            "nav2_loopback_sim/**/*.hpp",
            "nav2_loopback_sim/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_loopback_sim/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_loopback_sim/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_map_server",
    srcs = glob(
        [
            "nav2_map_server/**/*.hpp",
            "nav2_map_server/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_map_server/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_map_server/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_mppi_controller",
    srcs = glob(
        [
            "nav2_mppi_controller/**/*.hpp",
            "nav2_mppi_controller/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_mppi_controller/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_mppi_controller/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_navfn_planner",
    srcs = glob(
        [
            "nav2_navfn_planner/**/*.hpp",
            "nav2_navfn_planner/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_navfn_planner/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_navfn_planner/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_planner",
    srcs = glob(
        [
            "nav2_planner/**/*.hpp",
            "nav2_planner/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_planner/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_planner/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_regulated_pure_pursuit_controller",
    srcs = glob(
        [
            "nav2_regulated_pure_pursuit_controller/**/*.hpp",
            "nav2_regulated_pure_pursuit_controller/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_regulated_pure_pursuit_controller/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_regulated_pure_pursuit_controller/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_ros_common",
    srcs = glob(
        [
            "nav2_ros_common/**/*.hpp",
            "nav2_ros_common/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_ros_common/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_ros_common/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_rotation_shim_controller",
    srcs = glob(
        [
            "nav2_rotation_shim_controller/**/*.hpp",
            "nav2_rotation_shim_controller/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_rotation_shim_controller/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_rotation_shim_controller/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_route",
    srcs = glob(
        [
            "nav2_route/**/*.hpp",
            "nav2_route/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_route/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_route/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_rviz_plugins",
    srcs = glob(
        [
            "nav2_rviz_plugins/**/*.hpp",
            "nav2_rviz_plugins/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_rviz_plugins/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_rviz_plugins/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_simple_commander",
    srcs = glob(
        [
            "nav2_simple_commander/**/*.hpp",
            "nav2_simple_commander/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_simple_commander/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_simple_commander/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_smac_planner",
    srcs = glob(
        [
            "nav2_smac_planner/**/*.hpp",
            "nav2_smac_planner/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_smac_planner/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_smac_planner/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_smoother",
    srcs = glob(
        [
            "nav2_smoother/**/*.hpp",
            "nav2_smoother/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_smoother/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_smoother/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_system_tests",
    srcs = glob(
        [
            "nav2_system_tests/**/*.hpp",
            "nav2_system_tests/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_system_tests/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_system_tests/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_theta_star_planner",
    srcs = glob(
        [
            "nav2_theta_star_planner/**/*.hpp",
            "nav2_theta_star_planner/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_theta_star_planner/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_theta_star_planner/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_util",
    srcs = glob(
        [
            "nav2_util/**/*.hpp",
            "nav2_util/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_util/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_util/include"],
    deps = DEPS + [
        ":cpp_nav2_msgs",
    ],
)

ros2_cpp_library(
    name = "nav2_velocity_smoother",
    srcs = glob(
        [
            "nav2_velocity_smoother/**/*.hpp",
            "nav2_velocity_smoother/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_velocity_smoother/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_velocity_smoother/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_voxel_grid",
    srcs = glob(
        [
            "nav2_voxel_grid/**/*.hpp",
            "nav2_voxel_grid/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_voxel_grid/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_voxel_grid/include"],
    deps = DEPS,
)

ros2_cpp_library(
    name = "nav2_waypoint_follower",
    srcs = glob(
        [
            "nav2_waypoint_follower/**/*.hpp",
            "nav2_waypoint_follower/**/*.cpp",
        ],
        allow_empty = True,
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "nav2_waypoint_follower/**/*.hpp",
        ],
        allow_empty = True,
    ),
    includes = ["nav2_waypoint_follower/include"],
    deps = DEPS,
)
