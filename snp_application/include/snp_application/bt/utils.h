#pragma once

#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/node.hpp>

namespace snp_application
{
/** @brief BT blackboard key for recording error messages from BT nodes */
inline static const std::string ERROR_MESSAGE_KEY = "error_message";

template <typename T>
T getBTInput(const BT::TreeNode* node, const std::string& port)
{
  BT::Expected<T> input = node->getInput<T>(port);
  if (!input)
    throw BT::RuntimeError("Failed to get required input value: '" + input.error() + "'");

  return input.value();
}

// Parameters
static const std::string MOTION_GROUP_PARAM = "motion_group";
static const std::string FREESPACE_MOTION_GROUP_PARAM = "freespace_motion_group";
static const std::string REF_FRAME_PARAM = "reference_frame";
static const std::string TCP_FRAME_PARAM = "tcp_frame";
static const std::string CAMERA_FRAME_PARAM = "camera_frame";
static const std::string MESH_FILE_PARAM = "mesh_file";
static const std::string START_STATE_REPLACEMENT_TOLERANCE_PARAM = "start_state_replacement_tolerance";
// Home state
static const std::string HOME_STATE_JOINT_VALUES_PARAM = "home_state_joint_values";
static const std::string HOME_STATE_JOINT_NAMES_PARAM = "home_state_joint_names";
//   Industrial Reconstruction
static const std::string IR_TSDF_VOXEL_PARAM = "ir.tsdf.voxel_length";
static const std::string IR_TSDF_SDF_PARAM = "ir.tsdf.sdf_trunc";
static const std::string IR_TSDF_MIN_X_PARAM = "ir.tsdf.min.x";
static const std::string IR_TSDF_MIN_Y_PARAM = "ir.tsdf.min.y";
static const std::string IR_TSDF_MIN_Z_PARAM = "ir.tsdf.min.z";
static const std::string IR_TSDF_MAX_X_PARAM = "ir.tsdf.max.x";
static const std::string IR_TSDF_MAX_Y_PARAM = "ir.tsdf.max.y";
static const std::string IR_TSDF_MAX_Z_PARAM = "ir.tsdf.max.z";
static const std::string IR_RGBD_DEPTH_SCALE_PARAM = "ir.rgbd.depth_scale";
static const std::string IR_RGBD_DEPTH_TRUNC_PARAM = "ir.rgbd.depth_trunc";
static const std::string IR_LIVE_PARAM = "ir.live";
static const std::string IR_MIN_FACES_PARAM = "ir.min_faces";
static const std::string IR_NORMAL_ANGLE_TOL_PARAM = "ir.normal_angle_tol";
static const std::string IR_NORMAL_X_PARAM = "ir.normal_x";
static const std::string IR_NORMAL_Y_PARAM = "ir.normal_y";
static const std::string IR_NORMAL_Z_PARAM = "ir.normal_z";
static const std::string IR_ARCHIVE_DIR_PARAM = "ir.archive_dir";

// Enhanced reconstruction parameters
static const std::string IR_TRANSLATION_DISTANCE_PARAM = "ir.translation_distance";
static const std::string IR_ROTATIONAL_DISTANCE_PARAM = "ir.rotational_distance";
static const std::string IR_RGBD_CONVERT_INTENSITY_PARAM = "ir.rgbd.convert_rgb_to_intensity";

// Enhanced processing parameters
static const std::string IR_ENABLE_DEPTH_PREPROCESSING_PARAM = "ir.enhanced.enable_depth_preprocessing";
static const std::string IR_ENABLE_MESH_POSTPROCESSING_PARAM = "ir.enhanced.enable_mesh_postprocessing";
static const std::string IR_ENABLE_ADAPTIVE_PARAMS_PARAM = "ir.enhanced.enable_adaptive_params";
static const std::string IR_QUALITY_MODE_PARAM = "ir.enhanced.quality_mode";
static const std::string IR_SCENE_TYPE_PARAM = "ir.enhanced.scene_type";

// Depth preprocessing parameters
static const std::string IR_BILATERAL_D_PARAM = "ir.enhanced.bilateral_d";
static const std::string IR_BILATERAL_SIGMA_COLOR_PARAM = "ir.enhanced.bilateral_sigma_color";
static const std::string IR_BILATERAL_SIGMA_SPACE_PARAM = "ir.enhanced.bilateral_sigma_space";
static const std::string IR_MEDIAN_KERNEL_SIZE_PARAM = "ir.enhanced.median_kernel_size";
static const std::string IR_MORPHOLOGICAL_KERNEL_SIZE_PARAM = "ir.enhanced.morphological_kernel_size";
static const std::string IR_TEMPORAL_ALPHA_PARAM = "ir.enhanced.temporal_alpha";
static const std::string IR_OUTLIER_STD_THRESHOLD_PARAM = "ir.enhanced.outlier_std_threshold";
static const std::string IR_MAX_HOLE_SIZE_PARAM = "ir.enhanced.max_hole_size";

// Mesh post-processing parameters
static const std::string IR_SMOOTHING_ITERATIONS_PARAM = "ir.enhanced.smoothing_iterations";
static const std::string IR_SMOOTHING_LAMBDA_PARAM = "ir.enhanced.smoothing_lambda";
static const std::string IR_DECIMATION_RATIO_PARAM = "ir.enhanced.decimation_ratio";
static const std::string IR_TARGET_EDGE_LENGTH_PARAM = "ir.enhanced.target_edge_length";
static const std::string IR_HOLE_FILLING_DIAMETER_PARAM = "ir.enhanced.hole_filling_diameter";
static const std::string IR_CURVATURE_THRESHOLD_PARAM = "ir.enhanced.curvature_threshold";
static const std::string IR_QUALITY_THRESHOLD_PARAM = "ir.enhanced.quality_threshold";

// Filtering options
static const std::string IR_ENABLE_BILATERAL_SMOOTHING_PARAM = "ir.enhanced.enable_bilateral_smoothing";
static const std::string IR_ENABLE_CURVATURE_SMOOTHING_PARAM = "ir.enhanced.enable_curvature_smoothing";
static const std::string IR_ENABLE_LAPLACIAN_SMOOTHING_PARAM = "ir.enhanced.enable_laplacian_smoothing";
static const std::string IR_ENABLE_TOPOLOGY_OPTIMIZATION_PARAM = "ir.enhanced.enable_topology_optimization";
static const std::string IR_ENABLE_HOLE_FILLING_PARAM = "ir.enhanced.enable_hole_filling";
static const std::string IR_ENABLE_OUTLIER_REMOVAL_PARAM = "ir.enhanced.enable_outlier_removal";

template <typename T>
T get_parameter(rclcpp::Node::SharedPtr node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

template <typename T>
T get_parameter_or(rclcpp::Node::SharedPtr node, const std::string& key, const T& default_val)
{
  T val;
  node->get_parameter_or(key, val, default_val);
  return val;
}

template <typename T>
T get_parameter_or(std::weak_ptr<rclcpp::Node> node, const std::string& key, const T& default_val)
{
  auto node_ptr = node.lock();
  if (!node_ptr)
    throw std::runtime_error("Node is no longer valid");
  return get_parameter_or(node_ptr, key, default_val);
}

template <typename T>
T get_parameter(std::weak_ptr<rclcpp::Node> node, const std::string& key)
{
  auto node_ptr = node.lock();
  if (!node_ptr)
    throw std::runtime_error("Node is no longer valid");
  return get_parameter<T>(node_ptr, key);
}

}  // namespace snp_application
