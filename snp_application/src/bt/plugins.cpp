#include <snp_application/bt/button_approval_node.h>
#include <snp_application/bt/button_monitor_node.h>
#include <snp_application/bt/progress_decorator_node.h>
#include <snp_application/bt/set_page_decorator_node.h>
#include <snp_application/bt/snp_bt_ros_nodes.h>
#include <snp_application/bt/snp_sequence_with_memory_node.h>
#include <snp_application/bt/utils.h>

#include <behaviortree_cpp/bt_factory.h>

template <typename T>
void try_declare_parameter(std::weak_ptr<rclcpp::Node> node, const std::string& key, const T& value)
{
  auto node_ptr = node.lock();
  if (!node_ptr)
    throw std::runtime_error("Node is no longer valid");
  if (!node_ptr->has_parameter(key))
    node_ptr->declare_parameter(key, value);
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<snp_application::ButtonApprovalNode>("ButtonApproval");
  factory.registerNodeType<snp_application::ButtonMonitorNode>("ButtonMonitor");
  factory.registerNodeType<snp_application::ProgressDecoratorNode>("Progress");
  factory.registerNodeType<snp_application::SetPageDecoratorNode>("SetPage");
  factory.registerNodeType<snp_application::SNPSequenceWithMemory>("SNPSequenceWithMemory");
  factory.registerNodeType<snp_application::ReverseTrajectoryNode>("ReverseTrajectory");
  factory.registerNodeType<snp_application::CombineTrajectoriesNode>("CombineTrajectories");
}

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  using namespace snp_application;

  factory.registerNodeType<snp_application::RosSpinnerNode>("RosSpinner", params);
  factory.registerNodeType<snp_application::TriggerServiceNode>("TriggerService", params);
  factory.registerNodeType<snp_application::EmptyServiceNode>("EmptyService", params);
  factory.registerNodeType<snp_application::ExecuteMotionPlanServiceNode>("ExecuteMotionPlanService", params);
  factory.registerNodeType<snp_application::ToolPathsPubNode>("ToolPathsPub", params);
  factory.registerNodeType<snp_application::MotionPlanPubNode>("MotionPlanPub", params);
  factory.registerNodeType<snp_application::FollowJointTrajectoryActionNode>("FollowJointTrajectoryAction", params);
  factory.registerNodeType<snp_application::GetCurrentJointStateNode>("GetCurrentJointState", params);
  factory.registerNodeType<snp_application::GenerateScanMotionPlanServiceNode>("GenerateScanMotionPlanService", params);
  factory.registerNodeType<snp_application::GenerateToolPathsServiceNode>("GenerateToolPathsService", params);

  // Nodes requiring parameters
  // Update trajectory start state
  try_declare_parameter<double>(params.nh, START_STATE_REPLACEMENT_TOLERANCE_PARAM, 1.0 * M_PI / 180.0);
  factory.registerNodeType<snp_application::UpdateTrajectoryStartStateNode>("UpdateTrajectoryStartState", params);

  // Motion plan generation
  try_declare_parameter<std::string>(params.nh, MOTION_GROUP_PARAM, "");
  try_declare_parameter<std::string>(params.nh, REF_FRAME_PARAM, "");
  try_declare_parameter<std::string>(params.nh, TCP_FRAME_PARAM, "");
  factory.registerNodeType<snp_application::AddScanLinkServiceNode>("AddScanLinkService", params);
  factory.registerNodeType<snp_application::GenerateMotionPlanServiceNode>("GenerateMotionPlanService", params);

  try_declare_parameter<std::string>(params.nh, FREESPACE_MOTION_GROUP_PARAM, "");
  factory.registerNodeType<snp_application::GenerateFreespaceMotionPlanServiceNode>(
      "GenerateFreespaceMotionPlanService", params);

  // Industrial reconstruction start
  try_declare_parameter<std::string>(params.nh, CAMERA_FRAME_PARAM, "");
  try_declare_parameter<float>(params.nh, IR_TSDF_VOXEL_PARAM, 0.01f);
  try_declare_parameter<float>(params.nh, IR_TSDF_SDF_PARAM, 0.03f);
  try_declare_parameter<double>(params.nh, IR_TSDF_MIN_X_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MIN_Y_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MIN_Z_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MAX_X_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MAX_Y_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MAX_Z_PARAM, 0.0);
  try_declare_parameter<float>(params.nh, IR_RGBD_DEPTH_SCALE_PARAM, 1000.0);
  try_declare_parameter<float>(params.nh, IR_RGBD_DEPTH_TRUNC_PARAM, 1.1f);
  try_declare_parameter<bool>(params.nh, IR_LIVE_PARAM, true);
  try_declare_parameter<double>(params.nh, IR_NORMAL_ANGLE_TOL_PARAM, -1.0);
  try_declare_parameter<double>(params.nh, IR_NORMAL_X_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_NORMAL_Y_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_NORMAL_Z_PARAM, 1.0);
  factory.registerNodeType<snp_application::StartReconstructionServiceNode>("StartReconstructionService", params);

  // Industrial reconstruction stop
  try_declare_parameter<int>(params.nh, IR_MIN_FACES_PARAM, 0);
  try_declare_parameter<std::string>(params.nh, MESH_FILE_PARAM, "");
  try_declare_parameter<std::string>(params.nh, IR_ARCHIVE_DIR_PARAM, "");
  factory.registerNodeType<snp_application::StopReconstructionServiceNode>("StopReconstructionService", params);

  // Enhanced industrial reconstruction start
  try_declare_parameter<float>(params.nh, IR_TRANSLATION_DISTANCE_PARAM, 0.05f);
  try_declare_parameter<float>(params.nh, IR_ROTATIONAL_DISTANCE_PARAM, 0.01f);
  try_declare_parameter<bool>(params.nh, IR_RGBD_CONVERT_INTENSITY_PARAM, false);
  
  // Enhanced parameters
  try_declare_parameter<bool>(params.nh, IR_ENABLE_DEPTH_PREPROCESSING_PARAM, true);
  try_declare_parameter<bool>(params.nh, IR_ENABLE_MESH_POSTPROCESSING_PARAM, true);
  try_declare_parameter<bool>(params.nh, IR_ENABLE_ADAPTIVE_PARAMS_PARAM, true);
  try_declare_parameter<std::string>(params.nh, IR_QUALITY_MODE_PARAM, "balanced");
  try_declare_parameter<std::string>(params.nh, IR_SCENE_TYPE_PARAM, "industrial");
  
  // Depth preprocessing parameters
  try_declare_parameter<float>(params.nh, IR_BILATERAL_D_PARAM, 9.0f);
  try_declare_parameter<float>(params.nh, IR_BILATERAL_SIGMA_COLOR_PARAM, 75.0f);
  try_declare_parameter<float>(params.nh, IR_BILATERAL_SIGMA_SPACE_PARAM, 75.0f);
  try_declare_parameter<float>(params.nh, IR_MEDIAN_KERNEL_SIZE_PARAM, 5.0f);
  try_declare_parameter<float>(params.nh, IR_MORPHOLOGICAL_KERNEL_SIZE_PARAM, 3.0f);
  try_declare_parameter<float>(params.nh, IR_TEMPORAL_ALPHA_PARAM, 0.1f);
  try_declare_parameter<float>(params.nh, IR_OUTLIER_STD_THRESHOLD_PARAM, 2.0f);
  try_declare_parameter<float>(params.nh, IR_MAX_HOLE_SIZE_PARAM, 10.0f);
  
  // Mesh post-processing parameters
  try_declare_parameter<int>(params.nh, IR_SMOOTHING_ITERATIONS_PARAM, 3);
  try_declare_parameter<float>(params.nh, IR_SMOOTHING_LAMBDA_PARAM, 0.5f);
  try_declare_parameter<float>(params.nh, IR_DECIMATION_RATIO_PARAM, 0.5f);
  try_declare_parameter<float>(params.nh, IR_TARGET_EDGE_LENGTH_PARAM, 0.02f);
  try_declare_parameter<float>(params.nh, IR_HOLE_FILLING_DIAMETER_PARAM, 0.1f);
  try_declare_parameter<float>(params.nh, IR_CURVATURE_THRESHOLD_PARAM, 0.1f);
  try_declare_parameter<float>(params.nh, IR_QUALITY_THRESHOLD_PARAM, 0.8f);
  
  // Filtering options
  try_declare_parameter<bool>(params.nh, IR_ENABLE_BILATERAL_SMOOTHING_PARAM, true);
  try_declare_parameter<bool>(params.nh, IR_ENABLE_CURVATURE_SMOOTHING_PARAM, true);
  try_declare_parameter<bool>(params.nh, IR_ENABLE_LAPLACIAN_SMOOTHING_PARAM, true);
  try_declare_parameter<bool>(params.nh, IR_ENABLE_TOPOLOGY_OPTIMIZATION_PARAM, true);
  try_declare_parameter<bool>(params.nh, IR_ENABLE_HOLE_FILLING_PARAM, true);
  try_declare_parameter<bool>(params.nh, IR_ENABLE_OUTLIER_REMOVAL_PARAM, true);
  
  factory.registerNodeType<snp_application::StartEnhancedReconstructionServiceNode>("StartEnhancedReconstructionService", params);
}
