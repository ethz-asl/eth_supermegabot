#include "dense-reconstruction/dense-reconstruction-plugin.h"

#include <cstring>
#include <string>
#include <vector>

#include <console-common/console.h>
#include <dense-reconstruction/conversion-tools.h>
#include <dense-reconstruction/pmvs-file-utils.h>
#include <dense-reconstruction/pmvs-interface.h>
#include <dense-reconstruction/stereo-dense-reconstruction.h>
#include <depth-integration/depth-integration.h>
#include <gflags/gflags.h>
#include <map-manager/map-manager.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <visualization/rviz-visualization-sink.h>
#include <visualization/viwls-graph-plotter.h>
#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/esdf_server.h>

DECLARE_string(map_mission_list);
DECLARE_bool(overwrite);

DEFINE_bool(dense_esdf_use_clear_sphere, false, "");
DEFINE_double(dense_esdf_clear_sphere_inner_radius, 0.3, "");
DEFINE_double(dense_esdf_clear_sphere_outer_radius, 1.0, "");

DEFINE_bool(
    dense_tsdf_icp_refine_roll_and_pitch, false,
    "If enabled, ICP will refine roll and pitch as well when integrating point "
    "clouds into the TSDF or ESDF maps.");

DEFINE_bool(
    dense_use_distorted_camera, false,
    "If enabled, the depth map reprojection assumes that the map has "
    "been created using the distorted image. Therefore, the distorted "
    "camera model is used for reprojection.");

DEFINE_string(
    dense_result_mesh_output_file, "",
    "Path to the PLY mesh file that is generated from the "
    "reconstruction command.");

DEFINE_string(
    dense_esdf_export_map_for_panning_path, "",
    "Path to where the ESDF/TSDF map is exported to. If empty, no "
    "map will be exported.");

DEFINE_string(
    dense_tsdf_integrator_type, "merged",
    "Voxblox TSDF integrator type [simple, merged, fast]");

DEFINE_bool(
    dense_tsdf_icp_enabled, true,
    "If enabled, ICP is used to align the point clouds to the TSDF grid, "
    "before integrating them.");

DEFINE_bool(
    dense_tsdf_icp_accumulate_transformations, true,
    "If enabled, the ICP corrections are accumulated, such that subsequent "
    "alignments start from the pose obtained from the VIMap which is then "
    "corrected based on the previous ICP alignment.");

DEFINE_double(
    dense_tsdf_voxel_size_m, 0.10, "Voxel size of the TSDF grid [m].");

DEFINE_uint64(
    dense_tsdf_voxels_per_side, 16u,
    "Voxels per side of a Block of the TSDF grid.");

DEFINE_bool(
    dense_tsdf_voxel_carving_enabled, true,
    "Voxels per side of a Block of the TSDF grid.");

DEFINE_bool(
    dense_tsdf_voxel_use_clearing_rays, true,
    "If enabled it will ray-trace points that are beyond the maxium ray length "
    "defined by --dense_tsdf_max_ray_length_m up to the threshold to clear "
    "free space. This option is intended to create maps for path planning.");

DEFINE_double(
    dense_tsdf_truncation_distance_m, 0.4,
    "Truncation distance of the TSDF grid [m].");

DEFINE_double(
    dense_tsdf_min_ray_length_m, 0.05,
    "Minimum ray length integrated into the TSDF grid.");

DEFINE_double(
    dense_tsdf_max_ray_length_m, 20.,
    "Maximum ray length integrated into the TSDF grid.");

DEFINE_string(
    dense_image_export_path, "",
    "Export folder for image export function. console command: "
    "export_timestamped_images");

DEFINE_int32(
    dense_depth_resource_output_type, 17,
    "Output resource type of the dense reconstruction algorithms."
    "Supported commands: "
    "stereo_dense_reconstruction "
    "Supported types: "
    "PointCloudXYZRGBN = 17, RawDepthMap = 8");

DEFINE_int32(
    dense_depth_resource_input_type, 21,
    "Input resource type of the dense reconstruction algorithms."
    "Supported commands: "
    "create_tsdf_from_depth_resource "
    "Supported types: "
    "RawDepthMap = 8, OptimizedDepthMap = 9, PointCloudXYZ = 16, "
    "PointCloudXYZRGBN = 17, kPointCloudXYZI = 21");

namespace dense_reconstruction {

bool parseMultipleMissionIds(
    const vi_map::VIMap& vi_map, vi_map::MissionIdList* mission_ids) {
  CHECK_NOTNULL(mission_ids);
  if (!vi_map::csvIdStringToIdList(FLAGS_map_mission_list, mission_ids)) {
    LOG(ERROR) << "The provided CSV mission id list is not valid!";
    return false;
  }

  if (mission_ids->empty()) {
    LOG(INFO) << "No mission id list was provided, operating on all missions!";
    return true;
  }

  LOG(INFO) << "Compute depth maps from multi view stereo for the "
               "following missions:";
  bool success = true;
  for (const vi_map::MissionId& mission_id : *mission_ids) {
    if (mission_id.isValid() && vi_map.hasMission(mission_id)) {
      LOG(INFO) << "-> " << mission_id;
    } else {
      LOG(ERROR) << "-> " << mission_id
                 << " does not exist in the selected map!";
      success = false;
    }
  }
  return success;
}

common::CommandStatus exportTsdfMeshToFile(
    const std::string& mesh_file_path, const voxblox::TsdfMap& tsdf_map) {
  if (mesh_file_path.empty()) {
    LOG(ERROR) << "No mesh output path specified, please set "
                  "--dense_result_mesh_output_file .";
    return common::kStupidUserError;
  }

  if (!common::createPathToFile(mesh_file_path)) {
    LOG(ERROR) << "Unable to create a path for the mesh output file: "
               << mesh_file_path;
    return common::kStupidUserError;
  }

  if (common::fileExists(mesh_file_path)) {
    LOG(ERROR) << "Output mesh file already exists: " << mesh_file_path;
    return common::kStupidUserError;
  }

  voxblox::MeshLayer mesh_layer(tsdf_map.block_size());

  voxblox::MeshIntegratorConfig mesh_config;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_config, tsdf_map.getTsdfLayer(), &mesh_layer);
  // We mesh the whole grid at once anyways, so all of them should be
  // updated.
  constexpr bool kMeshOnlyUpdatedBlocks = false;
  // No need to reset, we are not gonna mesh again.
  constexpr bool kResetUpdatedFlag = false;
  mesh_integrator.generateMesh(kMeshOnlyUpdatedBlocks, kResetUpdatedFlag);

  voxblox::outputMeshLayerAsPly(mesh_file_path, mesh_layer);
  return common::kSuccess;
}

DenseReconstructionPlugin::DenseReconstructionPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"export_timestamped_images"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (FLAGS_dense_image_export_path.empty()) {
          LOG(ERROR) << "Please define the export path with "
                        "--dense_image_export_path!";
          return common::kStupidUserError;
        }

        if (!dense_reconstruction::exportAllImagesForCalibration(
                FLAGS_dense_image_export_path, map.get())) {
          return common::kUnknownError;
        }
        return common::kSuccess;
      },
      "Export timestamped image resources, such that they can be used for "
      "calibration. Use --dense_image_export_path to set the export path.",
      common::Processing::Sync);

  addCommand(
      {"export_for_pmvs_reconstruction", "export_for_pmvs"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapReadAccess map =
            map_manager.getMapReadAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        const dense_reconstruction::PmvsConfig config =
            dense_reconstruction::PmvsConfig::getFromGflags();
        if (mission_ids.empty()) {
          if (!dense_reconstruction::exportVIMapToPmvsSfmInputData(
                  config, *map)) {
            return common::kUnknownError;
          }
        } else {
          if (!dense_reconstruction::exportVIMapToPmvsSfmInputData(
                  config, mission_ids, *map)) {
            return common::kUnknownError;
          }
        }
        return common::kSuccess;
      },
      "Export the map and the associated image resources to the PMVS/CMVS "
      "input format, such that we can reconstruct the whole map.",
      common::Processing::Sync);

  addCommand(
      {"stereo_dense_reconstruction", "stereo_dense", "sdr"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        const backend::ResourceType output_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_output_type);

        if (mission_ids.empty()) {
          dense_reconstruction::computeDepthForAllStereoCameras(
              output_resource_type, map.get());
        } else {
          dense_reconstruction::computeDepthForAllStereoCameras(
              output_resource_type, mission_ids, map.get());
        }
        return common::kSuccess;
      },
      "Uses OpenCvs stereo matcher to compute depth resources for all stereo "
      "cameras in the map (or all selected missions). Use the "
      "--dense_stereo_* "
      "flags for configuration of the stereo matcher. Currently only the "
      "pinhole camera model is supported. The depth output type can be set "
      "using --dense_depth_resource_output_type",
      common::Processing::Sync);

  addCommand(
      {"convert_all_depth_maps_to_point_clouds"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (!dense_reconstruction::convertAllDepthMapsToPointClouds(
                map.get())) {
          return common::kUnknownError;
        }
        return common::kSuccess;
      },
      "Convert all depth maps into point clouds (which are stored as PLYs).",
      common::Processing::Sync);

  addCommand(
      {"create_tsdf_from_depth_resource", "tsdf", "depth_fusion"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        voxblox::TsdfIntegratorBase::Config tsdf_integrator_config;

        tsdf_integrator_config.voxel_carving_enabled =
            FLAGS_dense_tsdf_voxel_carving_enabled;
        tsdf_integrator_config.allow_clear =
            FLAGS_dense_tsdf_voxel_use_clearing_rays;
        tsdf_integrator_config.default_truncation_distance =
            static_cast<float>(FLAGS_dense_tsdf_truncation_distance_m);
        tsdf_integrator_config.min_ray_length_m =
            static_cast<voxblox::FloatingPoint>(
                FLAGS_dense_tsdf_min_ray_length_m);
        tsdf_integrator_config.max_ray_length_m =
            static_cast<voxblox::FloatingPoint>(
                FLAGS_dense_tsdf_max_ray_length_m);

        voxblox::TsdfMap::Config tsdf_map_config;
        tsdf_map_config.tsdf_voxel_size =
            static_cast<voxblox::FloatingPoint>(FLAGS_dense_tsdf_voxel_size_m);
        tsdf_map_config.tsdf_voxels_per_side = FLAGS_dense_tsdf_voxels_per_side;

        voxblox::TsdfMap tsdf_map(tsdf_map_config);

        voxblox::TsdfIntegratorBase::Ptr integrator =
            voxblox::TsdfIntegratorFactory::create(
                FLAGS_dense_tsdf_integrator_type, tsdf_integrator_config,
                tsdf_map.getTsdfLayerPtr());

        voxblox::ICP::Config config;
        config.refine_roll_pitch = true;
        voxblox::ICP icp(config);

        voxblox::Transformation T_G_C_icp_correction;

        depth_integration::IntegrationFunction integration_function =
            [&integrator, &icp, &T_G_C_icp_correction, &tsdf_map](
                const voxblox::Transformation& T_G_C,
                const voxblox::Pointcloud& points,
                const voxblox::Colors& colors) {
              voxblox::Transformation T_G_C_refined = T_G_C;

              if (FLAGS_dense_tsdf_icp_enabled) {
                if (!FLAGS_dense_tsdf_icp_accumulate_transformations) {
                  T_G_C_icp_correction.setIdentity();
                }
                icp.runICP(
                    tsdf_map.getTsdfLayer(), points,
                    T_G_C_icp_correction * T_G_C, &T_G_C_refined);

                T_G_C_icp_correction = T_G_C_refined * T_G_C.inverse();

                if (!icp.refiningRollPitch()) {
                  // its already removed internally but small floating point
                  // errors can build up if accumulating transforms
                  voxblox::Transformation::Vector6 T_vec =
                      T_G_C_icp_correction.log();
                  T_vec[3] = 0.0;
                  T_vec[4] = 0.0;
                  T_G_C_icp_correction = voxblox::Transformation::exp(T_vec);
                }
              }

              integrator->integratePointCloud(T_G_C_refined, points, colors);
            };

        const backend::ResourceType input_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_input_type);

        if (!depth_integration::integrateAllDepthResourcesOfType(
                mission_ids, input_resource_type,
                !FLAGS_dense_use_distorted_camera, *map,
                integration_function)) {
          LOG(ERROR) << "Unable to compute Voxblox TSDF grid.";
          return common::kStupidUserError;
        }

        const bool has_resource = map->hasVoxbloxTsdfMap(mission_ids);
        if (has_resource && FLAGS_overwrite) {
          map->replaceVoxbloxTsdfMap(mission_ids, tsdf_map);
        } else if (has_resource && !FLAGS_overwrite) {
          LOG(ERROR)
              << "Could not store the Voxblox TSDF map, because there is "
              << "already a map stored. Use --overwrite!";
          return common::kStupidUserError;
        } else {
          map->storeVoxbloxTsdfMap(tsdf_map, mission_ids);
        }

        constexpr double kBytesToMegaBytes = 1e-6;
        LOG(INFO) << "TSDF map:";
        LOG(INFO) << "  allocated blocks: "
                  << tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks();
        LOG(INFO) << "  size: "
                  << tsdf_map.getTsdfLayer().getMemorySize() * kBytesToMegaBytes
                  << "MB";

        if (!FLAGS_dense_result_mesh_output_file.empty()) {
          return exportTsdfMeshToFile(
              FLAGS_dense_result_mesh_output_file, tsdf_map);
        }
        return common::kSuccess;
      },
      "Use all depth resources the selected missions "
      "and integrate them into a Voxblox TSDF map. The map is then stored as "
      "resource associated with the selected set of missions. This command "
      "will use the resource type specified by "
      "--dense_depth_resource_input_type if available.",
      common::Processing::Sync);

  addCommand(
      {"create_esdf_from_depth_resource", "esdf"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        ros::NodeHandle& nh_private =
            visualization::RVizVisualizationSink::getNodeHandle();
        ros::NodeHandle nh;

        voxblox::TsdfIntegratorBase::Config tsdf_integrator_config;

        tsdf_integrator_config.voxel_carving_enabled =
            FLAGS_dense_tsdf_voxel_carving_enabled;
        tsdf_integrator_config.allow_clear =
            FLAGS_dense_tsdf_voxel_use_clearing_rays;
        tsdf_integrator_config.default_truncation_distance =
            static_cast<float>(FLAGS_dense_tsdf_truncation_distance_m);
        tsdf_integrator_config.min_ray_length_m =
            static_cast<voxblox::FloatingPoint>(
                FLAGS_dense_tsdf_min_ray_length_m);
        tsdf_integrator_config.max_ray_length_m =
            static_cast<voxblox::FloatingPoint>(
                FLAGS_dense_tsdf_max_ray_length_m);

        voxblox::TsdfMap::Config tsdf_map_config;
        tsdf_map_config.tsdf_voxel_size =
            static_cast<voxblox::FloatingPoint>(FLAGS_dense_tsdf_voxel_size_m);
        tsdf_map_config.tsdf_voxels_per_side = FLAGS_dense_tsdf_voxels_per_side;

        voxblox::EsdfMap::Config esdf_config;
        esdf_config.esdf_voxel_size = tsdf_map_config.tsdf_voxel_size;
        esdf_config.esdf_voxels_per_side = tsdf_map_config.tsdf_voxels_per_side;

        voxblox::EsdfIntegrator::Config esdf_integrator_config;
        esdf_integrator_config.min_distance_m =
            static_cast<float>(FLAGS_dense_tsdf_truncation_distance_m) * 0.75f;
        esdf_integrator_config.clear_sphere_radius =
            FLAGS_dense_esdf_clear_sphere_inner_radius;
        esdf_integrator_config.occupied_sphere_radius =
            FLAGS_dense_esdf_clear_sphere_outer_radius;

        voxblox::EsdfServer esdf_server(
            nh, nh_private, esdf_config, esdf_integrator_config,
            tsdf_map_config, tsdf_integrator_config);
        esdf_server.setWorldFrame("map");
        esdf_server.setClearSphere(FLAGS_dense_esdf_use_clear_sphere);

        voxblox::ICP::Config config;
        config.refine_roll_pitch = FLAGS_dense_tsdf_icp_refine_roll_and_pitch;
        voxblox::ICP icp(config);

        voxblox::Transformation T_G_C_icp_correction;

        voxblox::TsdfMap& tsdf_map = *(esdf_server.getTsdfMapPtr());
        voxblox::EsdfMap& esdf_map = *(esdf_server.getEsdfMapPtr());

        depth_integration::IntegrationFunction integration_function =
            [&esdf_server, &icp, &T_G_C_icp_correction, &tsdf_map](
                const voxblox::Transformation& T_G_C,
                const voxblox::Pointcloud& points,
                const voxblox::Colors& colors) {
              CHECK_EQ(points.size(), colors.size());

              voxblox::Transformation T_G_C_refined = T_G_C;

              if (FLAGS_dense_tsdf_icp_enabled) {
                if (!FLAGS_dense_tsdf_icp_accumulate_transformations) {
                  T_G_C_icp_correction.setIdentity();
                }
                icp.runICP(
                    tsdf_map.getTsdfLayer(), points,
                    T_G_C_icp_correction * T_G_C, &T_G_C_refined);

                T_G_C_icp_correction = T_G_C_refined * T_G_C.inverse();

                if (!icp.refiningRollPitch()) {
                  // its already removed internally but small floating point
                  // errors can build up if accumulating transforms
                  voxblox::Transformation::Vector6 T_vec =
                      T_G_C_icp_correction.log();
                  T_vec[3] = 0.0;
                  T_vec[4] = 0.0;
                  T_G_C_icp_correction = voxblox::Transformation::exp(T_vec);
                }
              }
              esdf_server.integratePointcloud(T_G_C_refined, points, colors);

              ros::spinOnce();

              esdf_server.updateEsdf();

              ros::spinOnce();

              esdf_server.publishPointclouds();

              ros::spinOnce();

              esdf_server.publishSlices();

              ros::spinOnce();
            };

        const backend::ResourceType input_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_input_type);

        if (!depth_integration::integrateAllDepthResourcesOfType(
                mission_ids, input_resource_type,
                !FLAGS_dense_use_distorted_camera, *map,
                integration_function)) {
          LOG(ERROR) << "Unable to compute Voxblox ESDF grid.";
          return common::kStupidUserError;
        }

        if (!FLAGS_dense_esdf_export_map_for_panning_path.empty()) {
          CHECK(common::createPathToFile(
              FLAGS_dense_esdf_export_map_for_panning_path));
          CHECK(esdf_server.saveMap(
              FLAGS_dense_esdf_export_map_for_panning_path));
        }

        const bool has_esdf_resource = map->hasVoxbloxEsdfMap(mission_ids);
        if (has_esdf_resource && FLAGS_overwrite) {
          map->replaceVoxbloxEsdfMap(mission_ids, esdf_map);
        } else if (has_esdf_resource && !FLAGS_overwrite) {
          LOG(ERROR)
              << "Could not store the Voxblox ESDF map, because there is "
              << "already a map stored. Use --overwrite!";
          return common::kStupidUserError;
        } else {
          map->storeVoxbloxEsdfMap(esdf_map, mission_ids);
        }

        const bool has_tsdf_resource = map->hasVoxbloxTsdfMap(mission_ids);
        if (has_tsdf_resource && FLAGS_overwrite) {
          map->replaceVoxbloxTsdfMap(mission_ids, tsdf_map);
        } else if (has_tsdf_resource && !FLAGS_overwrite) {
          LOG(ERROR)
              << "Could not store the Voxblox TSDF map, because there is "
              << "already a map stored. Use --overwrite!";
          return common::kStupidUserError;
        } else {
          map->storeVoxbloxTsdfMap(tsdf_map, mission_ids);
        }

        constexpr double kBytesToMegaBytes = 1e-6;
        LOG(INFO) << "=============  ESDF Map =============";
        LOG(INFO) << "TSDF layer:";
        LOG(INFO) << "  allocated blocks: "
                  << tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks();
        LOG(INFO) << "  size: "
                  << tsdf_map.getTsdfLayer().getMemorySize() * kBytesToMegaBytes
                  << "MB";
        LOG(INFO) << "ESDF layer:";
        LOG(INFO) << "  allocated blocks: "
                  << esdf_map.getEsdfLayer().getNumberOfAllocatedBlocks();
        LOG(INFO) << "  size: "
                  << esdf_map.getEsdfLayer().getMemorySize() * kBytesToMegaBytes
                  << "MB";

        if (!FLAGS_dense_result_mesh_output_file.empty()) {
          return exportTsdfMeshToFile(
              FLAGS_dense_result_mesh_output_file, tsdf_map);
        }
        return common::kSuccess;
      },
      "Use all depth resources the selected missions "
      "and integrate them into a Voxblox TSDF/ESDF map. The map is then stored "
      "as "
      "resource associated with the selected set of missions. This command "
      "will use the resource type specified by "
      "--dense_depth_resource_input_type if available.",
      common::Processing::Sync);

  addCommand(
      {"create_mesh_from_tsdf_grid", "export_tsdf"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        voxblox::TsdfMap::Config tsdf_map_config;
        voxblox::TsdfMap tsdf_map(tsdf_map_config);
        if (!map.get()->getVoxbloxTsdfMap(mission_ids, &tsdf_map)) {
          LOG(ERROR)
              << "No Voxblox TSDF grid stored for the selected missions!";
          return common::kStupidUserError;
        }

        return exportTsdfMeshToFile(
            FLAGS_dense_result_mesh_output_file, tsdf_map);
      },
      "Compute mesh of the Voxblox TSDF grid resource associated with "
      "the selected missions.",
      common::Processing::Sync);
}

}  // namespace dense_reconstruction

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    dense_reconstruction::DenseReconstructionPlugin);
