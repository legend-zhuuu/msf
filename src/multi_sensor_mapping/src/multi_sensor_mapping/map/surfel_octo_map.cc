#include "multi_sensor_mapping/map/surfel_octo_map.h"

#include "multi_sensor_mapping/utils/utils_log.h"
// #include "ufo/map/point_cloud.h"

// using ufo::map::PointCloud;

namespace multi_sensor_mapping {

SurfelOctoMap::SurfelOctoMap(double _resolution, int _depth_level)
    : leaf_resolution_(_resolution), depth_level_(_depth_level) {
  // map_core_ =
  //     std::make_shared<ufo::map::SurfelMap>(leaf_resolution_, depth_level_);
}

void SurfelOctoMap::InsertCloud(const CloudTypePtr& _cloud) {
  //   int cloud_size = _cloud->size();

  //   ufo::map::PointCloud ufo_cloud;
  //   ufo_cloud.resize(cloud_size);

  // #pragma omp parallel for num_threads(omp_get_max_threads())
  //   for (int i = 0; i < cloud_size; i++) {
  //     ufo_cloud[i].x = (float)_cloud->points[i].x;
  //     ufo_cloud[i].y = (float)_cloud->points[i].y;
  //     ufo_cloud[i].z = (float)_cloud->points[i].z;
  //   }
  //   map_core_->insertSurfelPoint(std::begin(ufo_cloud), std::end(ufo_cloud));
}

std::vector<SurfelUnit> SurfelOctoMap::QuerySurfelUnits(int _query_min_depth,
                                                        int _query_max_depth,
                                                        double _min_planarity) {
  // std::vector<SurfelUnit> surfel_result;
  // namespace ufopred = ufo::map::predicate;
  // auto pred = ufopred::HasSurfel() && ufopred::DepthMin(_query_min_depth) &&
  //             ufopred::DepthMax(_query_max_depth - 1) &&
  //             ufopred::NumSurfelPointsMin(10) &&
  //             ufopred::SurfelPlanarityMin(_min_planarity);

  // for (auto const& node : map_core_->query(pred)) {
  //   auto const& surfel = map_core_->getSurfel(node);

  //   Eigen::Vector3d evals =
  //       Eigen::Vector3d(surfel.getEigenValues().x, surfel.getEigenValues().y,
  //                       surfel.getEigenValues().z);

  //   auto surfel_evecs = surfel.getEigenVectors();
  //   Eigen::Matrix3d evecs;
  //   for (int i = 0; i < 3; i++) {
  //     evecs.col(i) = Eigen::Vector3d(surfel_evecs[i].x, surfel_evecs[i].y,
  //                                    surfel_evecs[i].z);
  //   }

  //   Eigen::Matrix3d evecs_reset;
  //   Eigen::Vector3d evals_reset = evals;
  //   Eigen::Vector3d tempp =
  //       evecs.block<3, 1>(0, 0).cross(evecs.block<3, 1>(0, 1));
  //   evecs_reset.block<3, 2>(0, 0) = evecs.block<3, 2>(0, 0);
  //   evecs_reset.block<3, 1>(0, 2) = tempp;

  //   Eigen::Quaterniond surfel_direction(evecs_reset);
  //   surfel_direction.normalized();

  //   SurfelUnit unit;
  //   unit.mean = Eigen::Vector3d(surfel.getMean().x, surfel.getMean().y,
  //                               surfel.getMean().z);
  //   unit.normal = Eigen::Vector3d(surfel.getNormal().x, surfel.getNormal().y,
  //                                 surfel.getNormal().z);
  //   unit.eigen_value = evals_reset;
  //   unit.direction = surfel_direction;
  //   unit.depth_level = node.depth();
  //   unit.leaf_resolution = map_core_->getNodeSize(node.depth());

  //   surfel_result.push_back(unit);
  // }

  // return surfel_result;
}

void SurfelOctoMap::Save(std::string _path) {
  // map_core_->write(_path);
}

bool SurfelOctoMap::Load(std::string _path) { return true; }

}  // namespace multi_sensor_mapping