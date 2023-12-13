#include "multi_sensor_mapping/frontend/gnss/GNSSInitializer.h"
#include "multi_sensor_mapping/factor/auto_diff/GnssFactor.h"

namespace multi_sensor_mapping {

GNSSInitializer::GNSSInitializer()
    : ext_initialized_flag_(false),
      map_in_local_ENU_position_(Eigen::Vector3d(0, 0, 0)),
      map_in_local_ENU_orientation_(Eigen::Quaterniond::Identity()) {}

bool GNSSInitializer::GNSSAlign(
    const std::vector<std::pair<GNSSData, Transf> > _gnss_pose_data) {
  ceres::Problem::Options options;
  options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  std::shared_ptr<ceres::Problem> opt_problem =
      std::make_shared<ceres::Problem>(options);

  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;

  for (size_t i = 0; i < _gnss_pose_data.size(); i++) {
    Eigen::Vector3d pose_in_map =
        _gnss_pose_data[i].second.block<3, 1>(0, 3).template cast<double>();
    Eigen::Vector3d pose_in_LENU = Eigen::Vector3d(
        _gnss_pose_data[i].first.local_E, _gnss_pose_data[i].first.local_N,
        _gnss_pose_data[i].first.local_U);

    using Functor = GNSSAlignmentADFactor;
    Functor *functor = new Functor(pose_in_map, pose_in_LENU);
    auto *cost_function =
        new ceres::DynamicAutoDiffCostFunction<Functor>(functor);
    cost_function->AddParameterBlock(3);
    cost_function->AddParameterBlock(4);

    cost_function->SetNumResiduals(3);

    std::vector<double *> vec;
    opt_problem->AddParameterBlock(map_in_local_ENU_position_.data(), 3);
    vec.emplace_back(map_in_local_ENU_position_.data());
    opt_problem->AddParameterBlock(
        map_in_local_ENU_orientation_.coeffs().data(), 4,
        quaternion_local_parameterization);
    vec.emplace_back(map_in_local_ENU_orientation_.coeffs().data());

    opt_problem->AddResidualBlock(cost_function, NULL, vec);
  }

  ceres::Solver::Options opt_options;
  opt_options.minimizer_type = ceres::TRUST_REGION;
  opt_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  opt_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  opt_options.minimizer_progress_to_stdout = true;
  opt_options.num_threads = 4;
  opt_options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(opt_options, opt_problem.get(), &summary);

  LOG(INFO) << summary.BriefReport();

  PrintResult();

  if (!summary.IsSolutionUsable()) {
    return false;
  }
  return true;
}

bool GNSSInitializer::GNSSAlign4D(
    const std::vector<std::pair<GNSSData, Transf> > _gnss_pose_data) {
  ceres::Problem::Options options;
  options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  std::shared_ptr<ceres::Problem> opt_problem =
      std::make_shared<ceres::Problem>(options);

  double yaw = 0;

  for (size_t i = 0; i < _gnss_pose_data.size(); i++) {
    Eigen::Vector3d pose_in_map =
        _gnss_pose_data[i].second.block<3, 1>(0, 3).template cast<double>();
    Eigen::Vector3d pose_in_LENU = Eigen::Vector3d(
        _gnss_pose_data[i].first.local_E, _gnss_pose_data[i].first.local_N,
        _gnss_pose_data[i].first.local_U);

    using Functor = GNSSAlignmentADFactor2;
    Functor *functor = new Functor(pose_in_map, pose_in_LENU);
    auto *cost_function =
        new ceres::DynamicAutoDiffCostFunction<Functor>(functor);
    cost_function->AddParameterBlock(3);
    cost_function->AddParameterBlock(1);

    cost_function->SetNumResiduals(3);

    std::vector<double *> vec;
    opt_problem->AddParameterBlock(map_in_local_ENU_position_.data(), 3);
    vec.emplace_back(map_in_local_ENU_position_.data());
    opt_problem->AddParameterBlock(&yaw, 1);
    vec.emplace_back(&yaw);

    opt_problem->AddResidualBlock(cost_function, NULL, vec);
  }

  ceres::Solver::Options opt_options;
  opt_options.minimizer_type = ceres::TRUST_REGION;
  opt_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  opt_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  opt_options.minimizer_progress_to_stdout = true;
  opt_options.num_threads = 4;
  opt_options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(opt_options, opt_problem.get(), &summary);

  LOG(INFO) << summary.BriefReport();

  Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  map_in_local_ENU_orientation_ = Eigen::Quaterniond(yaw_angle);

  PrintResult();

  if (!summary.IsSolutionUsable()) {
    return false;
  }
  return true;
}

bool GNSSInitializer::GNSSAlign5D(
    const std::vector<std::pair<GNSSData, Transf> > _gnss_pose_data) {
  ceres::Problem::Options options;
  options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  std::shared_ptr<ceres::Problem> opt_problem =
      std::make_shared<ceres::Problem>(options);

  double yaw = 0;
  double pitch = 0;

  for (size_t i = 0; i < _gnss_pose_data.size(); i++) {
    Eigen::Vector3d pose_in_map =
        _gnss_pose_data[i].second.block<3, 1>(0, 3).template cast<double>();
    Eigen::Vector3d pose_in_LENU = Eigen::Vector3d(
        _gnss_pose_data[i].first.local_E, _gnss_pose_data[i].first.local_N,
        _gnss_pose_data[i].first.local_U);

    using Functor = GNSSAlignmentADFactor3;
    Functor *functor = new Functor(pose_in_map, pose_in_LENU);
    auto *cost_function =
        new ceres::DynamicAutoDiffCostFunction<Functor>(functor);
    cost_function->AddParameterBlock(3);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(1);

    cost_function->SetNumResiduals(3);

    std::vector<double *> vec;
    opt_problem->AddParameterBlock(map_in_local_ENU_position_.data(), 3);
    vec.emplace_back(map_in_local_ENU_position_.data());
    opt_problem->AddParameterBlock(&yaw, 1);
    vec.emplace_back(&yaw);
    opt_problem->AddParameterBlock(&pitch, 1);
    vec.emplace_back(&pitch);

    opt_problem->AddResidualBlock(cost_function, NULL, vec);
  }

  ceres::Solver::Options opt_options;
  opt_options.minimizer_type = ceres::TRUST_REGION;
  opt_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  opt_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  opt_options.minimizer_progress_to_stdout = true;
  opt_options.num_threads = 4;
  opt_options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(opt_options, opt_problem.get(), &summary);

  LOG(INFO) << summary.BriefReport();

  map_in_local_ENU_orientation_ =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));

  PrintResult();

  return true;
}

bool GNSSInitializer::CheckAlignResult(
    const std::vector<std::pair<GNSSData, Transf> > _gnss_pose_data) {
  double accum_distance = 0;
  Eigen::Vector3d latest_pose_in_map(0, 0, 0);
  for (size_t i = 0; i < _gnss_pose_data.size(); i++) {
    Eigen::Vector3d pose_in_map =
        _gnss_pose_data[i].second.block<3, 1>(0, 3).template cast<double>();
    Eigen::Vector3d pose_in_LENU = map_in_local_ENU_orientation_ * pose_in_map +
                                   map_in_local_ENU_position_;

    //    std::cout << "map : " << pose_in_LENU.transpose() << std::endl;

    Eigen::Vector3d gnss_in_LENU = Eigen::Vector3d(
        _gnss_pose_data[i].first.local_E, _gnss_pose_data[i].first.local_N,
        _gnss_pose_data[i].first.local_U);

    //    std::cout << "gnss : " << gnss_in_LENU.transpose() << std::endl;

    double delta_distance = sqrt((pose_in_LENU(0) - gnss_in_LENU(0)) *
                                     (pose_in_LENU(0) - gnss_in_LENU(0)) +
                                 (pose_in_LENU(1) - gnss_in_LENU(1)) *
                                     (pose_in_LENU(1) - gnss_in_LENU(1)));
    accum_distance += sqrt((pose_in_map(0) - latest_pose_in_map(0)) *
                               (pose_in_map(0) - latest_pose_in_map(0)) +
                           (pose_in_map(1) - latest_pose_in_map(1)) *
                               (pose_in_map(1) - latest_pose_in_map(1)));
    if (accum_distance > 2.0) {
      double deviation_ratio = delta_distance / accum_distance;
      if (deviation_ratio > 0.5) {
        //        std::cout << "id : " << i << " deviation_ratio : " <<
        //        deviation_ratio
        //                  << std::endl;
        //        std::cout << "delta_distance : " << delta_distance
        //                  << " accum_distance : " << accum_distance <<
        //                  std::endl;

        return false;
      }
    }

    latest_pose_in_map = pose_in_map;
  }

  return true;
}

void GNSSInitializer::PrintResult() {
  LOG(INFO) << "Map in Local-ENU : ";
  LOG(INFO) << "positon: " << map_in_local_ENU_position_.transpose();

  Eigen::Vector3d euler =
      map_in_local_ENU_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
  euler = euler * 180.0 / M_PI;

  LOG(INFO) << "rotation : " << euler.transpose();
}
}  // namespace multi_sensor_mapping
