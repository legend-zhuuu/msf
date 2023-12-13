#include "pclomp/ndt_omp.h"

#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <cstdlib>
#include <ctime>

#include <eigen3/Eigen/Eigen>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using CloudTypePtr = CloudType::Ptr;

class TicToc {
public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return (elapsed_seconds.count() * 1000);
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

int main(int argc, char** argv) {
  std::string dataset_folder_path = "/home/hkw/dataset/Yang2Lidar";

  int num_cores = 2;
  double target_resolution = 2.0;
  double source_resolution = 1.0;

  std::string target_cloud_path = dataset_folder_path + "/global_cloud.pcd";

  CloudTypePtr target_cloud(new CloudType);
  if(pcl::io::loadPCDFile(target_cloud_path, *target_cloud) == -1) {
    std::cerr << "failed to load " << target_cloud_path << std::endl;
    return 0;
  }

  std::vector<double> align_time_cache;

  pcl::VoxelGrid<PointType> cloud_filter;
  cloud_filter.setLeafSize(source_resolution, source_resolution, source_resolution);

  // 初始化NDT空间
  TicToc timer;
  pclomp::NormalDistributionsTransform<PointType, PointType> ndt_map;
  ndt_map.setNumThreads(num_cores);
  ndt_map.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_map.setTransformationEpsilon(0.01);
  ndt_map.setStepSize(0.1);
  ndt_map.setResolution(target_resolution);
  ndt_map.setMaximumIterations(30);
  ndt_map.setOulierRatio(0.15);
  ndt_map.setInputTarget(target_cloud);
  std::cout << "Init NDT : " << timer.toc() << std::endl;

  // 读取姿态数据
  std::string ref_cloud_pose_path = dataset_folder_path + "/cloud_pose.txt";
  std::vector<Eigen::Matrix4f> ref_cloud_pose_cache;
  std::vector<double> cloud_timestamp_cache;
  std::vector<std::pair<double, Eigen::Matrix4f>> cloud_aligned_cache;

  std::ifstream infile;
  infile.open(ref_cloud_pose_path);
  std::string current_line;

  while(std::getline(infile, current_line)) {
    std::istringstream s(current_line);
    std::string field;
    std::vector<double> vec;
    while(std::getline(s, field, ' ')) {
      if(field.empty())  // Skip if empty
        continue;
      // save the data to our vector
      vec.push_back(std::atof(field.c_str()));
    }

    if(vec.size() == 8) {
      cloud_timestamp_cache.push_back(vec[0]);
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
      pose(0, 3) = vec[1];
      pose(1, 3) = vec[2];
      pose(2, 3) = vec[3];
      Eigen::Quaternionf q(vec[7], vec[4], vec[5], vec[6]);
      q.normalize();
      pose.block<3, 3>(0, 0) = q.toRotationMatrix();
      ref_cloud_pose_cache.push_back(pose);
    }
  }

  std::cout << "Load pose size : " << ref_cloud_pose_cache.size() << std::endl;

  // 读取点云数据并匹配
  for(size_t i = 1; i < ref_cloud_pose_cache.size(); i++) {
    std::string source_cloud_path = dataset_folder_path + "/cloud/cloud" + std::to_string(i) + ".pcd";
    CloudTypePtr source_cloud(new CloudType);
    if(pcl::io::loadPCDFile(source_cloud_path, *source_cloud) == -1) {
      continue;
    }

    TicToc timer2;
    CloudTypePtr source_cloud_ds(new CloudType);
    cloud_filter.setInputCloud(source_cloud);
    cloud_filter.filter(*source_cloud_ds);

    ndt_map.setInputSource(source_cloud_ds);
    CloudType output;
    ndt_map.align(output, ref_cloud_pose_cache[i - 1]);
    align_time_cache.push_back(timer2.toc());

    cloud_aligned_cache.push_back(std::make_pair(cloud_timestamp_cache[i], ndt_map.getFinalTransformation()));
  }

  // 计算统计结果
  if(align_time_cache.empty()) {
    return 0;
  }

  double min = align_time_cache[1];
  double max = align_time_cache[1];
  double sum = 0;
  int max_index = 0;
  for(size_t i = 1; i < align_time_cache.size(); i++) {
    sum += align_time_cache[i];
    if(align_time_cache[i] > max) {
      max = align_time_cache[i];
      max_index = i;
    } else if(align_time_cache[i] < min) {
      min = align_time_cache[i];
    }
  }
  int cnt = (int)align_time_cache.size();
  double avg = sum / (double)cnt;
  printf("\n----------------------------Summary----------------------------\n");
  printf("|%-25s|%-6s|%-6s|%-6s|%-6s\n", "Item", "num", "max", "min", "avg");
  printf("-----------------------------------------------------------------\n");
  printf("|%-25s|%-6i|%-6.3f|%-6.3f|%-6.3f\n", "NDT", cnt, max, min, avg);

  std::cout << "max index : " << max_index << std::endl;

  std::string align_result_path = dataset_folder_path + "/align_result.txt";
  std::ofstream align_result_outfile;
  align_result_outfile.open(align_result_path);
  for(size_t i = 0; i < cloud_aligned_cache.size(); i++) {
    align_result_outfile.precision(16);
    align_result_outfile << cloud_aligned_cache[i].first << " ";
    align_result_outfile.precision(5);
    Eigen::Vector3f p = cloud_aligned_cache[i].second.block<3, 1>(0, 3);
    Eigen::Quaternionf q(cloud_aligned_cache[i].second.block<3, 3>(0, 0));
    q.normalize();

    align_result_outfile << p(0) << " " << p(1) << " " << p(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  align_result_outfile.close();
  return 0;
}