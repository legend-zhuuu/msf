#ifndef MSM_POSE_GRAPH_VISUALIZER_H
#define MSM_POSE_GRAPH_VISUALIZER_H

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

class PoseGraph;

/**
 * @brief 3维三角形
 *
 */
class Triangle3D {
 public:
  /**
   * @brief Construct a new Triangle 3 D object 构造函数
   *
   */
  Triangle3D();

  /**
   * @brief Construct a new Triangle 3 D object 构造函数
   *
   * @param _length
   * @param _width
   * @param _height
   */
  Triangle3D(const double _length, const double _width);

  /**
   * @brief 变换
   *
   * @param _translation
   * @param _rotation
   */
  std::vector<Eigen::Vector3d> Transform(const Eigen::Vector3d& _translation,
                                         const Eigen::Quaterniond& _rotation);

  /**
   * @brief 变换
   *
   * @param _mat
   */
  std::vector<Eigen::Vector3d> Transform(const Eigen::Matrix4d& _mat);

 private:
  /// @brief 三角形顶点
  Eigen::Vector3d vertex1_;
  Eigen::Vector3d vertex2_;
  Eigen::Vector3d vertex3_;
};

/**
 * @brief 位姿图可视化器
 *
 */
class PoseGraphVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Pose Graph Visualizer object 构造函数
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  PoseGraphVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                      std::string _topic_name);

  /**
   * @brief 显示
   *
   * @param _graph
   * @param _stamp
   */
  void Display(const PoseGraph& _graph, ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重复显示
   *
   * @param _stamp
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

 private:
  /// @brief 位姿图 marker
  visualization_msgs::MarkerArray pose_graph_markers_;
};

}  // namespace multi_sensor_mapping

#endif