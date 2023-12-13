#ifndef MSM_REFLECTION_COLUMN_VISUALIZER_H
#define MSM_REFLECTION_COLUMN_VISUALIZER_H

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

struct ReflectionColumn;

/**
 * @brief 反光柱可视化器
 *
 */
class ReflectionColumnVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Reflection Column Visualizer object 构造函数
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  ReflectionColumnVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                             std::string _topic_name);

  /**
   * @brief 显示地图
   *
   * @param _surfel_units
   * @param _stamp
   */
  void Display(const std::vector<ReflectionColumn>& _reflection_columns,
               ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重复显示
   *
   * @param _stamp
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

 private:
  /// @brief 面元marker
  visualization_msgs::MarkerArray reflection_column_markers_;
};

}  // namespace multi_sensor_mapping

#endif