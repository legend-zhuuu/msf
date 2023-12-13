#ifndef MSM_SURFEL_MAP_VISUALIZER_H
#define MSM_SURFEL_MAP_VISUALIZER_H

#include <visualization_msgs/MarkerArray.h>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

struct SurfelUnit;

/**
 * @brief 面元地图可视化器
 *
 */
class SurfelMapVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Surfel Map Visualizer object 构造函数
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  SurfelMapVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                      std::string _topic_name);

  /**
   * @brief 显示地图
   *
   * @param _surfel_units
   * @param _stamp
   */
  void Display(const std::vector<SurfelUnit>& _surfel_units,
               ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重复显示
   *
   * @param _stamp
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

 private:
  /// @brief 面元marker
  visualization_msgs::MarkerArray surfel_map_markers_;
};
}  // namespace multi_sensor_mapping

#endif