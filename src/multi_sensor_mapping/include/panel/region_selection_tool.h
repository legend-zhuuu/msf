#ifndef MSM_REGION_SELECTION_TOOL_H
#define MSM_REGION_SELECTION_TOOL_H

#ifndef Q_MOC_RUN
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "rviz/tool.h"

#include <QCursor>
#include <QObject>
#endif

#include "rviz/default_plugin/tools/selection_tool.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Float64MultiArray.h>

namespace multi_sensor_mapping {

class RegionSelectionTool : public rviz::SelectionTool {
  Q_OBJECT

 public:
  RegionSelectionTool();

  virtual ~RegionSelectionTool();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);

 protected:
  void ProcessSelectedRegion();

  /// @brief ros节点
  ros::NodeHandle nh_;
  /// @brief 区域marker发布器
  ros::Publisher pub_region_marker_;

  bool selecting_flag_ = false;
};

}  // namespace multi_sensor_mapping

#endif
