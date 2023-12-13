#include "panel/region_selection_tool.h"

#include "OGRE/OgreCamera.h"
#include "rviz/display_context.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/vector_property.h"
#include "rviz/selection/forwards.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include "rviz/viewport_mouse_event.h"

namespace multi_sensor_mapping {

RegionSelectionTool::RegionSelectionTool() {
  pub_region_marker_ =
      nh_.advertise<std_msgs::Float64MultiArray>("/region_info", 1);
}

RegionSelectionTool::~RegionSelectionTool() {}

int RegionSelectionTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  int flags = rviz::SelectionTool::processMouseEvent(event);

  // determine current selection mode
  if (event.alt()) {
    selecting_flag_ = false;
  } else {
    if (event.leftDown()) {
      selecting_flag_ = true;
    }
  }

  if (selecting_flag_) {
    if (event.leftUp()) {
      this->ProcessSelectedRegion();
    }
  }
  return flags;
}

int RegionSelectionTool::processKeyEvent(QKeyEvent* event,
                                         rviz::RenderPanel* panel) {
  //  if (event->type() == QKeyEvent::KeyPress) {
  //    if (event->key() == 'c' || event->key() == 'C') {
  //      ROS_INFO_STREAM_NAMED(
  //          "SelectedPointsPublisher::processKeyEvent",
  //          "Cleaning ALL previous selection (selected area and points).");
  //      rviz::SelectionManager* sel_manager = context_->getSelectionManager();
  //      rviz::M_Picked selection = sel_manager->getSelection();
  //      sel_manager->removeSelection(selection);
  //      visualization_msgs::Marker marker;
  //      // Set the frame ID and timestamp.  See the TF tutorials for
  //      information
  //      // on these.
  //      marker.header.frame_id =
  //      context_->getFixedFrame().toStdString().c_str(); marker.header.stamp =
  //      ros::Time::now(); marker.ns = "basic_shapes"; marker.id = 0;
  //      marker.type = visualization_msgs::Marker::CUBE;
  //      marker.action = visualization_msgs::Marker::DELETE;
  //      marker.lifetime = ros::Duration();

  //      selected_segment_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  //      accumulated_segment_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  //      bb_marker_pub_.publish(marker);
  //    } else if (event->key() == 'r' || event->key() == 'R') {
  //      ROS_INFO_STREAM_NAMED(
  //          "SelectedPointsPublisher.processKeyEvent",
  //          "Reusing the LAST selected area to find a NEW bounding box.");
  //      this->_processSelectedAreaAndFindPoints();
  //    } else if (event->key() == 'y' || event->key() == 'Y') {
  //      this->_publishAccumulatedPoints();
  //    } else if (event->key() == '+') {
  //      ROS_INFO_STREAM_NAMED(
  //          "SelectedPointsPublisher.processKeyEvent",
  //          "Adding the points to the accumulated point cloud. Removing them "
  //          "from the original point cloud. Clearing the LAST selected
  //          area.");
  //      rviz::SelectionManager* sel_manager = context_->getSelectionManager();
  //      rviz::M_Picked selection = sel_manager->getSelection();
  //      sel_manager->removeSelection(selection);
  //      visualization_msgs::Marker marker;
  //      // Set the frame ID and timestamp.  See the TF tutorials for
  //      information
  //      // on these.
  //      marker.header.frame_id =
  //      context_->getFixedFrame().toStdString().c_str(); marker.header.stamp =
  //      ros::Time::now(); marker.ns = "basic_shapes"; marker.id = 0;
  //      marker.type = visualization_msgs::Marker::CUBE;
  //      marker.action = visualization_msgs::Marker::DELETE;
  //      marker.lifetime = ros::Duration();
  //      bb_marker_pub_.publish(marker);

  //      // First remove the selected point of the original point cloud so that
  //      // they cannot be selected again:
  //      pcl::PointCloud<pcl::PointXYZRGB> temp_new_pc;
  //      extract_indices_filter_->setKeepOrganized(true);
  //      extract_indices_filter_->setNegative(true);
  //      temp_new_pc.header = this->current_pc_->header;
  //      extract_indices_filter_->filter(temp_new_pc);
  //      pcl::copyPointCloud(temp_new_pc, *this->current_pc_);

  //      sensor_msgs::PointCloud2 partial_pc_ros;
  //      pcl::toROSMsg(*this->current_pc_, partial_pc_ros);
  //      partial_pc_pub_.publish(partial_pc_ros);

  //      // Then I copy the that were selected before in the accumulated point
  //      // cloud (except if it is the first selected segment, then I copy the
  //      // whole dense point cloud)
  //      if (this->accumulated_segment_pc_->points.size() == 0) {
  //        pcl::copyPointCloud(*this->selected_segment_pc_,
  //                            *this->accumulated_segment_pc_);
  //        this->num_acc_points_ = this->num_selected_points_;
  //      } else {
  //        // Both are dense organized point clouds and the points were
  //        // selected_segment_pc_ has a not NaN value must be NaN in the
  //        // accumulated_segment_pc_ (and viceversa)
  //        for (unsigned int idx_selected = 0;
  //             idx_selected < this->selected_segment_pc_->points.size();
  //             idx_selected++) {
  //          if (pcl::isFinite(
  //                  this->selected_segment_pc_->points.at(idx_selected))) {
  //            this->accumulated_segment_pc_->points.at(idx_selected) =
  //                this->selected_segment_pc_->points.at(idx_selected);
  //          }
  //        }

  //        this->num_acc_points_ += this->num_selected_points_;
  //      }

  //      selected_segment_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  //      this->num_selected_points_ = 0;

  //      ROS_INFO_STREAM_NAMED(
  //          "SelectedPointsPublisher._processSelectedAreaAndFindPoints",
  //          "Number of accumulated points (not published): "
  //              << this->num_acc_points_);

  //      ROS_INFO_STREAM_NAMED(
  //          "SelectedPointsPublisher.processKeyEvent",
  //          "Select a new area and press '+' again to accumulate more points,
  //          or " "press 'y' to publish the accumulated point cloud.");
  //    }
  //  }
}

void RegionSelectionTool::ProcessSelectedRegion() {
  rviz::SelectionManager* sel_manager = context_->getSelectionManager();
  rviz::M_Picked selection = sel_manager->getSelection();
  rviz::PropertyTreeModel* model = sel_manager->getPropertyModel();
  int num_points = model->rowCount();

  if (num_points < 5) return;

  double max_x = -100000;
  double min_x = 100000;
  double max_y = -100000;
  double min_y = 100000;

  for (int i = 0; i < num_points; i++) {
    QModelIndex child_index = model->index(i, 0);
    rviz::Property* child = model->getProp(child_index);
    rviz::VectorProperty* subchild = (rviz::VectorProperty*)child->childAt(0);
    Ogre::Vector3 vec = subchild->getVector();

    if (vec.x > max_x) {
      max_x = vec.x;
    }
    if (vec.x < min_x) {
      min_x = vec.x;
    }
    if (vec.y > max_y) {
      max_y = vec.y;
    }
    if (vec.y < min_y) {
      min_y = vec.y;
    }
  }

  std_msgs::Float64MultiArray region_info_msg;
  region_info_msg.data.push_back(max_x);
  region_info_msg.data.push_back(min_x);
  region_info_msg.data.push_back(max_y);
  region_info_msg.data.push_back(min_y);

  pub_region_marker_.publish(region_info_msg);
}

}  // namespace multi_sensor_mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_sensor_mapping::RegionSelectionTool, rviz::Tool)
