#ifndef MSM_ADJUST_CLOUD_PANEL_H
#define MSM_ADJUST_CLOUD_PANEL_H

#include <qt5/QtWidgets/qcombobox.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <tf/tf.h>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QTimer>
#include <qt5/QtGui/QPainter>
#include <qt5/QtWidgets/QCheckBox>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QLayout>
#include <qt5/QtWidgets/QLineEdit>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QVBoxLayout>
#include <qt5/QtWidgets/QtWidgets>

#include "multi_sensor_mapping/adjust_cloud_panel_cmd.h"

namespace multi_sensor_mapping {

class AdjustCloudPanel : public rviz::Panel {
  Q_OBJECT
 public:
  AdjustCloudPanel(QWidget* parent = 0);

  /**
   * @brief load
   * @param config
   */
  virtual void load(const rviz::Config& config);

  /**
   * @brief save
   * @param config
   */
  virtual void save(rviz::Config config) const;

  /**
   * @brief SendAdjustParam
   */
  void SendAdjustParam();

 protected Q_SLOTS:

  void ConfirmButtonCallback();

  void DxSpinboxCallback(double _data);

  void DySpinboxCallback(double _data);

  void DzSpinboxCallback(double _data);

  void DrollSpinboxCallback(double _data);

  void DpitchSpinboxCallback(double _data);

  void DyawSpinboxCallback(double _data);

  void GroundExtractionButtonCallback();

  void SelectedDeleteButtonCallback();

  void CancelButtonCallback();

  void RefreshButtonCallback();

  /**
   * @brief TimeSpin
   */
  void TimeSpin();

 private:
  /// @brief ros节点
  ros::NodeHandle nh_;
  /// @brief panel命令发布器
  ros::Publisher pub_panel_command_;

  QDoubleSpinBox* spin_box_dx_;
  QDoubleSpinBox* spin_box_dy_;
  QDoubleSpinBox* spin_box_dz_;
  QDoubleSpinBox* spin_box_droll_;
  QDoubleSpinBox* spin_box_dpitch_;
  QDoubleSpinBox* spin_box_dyaw_;

  QPushButton* button_confirm_;

  QPushButton* button_ground_extraction_;
  QPushButton* button_selected_delete_;
  QPushButton* button_cancel_;
  QPushButton* button_refresh_;
};

}  // namespace multi_sensor_mapping

#endif
