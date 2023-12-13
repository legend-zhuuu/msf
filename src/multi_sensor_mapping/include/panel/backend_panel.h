#ifndef MSM_BACKEND_PANEL_H
#define MSM_BACKEND_PANEL_H

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
#include <qt5/QtWidgets/QTableWidget>
#include <qt5/QtWidgets/QVBoxLayout>
#include <qt5/QtWidgets/QtWidgets>

#include "multi_sensor_mapping/backend_panel_cmd.h"

namespace multi_sensor_mapping {

/**
 * @brief 后端优化Panel
 *
 */
class BackendPanel : public rviz::Panel {
  Q_OBJECT
 public:
  BackendPanel(QWidget* parent = 0);

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

  /**
   * @brief
   *
   */
  void SendPostProcessParam();

  /**
   * @brief 后端信息回调函数
   *
   * @param _panel_msg
   */
  void BackendPanelInfoHandler(
      const multi_sensor_mapping::backend_panel_cmd::ConstPtr& _panel_msg);

 protected Q_SLOTS:

  /**
   * @brief 自动闭环检测按钮回调函数
   *
   */
  void AutoLoopClosureDetectionButtonCallback();

  /**
   * @brief 优化回调函数
   *
   */
  void BackendLoopClosureDetectionButtonCallback();

  /**
   * @brief 确认回调函数
   *
   */
  void ConfirmButtonCallback();

  /**
   * @brief 匹配按钮回调函数
   *
   */
  void MatchButtonCallback();

  /**
   * @brief 后处理按钮回调
   *
   */
  void PostProcessButtonCallback();

  /**
   * @brief 保存地图回调函数
   *
   */
  void SaveButtonCallback();

  /**
   * @brief 更新后处理参数回调函数
   *
   */
  void UpdatePostParamButtonCallback();

  void DxSpinboxCallback(double _data);

  void DySpinboxCallback(double _data);

  void DzSpinboxCallback(double _data);

  void DrollSpinboxCallback(double _data);

  void DpitchSpinboxCallback(double _data);

  void DyawSpinboxCallback(double _data);

  void LCSpinboxCallback(double _data);

  void MapYawCallback(double _data);

  void SliceRadiusCallback(double _data);

  void SliceHeightCallback(double _data);

  void SliceThickCallback(double _data);

  /**
   * @brief TimeSpin
   */
  void TimeSpin();

 private:
  /// @brief ros节点
  ros::NodeHandle nh_;
  /// @brief panel命令发布器
  ros::Publisher pub_panel_command_;
  /// @brief panel信息订阅器
  ros::Subscriber sub_panel_info_;

  /// @brief 调整box
  QDoubleSpinBox* spin_box_dx_;
  QDoubleSpinBox* spin_box_dy_;
  QDoubleSpinBox* spin_box_dz_;
  QDoubleSpinBox* spin_box_droll_;
  QDoubleSpinBox* spin_box_dpitch_;
  QDoubleSpinBox* spin_box_dyaw_;
  /// @brief 闭环box
  QDoubleSpinBox* spin_box_loop_closure_id_;

  /// @brief 自动闭环检测按钮
  QPushButton* button_auto_loop_cloure_detection_;
  /// @brief 后端优化按钮
  QPushButton* button_backend_optimization_;
  /// @brief 确定按钮
  QPushButton* button_confirm_;
  /// @brief 配准按钮
  QPushButton* button_match_;

  /// @brief 保存按钮
  QPushButton* button_save_;

  /// @brief 地图yaw角调整
  QDoubleSpinBox* spin_box_map_yaw_;

  /// @brief 切片半径调整
  QDoubleSpinBox* spin_box_delta_slice_radius_;
  /// @brief 切片高度调整
  QDoubleSpinBox* spin_box_delta_slice_height_;
  /// @brief 切片厚度调整
  QDoubleSpinBox* spin_box_delta_slice_thickness_;
  /// @brief 地图后处理按钮
  QPushButton* button_post_process_;
  /// @brief 更新后处理参数
  QPushButton* button_update_post_param_;
  /// @brief 值改变冷却时间
  double value_changed_ignore_time_;
};
}  // namespace multi_sensor_mapping

#endif