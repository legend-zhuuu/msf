#include "panel/adjust_cloud_panel.h"

namespace multi_sensor_mapping {

AdjustCloudPanel::AdjustCloudPanel(QWidget* parent) : rviz::Panel(parent) {
  pub_panel_command_ =
      nh_.advertise<multi_sensor_mapping::adjust_cloud_panel_cmd>(
          "/adjust_panel_cmd", 1);

  QGridLayout* adjust_layout = new QGridLayout;

  adjust_layout->addWidget(new QLabel("Adjust Cloud "), 0, 0);
  adjust_layout->addWidget(new QLabel("  x (m)"), 1, 0);
  adjust_layout->addWidget(new QLabel("  y (m)"), 2, 0);
  adjust_layout->addWidget(new QLabel("  z (m)"), 3, 0);

  spin_box_dx_ = new QDoubleSpinBox;
  spin_box_dx_->setValue(0);
  spin_box_dx_->setSingleStep(0.1);
  spin_box_dx_->setMinimum(-100);
  spin_box_dx_->setMaximum(100);
  spin_box_dy_ = new QDoubleSpinBox;
  spin_box_dy_->setValue(0);
  spin_box_dy_->setSingleStep(0.1);
  spin_box_dy_->setMinimum(-100);
  spin_box_dy_->setMaximum(100);
  spin_box_dz_ = new QDoubleSpinBox;
  spin_box_dz_->setValue(0);
  spin_box_dz_->setSingleStep(0.1);
  spin_box_dz_->setMinimum(-100);
  spin_box_dz_->setMaximum(100);

  adjust_layout->addWidget(spin_box_dx_, 1, 1);
  adjust_layout->addWidget(spin_box_dy_, 2, 1);
  adjust_layout->addWidget(spin_box_dz_, 3, 1);

  adjust_layout->addWidget(new QLabel(" roll (°)"), 1, 2);
  adjust_layout->addWidget(new QLabel(" pitch(°)"), 2, 2);
  adjust_layout->addWidget(new QLabel("  yaw (°)"), 3, 2);

  spin_box_droll_ = new QDoubleSpinBox;
  spin_box_droll_->setValue(0);
  spin_box_droll_->setSingleStep(0.5);
  spin_box_droll_->setMaximum(180);
  spin_box_droll_->setMinimum(-180);
  spin_box_dpitch_ = new QDoubleSpinBox;
  spin_box_dpitch_->setValue(0);
  spin_box_dpitch_->setSingleStep(0.5);
  spin_box_dpitch_->setMaximum(180);
  spin_box_dpitch_->setMinimum(-180);
  spin_box_dyaw_ = new QDoubleSpinBox;
  spin_box_dyaw_->setValue(0);
  spin_box_dyaw_->setSingleStep(0.5);
  spin_box_dyaw_->setMaximum(180);
  spin_box_dyaw_->setMinimum(-180);

  button_confirm_ = new QPushButton("Confirm");

  adjust_layout->addWidget(button_confirm_, 0, 3);
  adjust_layout->addWidget(spin_box_droll_, 1, 3);
  adjust_layout->addWidget(spin_box_dpitch_, 2, 3);
  adjust_layout->addWidget(spin_box_dyaw_, 3, 3);

  QGridLayout* object_layout = new QGridLayout;
  object_layout->addWidget(new QLabel("Object Removal"), 0, 0);

  button_ground_extraction_ = new QPushButton("Ground Extract");
  button_selected_delete_ = new QPushButton("Delete");
  button_cancel_ = new QPushButton("Cancel");
  button_refresh_ = new QPushButton("Refresh");
  object_layout->addWidget(button_ground_extraction_, 0, 1);
  object_layout->addWidget(button_selected_delete_, 0, 2);
  object_layout->addWidget(button_cancel_, 0, 3);
  object_layout->addWidget(button_refresh_, 1, 1);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(object_layout);
  layout->addLayout(adjust_layout);

  setLayout(layout);

  connect(button_confirm_, SIGNAL(clicked()), this,
          SLOT(ConfirmButtonCallback()));
  connect(button_ground_extraction_, SIGNAL(clicked()), this,
          SLOT(GroundExtractionButtonCallback()));
  connect(button_selected_delete_, SIGNAL(clicked()), this,
          SLOT(SelectedDeleteButtonCallback()));
  connect(button_cancel_, SIGNAL(clicked()), this,
          SLOT(CancelButtonCallback()));
  connect(button_refresh_, SIGNAL(clicked()), this,
          SLOT(RefreshButtonCallback()));

  connect(spin_box_dx_, SIGNAL(valueChanged(double)), this,
          SLOT(DxSpinboxCallback(double)));
  connect(spin_box_dy_, SIGNAL(valueChanged(double)), this,
          SLOT(DySpinboxCallback(double)));
  connect(spin_box_dz_, SIGNAL(valueChanged(double)), this,
          SLOT(DzSpinboxCallback(double)));
  connect(spin_box_droll_, SIGNAL(valueChanged(double)), this,
          SLOT(DrollSpinboxCallback(double)));
  connect(spin_box_dpitch_, SIGNAL(valueChanged(double)), this,
          SLOT(DpitchSpinboxCallback(double)));
  connect(spin_box_dyaw_, SIGNAL(valueChanged(double)), this,
          SLOT(DyawSpinboxCallback(double)));

  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(TimeSpin()));
  output_timer->start(100);
}

void AdjustCloudPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void AdjustCloudPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

void AdjustCloudPanel::SendAdjustParam() {
  multi_sensor_mapping::adjust_cloud_panel_cmd cmd_msg;
  cmd_msg.cmd = "ADJ";
  cmd_msg.x = spin_box_dx_->value();
  cmd_msg.y = spin_box_dy_->value();
  cmd_msg.z = spin_box_dz_->value();
  cmd_msg.roll = spin_box_droll_->value();
  cmd_msg.pitch = spin_box_dpitch_->value();
  cmd_msg.yaw = spin_box_dyaw_->value();

  pub_panel_command_.publish(cmd_msg);
}

void AdjustCloudPanel::ConfirmButtonCallback() {
  multi_sensor_mapping::adjust_cloud_panel_cmd cmd_msg;
  cmd_msg.cmd = "CONFIRM";

  pub_panel_command_.publish(cmd_msg);
}

void AdjustCloudPanel::DxSpinboxCallback(double _data) { SendAdjustParam(); }

void AdjustCloudPanel::DySpinboxCallback(double _data) { SendAdjustParam(); }

void AdjustCloudPanel::DzSpinboxCallback(double _data) { SendAdjustParam(); }

void AdjustCloudPanel::DrollSpinboxCallback(double _data) { SendAdjustParam(); }

void AdjustCloudPanel::DpitchSpinboxCallback(double _data) {
  SendAdjustParam();
}

void AdjustCloudPanel::DyawSpinboxCallback(double _data) { SendAdjustParam(); }

void AdjustCloudPanel::GroundExtractionButtonCallback() {
  multi_sensor_mapping::adjust_cloud_panel_cmd cmd_msg;
  cmd_msg.cmd = "GROUND_EXTRACTION";

  pub_panel_command_.publish(cmd_msg);
}

void AdjustCloudPanel::SelectedDeleteButtonCallback() {
  multi_sensor_mapping::adjust_cloud_panel_cmd cmd_msg;
  cmd_msg.cmd = "SELECTED_DELETE";

  pub_panel_command_.publish(cmd_msg);
}

void AdjustCloudPanel::CancelButtonCallback() {
  multi_sensor_mapping::adjust_cloud_panel_cmd cmd_msg;
  cmd_msg.cmd = "CANCEL";

  pub_panel_command_.publish(cmd_msg);
}

void AdjustCloudPanel::RefreshButtonCallback() {
  multi_sensor_mapping::adjust_cloud_panel_cmd cmd_msg;
  cmd_msg.cmd = "REFRESH";

  pub_panel_command_.publish(cmd_msg);
}

void AdjustCloudPanel::TimeSpin() { ros::spinOnce(); }

}  // namespace multi_sensor_mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_sensor_mapping::AdjustCloudPanel, rviz::Panel)
