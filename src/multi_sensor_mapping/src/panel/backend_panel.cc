#include "panel/backend_panel.h"

namespace multi_sensor_mapping {

BackendPanel::BackendPanel(QWidget* parent) : rviz::Panel(parent) {
  pub_panel_command_ = nh_.advertise<multi_sensor_mapping::backend_panel_cmd>(
      "/backend_panel_cmd", 1);
  sub_panel_info_ = nh_.subscribe<multi_sensor_mapping::backend_panel_cmd>(
      "/backend_info", 10, &BackendPanel::BackendPanelInfoHandler, this);
  value_changed_ignore_time_ = 0;

  QGridLayout* adjust_layout = new QGridLayout;

  adjust_layout->addWidget(new QLabel("Adjust"), 0, 0);
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

  adjust_layout->addWidget(new QLabel("  LC ID  "), 0, 2);
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

  spin_box_loop_closure_id_ = new QDoubleSpinBox;
  spin_box_loop_closure_id_->setValue(0);
  spin_box_loop_closure_id_->setSingleStep(1);
  spin_box_loop_closure_id_->setMaximum(1000);
  spin_box_loop_closure_id_->setMinimum(0);

  adjust_layout->addWidget(spin_box_loop_closure_id_, 0, 3);
  adjust_layout->addWidget(spin_box_droll_, 1, 3);
  adjust_layout->addWidget(spin_box_dpitch_, 2, 3);
  adjust_layout->addWidget(spin_box_dyaw_, 3, 3);

  button_confirm_ = new QPushButton("Confirm");
  button_match_ = new QPushButton("Match");
  adjust_layout->addWidget(button_match_, 4, 2);
  adjust_layout->addWidget(button_confirm_, 4, 3);

  button_auto_loop_cloure_detection_ = new QPushButton("Auto-LCD");
  button_backend_optimization_ = new QPushButton("Optimization");
  button_save_ = new QPushButton("Save");

  QGridLayout* backend_layout = new QGridLayout;
  backend_layout->addWidget(new QLabel("Backend"), 0, 0);
  backend_layout->addWidget(button_auto_loop_cloure_detection_, 1, 0);
  backend_layout->addWidget(button_backend_optimization_, 1, 1);
  backend_layout->addWidget(button_save_, 1, 2);

  spin_box_map_yaw_ = new QDoubleSpinBox;
  spin_box_map_yaw_->setValue(0);
  spin_box_map_yaw_->setSingleStep(0.5);
  spin_box_map_yaw_->setMaximum(180);
  spin_box_map_yaw_->setMinimum(-180);
  spin_box_delta_slice_radius_ = new QDoubleSpinBox;
  spin_box_delta_slice_radius_->setValue(0);
  spin_box_delta_slice_radius_->setSingleStep(1);
  spin_box_delta_slice_radius_->setMaximum(100);
  spin_box_delta_slice_radius_->setMinimum(-100);
  spin_box_delta_slice_height_ = new QDoubleSpinBox;
  spin_box_delta_slice_height_->setValue(0);
  spin_box_delta_slice_height_->setSingleStep(0.2);
  spin_box_delta_slice_height_->setMaximum(20);
  spin_box_delta_slice_height_->setMinimum(-20);
  spin_box_delta_slice_thickness_ = new QDoubleSpinBox;
  spin_box_delta_slice_thickness_->setValue(0);
  spin_box_delta_slice_thickness_->setSingleStep(0.1);
  spin_box_delta_slice_thickness_->setMaximum(10);
  spin_box_delta_slice_thickness_->setMinimum(-10);

  button_post_process_ = new QPushButton("Post Process");
  button_update_post_param_ = new QPushButton("Update Session");

  QGridLayout* post_process_layout = new QGridLayout;
  post_process_layout->addWidget(new QLabel("Post Process"), 0, 0);
  post_process_layout->addWidget(new QLabel("radius (m)"), 1, 0);
  post_process_layout->addWidget(new QLabel("height (m)"), 2, 0);
  post_process_layout->addWidget(new QLabel("thick  (m)"), 3, 0);
  post_process_layout->addWidget(new QLabel("yaw  (rad)"), 4, 0);

  post_process_layout->addWidget(spin_box_delta_slice_radius_, 1, 1);
  post_process_layout->addWidget(spin_box_delta_slice_height_, 2, 1);
  post_process_layout->addWidget(spin_box_delta_slice_thickness_, 3, 1);
  post_process_layout->addWidget(spin_box_map_yaw_, 4, 1);

  post_process_layout->addWidget(button_post_process_, 1, 2);
  post_process_layout->addWidget(button_update_post_param_, 2, 2);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(adjust_layout);
  layout->addLayout(backend_layout);
  layout->addLayout(post_process_layout);

  setLayout(layout);

  connect(button_confirm_, SIGNAL(clicked()), this,
          SLOT(ConfirmButtonCallback()));
  connect(button_match_, SIGNAL(clicked()), this, SLOT(MatchButtonCallback()));

  connect(button_auto_loop_cloure_detection_, SIGNAL(clicked()), this,
          SLOT(AutoLoopClosureDetectionButtonCallback()));
  connect(button_backend_optimization_, SIGNAL(clicked()), this,
          SLOT(BackendLoopClosureDetectionButtonCallback()));
  connect(button_save_, SIGNAL(clicked()), this, SLOT(SaveButtonCallback()));

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
  connect(spin_box_loop_closure_id_, SIGNAL(valueChanged(double)), this,
          SLOT(LCSpinboxCallback(double)));

  connect(spin_box_delta_slice_radius_, SIGNAL(valueChanged(double)), this,
          SLOT(SliceRadiusCallback(double)));
  connect(spin_box_delta_slice_height_, SIGNAL(valueChanged(double)), this,
          SLOT(SliceHeightCallback(double)));
  connect(spin_box_delta_slice_thickness_, SIGNAL(valueChanged(double)), this,
          SLOT(SliceThickCallback(double)));
  connect(spin_box_map_yaw_, SIGNAL(valueChanged(double)), this,
          SLOT(MapYawCallback(double)));

  connect(button_post_process_, SIGNAL(clicked()), this,
          SLOT(PostProcessButtonCallback()));
  connect(button_update_post_param_, SIGNAL(clicked()), this,
          SLOT(UpdatePostParamButtonCallback()));

  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(TimeSpin()));
  output_timer->start(100);
}

void BackendPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void BackendPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

void BackendPanel::SendAdjustParam() {
  double ignore_time = ros::Time::now().toSec() - value_changed_ignore_time_;
  if (ignore_time < 0.2) return;

  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "ADJ";
  cmd_msg.x = spin_box_dx_->value();
  cmd_msg.y = spin_box_dy_->value();
  cmd_msg.z = spin_box_dz_->value();
  cmd_msg.roll = spin_box_droll_->value();
  cmd_msg.pitch = spin_box_dpitch_->value();
  cmd_msg.yaw = spin_box_dyaw_->value();
  cmd_msg.id = (int)spin_box_loop_closure_id_->value();

  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::SendPostProcessParam() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "SLICEADJ";
  cmd_msg.slice_radius = spin_box_delta_slice_radius_->value();
  cmd_msg.slice_height = spin_box_delta_slice_height_->value();
  cmd_msg.slice_thick = spin_box_delta_slice_thickness_->value();

  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::BackendPanelInfoHandler(
    const multi_sensor_mapping::backend_panel_cmd::ConstPtr& _panel_msg) {
  if (_panel_msg->cmd == "LOCAL") {
    value_changed_ignore_time_ = ros::Time::now().toSec();
    spin_box_dx_->setValue(_panel_msg->x);
    spin_box_dy_->setValue(_panel_msg->y);
    spin_box_dz_->setValue(_panel_msg->z);
    spin_box_droll_->setValue(_panel_msg->roll);
    spin_box_dpitch_->setValue(_panel_msg->pitch);
    spin_box_dyaw_->setValue(_panel_msg->yaw);
  }
}

void BackendPanel::UpdatePostParamButtonCallback() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "UPDATESESSION";
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::DxSpinboxCallback(double _data) { SendAdjustParam(); }

void BackendPanel::DySpinboxCallback(double _data) { SendAdjustParam(); }

void BackendPanel::DzSpinboxCallback(double _data) { SendAdjustParam(); }

void BackendPanel::DrollSpinboxCallback(double _data) { SendAdjustParam(); }

void BackendPanel::DpitchSpinboxCallback(double _data) { SendAdjustParam(); }

void BackendPanel::DyawSpinboxCallback(double _data) { SendAdjustParam(); }

void BackendPanel::SliceRadiusCallback(double _data) { SendPostProcessParam(); }

void BackendPanel::SliceHeightCallback(double _data) { SendPostProcessParam(); }

void BackendPanel::SliceThickCallback(double _data) { SendPostProcessParam(); }

void BackendPanel::MapYawCallback(double _data) {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "MAPADJ";
  cmd_msg.yaw = spin_box_map_yaw_->value();
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::LCSpinboxCallback(double _data) {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "MANUALLCD";
  cmd_msg.id = (int)_data;
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::AutoLoopClosureDetectionButtonCallback() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "AUTOLCD";
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::BackendLoopClosureDetectionButtonCallback() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "OPT";
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::ConfirmButtonCallback() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "CONFIRM";
  cmd_msg.id = (int)spin_box_loop_closure_id_->value();
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::MatchButtonCallback() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "MATCH";
  cmd_msg.x = spin_box_dx_->value();
  cmd_msg.y = spin_box_dy_->value();
  cmd_msg.z = spin_box_dz_->value();
  cmd_msg.roll = spin_box_droll_->value();
  cmd_msg.pitch = spin_box_dpitch_->value();
  cmd_msg.yaw = spin_box_dyaw_->value();
  cmd_msg.id = (int)spin_box_loop_closure_id_->value();
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::SaveButtonCallback() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "SAVE";
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::PostProcessButtonCallback() {
  multi_sensor_mapping::backend_panel_cmd cmd_msg;
  cmd_msg.cmd = "POST";
  pub_panel_command_.publish(cmd_msg);
}

void BackendPanel::TimeSpin() { ros::spinOnce(); }

}  // namespace multi_sensor_mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_sensor_mapping::BackendPanel, rviz::Panel)
