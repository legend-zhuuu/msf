#ifndef MSM_SCAN_UNDISTORTION_H
#define MSM_SCAN_UNDISTORTION_H

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

/**
 * @brief The ScanUndistortion class 点云矫正器
 */
class ScanUndistortion {
 public:
  typedef std::shared_ptr<ScanUndistortion> Ptr;

  /**
   * @brief ScanUndistortion 构造函数 参数为矫正的配置参数
   * @param _correct_translation_distortion
   * @param _correct_rotation_distortion
   * @param _use_imu_orientation
   */
  ScanUndistortion(bool _correct_translation_distortion,
                   bool _correct_rotation_distortion,
                   bool _use_imu_orientation);

  /**
   * @brief InputImuData 添加IMU数据
   * @param imu_data
   */
  void InputImuData(const IMUData& _imu_data);

  /**
   * @brief InputWheelData 添加轮速计数据
   * @param odom_data
   */
  void InputWheelData(const OdometryData& _odom_data);

  /**
   * @brief UndistortScan 点云矫正(velodyne数据格式)
   * @param _scan_timestamp
   * @param _input_cloud
   * @param _output_cloud
   * @return
   */
  bool UndistortScan(const double _scan_timestamp,
                     VelRTPointCloudPtr _input_cloud,
                     VelRTPointCloudPtr _output_cloud);

  /**
   * @brief UndistortScan 点云矫正(RsLidar数据格式)
   * @param _scan_timestamp
   * @param _input_cloud
   * @param _output_cloud_
   * @return
   */
  bool UndistortScan(const double _scan_timestamp,
                     RsRTPointCloudPtr _input_cloud,
                     VelRTPointCloudPtr _output_cloud);

  /**
   * @brief UndoScan 不处理(为了统一接口)
   * @param _scan_timestamp
   * @param _input_cloud
   * @param _output_cloud
   * @return
   */
  bool UndoScan(const double _scan_timestamp, RsRTPointCloudPtr _input_cloud,
                VelRTPointCloudPtr _output_cloud);

 protected:
  /**
   * @brief CheckParameters 检测输入的参数
   */
  void CheckParameters();

  /**
   * @brief ResetParameters reset参数
   */
  void ResetParameters();

  /**
   * @brief DeskewPoint 对点进行矫正
   * @param _point
   * @param _time
   * @return
   */
  VelRTPoint DeskewPoint(VelRTPoint& _point, double _time);

  /**
   * @brief TrimImuData 删除队列无用的IMU信息
   */
  void TrimImuData();

  /**
   * @brief TrimOdomData 删除队列无用的odom信息
   */
  void TrimOdomData();

  /**
   * @brief DeskewInfo 去畸变数据
   */
  void DeskewInfo();

  /**
   * @brief ImuDeskewInfo 使用IMU数据准备去旋转畸变
   */
  void ImuDeskewInfo();

  /**
   * @brief OdomDeskewInfo 使用轮速计数据去除平移畸变
   */
  void OdomDeskewInfo();

  /**
   * @brief FindRotation 寻找point_time时刻的旋转
   * @param point_time
   * @param rot_x
   * @param rot_y
   * @param rot_z
   */
  void FindRotation(const double point_time, float& rot_x, float& rot_y,
                    float& rot_z);

  /**
   * @brief FindPosition 寻找point_time时刻的平移
   * @param point_time
   * @param pos_x
   * @param pos_y
   * @param pos_z
   */
  void FindPosition(const double point_time, float& pos_x, float& pos_y,
                    float& pos_z);

  /**
   * @brief FindPose 找到point_time时刻的位姿
   * @param _point_time
   * @return
   */
  Eigen::Affine3f FindPose(const double _point_time);

  /**
   * @brief FindRigid3 找到point_time时刻的位姿
   * @param _point_time
   * @return
   */
  Eigen::Matrix4f FindRigid3(const double _point_time);

 private:
  /// @brief 是否矫正平移运动失真
  bool correct_translation_distortion_;
  /// @brief 是否矫正旋转运动失真
  bool correct_rotation_distortion_;
  /// @brief 使用IMU数据对激光进行矫正
  bool use_imu_data_;
  /// @brief 使用轮速计数据对激光进行矫正
  bool use_odom_data_;
  /// @brief 使用IMU的orientation进行矫正
  bool use_imu_orientation_;

  /// @brief IMU数据队列
  std::deque<IMUData> imu_data_queue_;
  /// @brief odometry数据队列
  std::deque<OdometryData> odom_data_queue_;
  /// @brief 当前帧开始时间
  double scan_start_time_;
  /// @brief 当前帧结束时间
  double scan_end_time_;
  /// @brief 矫正数据的队列长度
  const int queue_length_ = 1000;
  ///  @brief odom矫正序列
  double* odom_deskew_time_ = new double[queue_length_];
  double* odom_deskew_pos_x_ = new double[queue_length_];
  double* odom_deskew_pos_y_ = new double[queue_length_];
  double* odom_deskew_pos_z_ = new double[queue_length_];
  /// @brief imu矫正序列
  double* imu_deskew_time_ = new double[queue_length_];
  double* imu_deskew_rot_x_ = new double[queue_length_];
  double* imu_deskew_rot_y_ = new double[queue_length_];
  double* imu_deskew_rot_z_ = new double[queue_length_];

  /// @brief imu矫正序列指针
  int imu_pointer_cur_;
  /// @brief odom矫正序列指针
  int odom_pointer_cur_;
  /// @brief 是否可以使用IMU数据去畸变
  bool imu_avaliable_flag_;
  /// @brief 是否可以使用odom数据去畸变
  bool odom_avaliable_flag_;

  /// @brief scan_start_time_时刻的位姿的逆
  Eigen::Affine3f trans_start_inverse_;

  /// @brief scan_start_time_时刻的位姿的逆
  Eigen::Matrix4f scan_start_pose_inv_;
};

}  // namespace multi_sensor_mapping

#endif
