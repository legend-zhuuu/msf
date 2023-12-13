#ifndef MSM_MARGINALIZATION_FACTOR_H
#define MSM_MARGINALIZATION_FACTOR_H

#include <ceres/ceres.h>
#include <pthread.h>

#include <unordered_map>

namespace multi_sensor_mapping {

const int NUM_THREADS = 4;

enum ResidualType {
  RType_Pose = 0,
  RType_IMU,
  RType_Bias,
  RType_Gravity,
  RType_LiDAR,
  RType_LiDAR_Relative,
  RType_LiDAROpt,
  RType_PreIntegration,
  RType_GlobalVelocity,
  RType_LocalVelocity,
  RType_Local6DoFVel,
  RType_Image,
  RType_Epipolar,
  RType_Prior
};

const std::string ResidualTypeStr[] = {
    "Pose           ",  //
    "IMU            ",  //
    "Bias           ",  //
    "Gravity        ",  //
    "LiDAR          ",  //
    "LiDAR_Relative ",  //
    "LiDAROptMap    ",  //
    "PreIntegration ",  //
    "GlobalVelocity ",  //
    "LocalVelocity  ",  //
    "Local6DoFVel   ",  //
    "Image          ",  //
    "Epipolar       ",  //
    "Prior          "   //
};

/**
 * @brief 残差因子的通用形式
 *
 */
struct ResidualBlockInfo {
  ResidualBlockInfo(ResidualType _residual_type,
                    ceres::CostFunction *_cost_function,
                    ceres::LossFunction *_loss_function,
                    std::vector<double *> _parameter_blocks,
                    std::vector<int> _drop_set)
      : residual_type(_residual_type),
        cost_function(_cost_function),
        loss_function(_loss_function),
        parameter_blocks(_parameter_blocks),
        drop_set(_drop_set) {}

  /// @brief  调用cost_function的evaluate函数计算残差和雅克比，作为线性化点
  void Evaluate();

  int LocalSize(int size) { return size == 4 ? 3 : size; }

  /// @brief 残差类型
  ResidualType residual_type;

  ceres::CostFunction *cost_function;
  ceres::LossFunction *loss_function;
  /// @brief 残差因子涉及的优化变量
  std::vector<double *> parameter_blocks;
  /// @brief 待边缘化的优化变量id
  std::vector<int> drop_set;

  /// @brief  Evaluate() 得到的雅克比和残差
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      jacobians;
  /// @brief 残差
  Eigen::VectorXd residuals;
};

struct ThreadsStruct {
  std::vector<ResidualBlockInfo *> sub_factors;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  std::unordered_map<long, int> parameter_block_size;  // parameter size
  std::unordered_map<long, int> parameter_block_idx;   // position in H matrix
};

// 该类中 unordered_map 的索引 long 均为 ResidualBlockInfo::parameter_blocks
// 的参数块地址
/**
 * @brief 边缘化信息
 *
 */
class MarginalizationInfo {
 public:
  typedef std::shared_ptr<MarginalizationInfo> Ptr;

  ~MarginalizationInfo();

  int LocalSize(int size) const;

  // 边缘化涉及的残差因子，统计 parameter_block_size, parameter_block_idx
  void AddResidualBlockInfo(ResidualBlockInfo *residual_block_info);

  // [1] 调用ResidualBlockInfo::Evaluate函数, 计算雅克比和残差
  // [2] 使用ResidualBlockInfo的参数初始化本类的parameter_block_data
  void PreMarginalize();

  // 开启多线程构建H和b，做舒尔消元，从消元后的H和b中恢复出雅克比和残差
  bool Marginalize();

  // 取出此次得到的先验参数,保存至 keep_* 参数中
  std::vector<double *> GetParameterBlocks(
      std::unordered_map<long, double *> &addr_shift);

  // 取出此次得到的先验参数,保存至 keep_* 参数中(不需要改变内存地址)
  std::vector<double *> GetParameterBlocks();

  // 所有观测项
  std::vector<ResidualBlockInfo *> factors;

  // m: 待边缘化的参数的总 localsize
  // n: 非边缘化参数的总 localsize
  int m, n;

  // 参数块地址，参数块的 globalsize
  std::unordered_map<long, int> parameter_block_size;

  // case1: <待边缘化的优化变量内存地址, 该参数在Ｘ中的位置>
  // case2: <非边缘化的变量内存地址, 该参数在Ｘ中的位置>
  // (X为HX=b中所有参数堆叠的值)
  std::unordered_map<long, int> parameter_block_idx;

  // 参数块地址(所有factor的)，参数块线性化点X0(本类新申请的内存)
  std::unordered_map<long, double *> parameter_block_data;

  // 保留参数块的global size
  std::vector<int> keep_block_size;
  // 保留参数块在H矩阵的索引
  std::vector<int> keep_block_idx;
  // 保留参数块的值(本类自己new的内存 parameter_block_data)
  std::vector<double *> keep_block_data;

  Eigen::MatrixXd linearized_jacobians;
  Eigen::VectorXd linearized_residuals;
  const double eps = 1e-15;
};

/**
 * @brief 边缘化因子
 *
 */
class MarginalizationFactor : public ceres::CostFunction {
 public:
  // 设置优化变量的globalsize和残差r*的维度
  MarginalizationFactor(const MarginalizationInfo::Ptr &_marginalization_info);
  // 利用配对的优化变量和残差计算雅克比
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;

  MarginalizationInfo::Ptr marginalization_info;
};

}  // namespace multi_sensor_mapping

#endif