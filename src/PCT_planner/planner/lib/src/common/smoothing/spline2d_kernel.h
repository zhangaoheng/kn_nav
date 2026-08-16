/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

// ============================================================================
// 文件名: spline2d_kernel.h
// 用途:   二维样条的目标函数 (核) 构造器: 把各项代价 (导数惩罚、
//         正则化、参考线贴合、横/纵向偏移) 累加成二次型核矩阵与梯度,
//         供 QP 求解器最小化。
// 结构:   Spline2dKernel 类: 内部由 x/y 两个 Spline1dKernel 与协同核
//         cooperative_kernel_ 组装出最终的 kernel_matrix_ 和 gradient_。
// ============================================================================
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cinttypes>
#include <vector>

#include "common/smoothing/spline1d_kernel.h"

namespace common {
// 二维样条核: 各项 Add 接口累加二次代价, 最终经
// kernel_matrix()/gradient() 提供给求解器。
class Spline2dKernel {
 public:
  Spline2dKernel(const std::vector<double>& t_knots,
                 const uint32_t spline_order);

// 正则化: 对样条系数施加二次惩罚, 防止系数过大导致数值病态。
  void AddRegularization(const double regularized_param);
  void Add2dDerivativeKernelMatrix(const double weight);
  void Add2dSecondOrderDerivativeMatrix(const double weight);
  void Add2dThirdOrderDerivativeMatrix(const double weight);

  bool Add2dReferenceLineKernelMatrix(
      const std::vector<double>& t_coord,
      const vector_Eigen<Eigen::Vector2d>& ref_ft, const double weight);

  void Add2dLateralOffsetKernelMatrix(
      const std::vector<double>& t_coord,
      const vector_Eigen<Eigen::Vector2d>& ref_ft,
      const ::std::vector<double> ref_angle, const double weight);

  void Add2dLongitudinalOffsetKernelMatrix(
      const std::vector<double>& t_coord,
      const vector_Eigen<Eigen::Vector2d>& ref_ft,
      const ::std::vector<double> ref_angle, const double weight);

  size_t FindSegStartIndex(const double t) const;

  const Eigen::MatrixXd& kernel_matrix();
  const Eigen::MatrixXd& gradient();

 private:
  /* kernel only contains x's component */
  Spline1dKernel x_kernel_;
  /* kernel only contains y's component */
  Spline1dKernel y_kernel_;
  /* kernel contains both x and y's component */
  Eigen::MatrixXd cooperative_kernel_;

  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd gradient_;

  std::vector<double> t_knots_;

  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t total_params_ = 0;
};
}  // namespace common
