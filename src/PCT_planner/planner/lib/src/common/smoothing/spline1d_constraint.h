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
// 文件名: spline1d_constraint.h
// 用途:   一维样条优化问题的约束构造器: 将采样点处的值/导数/边界/光滑性
//         等约束整理为矩阵形式(仿射约束), 供 QP 求解器使用
// 结构:   Spline1dConstraint 类, 内部为 AffineConstraint 与约束矩阵/上下界
// 依赖:   common/smoothing/affine_constraint.h
// ============================================================================

#pragma once

//#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <vector>

#include "common/smoothing/affine_constraint.h"

// 一维样条约束: 按采样点 x 所在区间定位段, 构造函数值/导数/光滑性约束行
namespace common {
/**
 * @class Spline1dConstraint
 * @brief specify one-dimension polynomial spline constraint in matrix form
 */
class Spline1dConstraint {
 public:
  Spline1dConstraint() = default;
  Spline1dConstraint(const std::vector<double>& x_knots, const uint32_t order);

// 直接追加一组完整约束(矩阵 + 上下界)
  bool AddConstraint(const Eigen::MatrixXd constraint,
                     const std::vector<double>& lower_bound,
                     const std::vector<double>& upper_bound);

// 在多个采样点添加函数值边界约束(下限/上限)
  bool AddBoundary(const std::vector<double>& x,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);

  bool AddDerivativeBoundary(const std::vector<double>& x,
                             const std::vector<double>& lower_bound,
                             const std::vector<double>& upper_bound);

  bool AddSecondDerivativeBoundary(const std::vector<double>& x,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound);

  bool AddThirdDerivativeBoundary(const std::vector<double>& x,
                                  const std::vector<double>& lower_bound,
                                  const std::vector<double>& upper_bound);

// 在单个点添加函数值或 1~3 阶导数的等式约束
  bool AddPointConstraint(const double x, const double fx);
  bool AddPointDerivativeConstraint(const double x, const double dfx);
  bool AddPointSecondDerivativeConstraint(const double x, const double ddfx);
  bool AddPointThirdDerivativeConstraint(const double x, const double dddfx);

// 添加段间光滑性约束(相邻段在节点处函数值及各阶导数连续)
  bool AddSmoothConstraint();
  bool AddDerivativeSmoothConstraint();
  bool AddSecondDerivativeSmoothConstraint();
  bool AddThirdDerivativeSmoothConstraint();

  const AffineConstraint& affine_constraint() const;
  const Eigen::MatrixXd& constraint_matrix() const;
  const std::vector<double>& lower_bound() const;
  const std::vector<double>& upper_bound() const;

 private:
// 定位采样点 x 所在样条段的起始索引
  size_t FindSegStartIndex(const double x) const;

 private:
  AffineConstraint affine_constraint_;
  std::vector<double> x_knots_;

  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t columns_;
};
}  // namespace common
