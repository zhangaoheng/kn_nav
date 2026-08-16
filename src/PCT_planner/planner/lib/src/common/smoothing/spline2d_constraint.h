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
// 文件名: spline2d_constraint.h
// 用途:   二维样条的约束构造器: 把边界约束、点约束、光滑性约束与
//         运动学约束等整理为统一的线性不等式/等式 (约束矩阵 + 上下界),
//         供 QP 求解器使用。x/y 独立约束与 x/y 协同约束并存。
// 结构:   Spline2dConstraint 类: 内部组合 Spline1dConstraint (x/y 独立)
//         与 AffineConstraint (协同), 对外暴露合并后的矩阵与上下界。
// ============================================================================
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <utility>
#include <vector>

#include "common/smoothing/spline1d_constraint.h"

namespace common {
/**
 * @class Spline2dConstraint
 * @brief specify two-dimension polynomial spline constraint in matrix form
 */
// 二维样条约束集: 所有 Add 系列接口向内部约束矩阵追加行,
// 最终经 constraint_matrix()/lower_bound()/upper_bound() 供求解器读取。
class Spline2dConstraint {
 public:
  Spline2dConstraint() = default;

  Spline2dConstraint(const std::vector<double>& t_knots, const uint32_t order);

// 位置边界约束: 在给定节点处约束样条取值落在 [下界, 上界] 区间。
  bool Add2dBoundary(const std::vector<double>& t,
                     const vector_Eigen<Eigen::Vector2d>& lower_bound,
                     const vector_Eigen<Eigen::Vector2d>& upper_bound);

  // TODO(chengjie): add 2d station-lateral boundary constraint
  bool Add2dStationLateralBoundary(const std::vector<double>& t,
                                   const vector_Eigen<Eigen::Vector2d>& ref_xy,
                                   const std::vector<double>& ref_theta,
                                   const std::vector<double>& lon_tol,
                                   const std::vector<double>& lat_tol);

  bool Add2dDerivativeBoundary(const std::vector<double>& t,
                               const vector_Eigen<Eigen::Vector2d>& lower_bound,
                               const vector_Eigen<Eigen::Vector2d>& upper_bound);

  bool Add2dSecondDerivativeBoundary(
      const std::vector<double>& t,
      const vector_Eigen<Eigen::Vector2d>& lower_bound,
      const vector_Eigen<Eigen::Vector2d>& upper_bound);

  bool Add2dThirdDerivativeBoundary(
      const std::vector<double>& t,
      const vector_Eigen<Eigen::Vector2d>& lower_bound,
      const vector_Eigen<Eigen::Vector2d>& upper_bound);

  bool Add2dPointConstraint(const double t, const Eigen::Vector2d ft);
  bool Add2dPointDerivativeConstraint(const double t,
                                      const Eigen::Vector2d dft);
  bool Add2dPointSecondDerivativeConstraint(const double t,
                                            const Eigen::Vector2d ddft);
  bool Add2dPointThirdDerivativeConstraint(const double t,
                                           const Eigen::Vector2d dddft);

// 光滑性约束: 约束相邻分段在节点处连续 (另有导数/二阶/三阶导数光滑版本),
// 消除分段拼接处的跳变, 保证整条样条平滑。
  bool Add2dSmoothConstraint();
  bool Add2dDerivativeSmoothConstraint();
  bool Add2dSecondDerivativeSmoothConstraint();
  bool Add2dThirdDerivativeSmoothConstraint();

  bool AddPointAngleConstraint(const double t, const double angle);
  bool AddPointVelocityConstraint(const double t, const double fabs_v,
                                  const int gear, const double heading);
  bool AddPointAccelerationConstraint(const double t, const double a,
                                      const int gear, const double a_direction);

  /* velocity and acceleration contraint according to kinetic bicycle model */
// 运动学自行车模型约束: 由速度/加速度/前轮转角/航向角/轴距推算出
// 该点的期望位置、速度与加速度, 再分别对 x/y 施加位置、一阶导数与
// 二阶导数点约束, 使优化路径满足车辆运动学特性。
  bool AddPointKineticBicycleModelConstraint(const double t,
                                             const Eigen::Vector2d position,
                                             const double v, const double a,
                                             const double steer,
                                             const double heading,
                                             const double wheel_base);

  const AffineConstraint& x_affine_constraint();
  const AffineConstraint& y_affine_constraint();

  const Eigen::MatrixXd& constraint_matrix();
  const std::vector<double>& lower_bound();
  const std::vector<double>& upper_bound();

 private:
  size_t FindSegStartIndex(const double t) const;

  double SignDistance(const Eigen::Vector2d& ref_point,
                      const double ref_angle) const;

  std::vector<double> AffineCoef(const double angle, const double t) const;

 private:
  // independent x,y constraint
  Spline1dConstraint x_constraint_;
  Spline1dConstraint y_constraint_;

  // cooperative x,y constraint
  AffineConstraint coxy_constraint_;

  std::vector<double> t_knots_;
  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t total_param_;

  Eigen::MatrixXd constraint_matrix_;
  std::vector<double> lower_bound_;
  std::vector<double> upper_bound_;
};
}  // namespace common
