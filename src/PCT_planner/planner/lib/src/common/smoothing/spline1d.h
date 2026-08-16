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
// 文件名: spline1d.h
// 用途:   一维分段多项式样条: 以 x_knots 划分区间, 每个区间用一段
//         Spline1dSeg 多项式表示, 提供求值及 1~3 阶导数
// 结构:   Spline1d 类, 内部含 std::vector<Spline1dSeg> 与节点 x_knots_
// 依赖:   common/smoothing/spline1d_seg.h
// ============================================================================

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "common/smoothing/spline1d_seg.h"

// 一维样条: 由多段 Spline1dSeg 拼接, 通过 FindSegStartIndex 定位查询点所在区间
namespace common {

class Spline1d {
 public:
  Spline1d() = default;
  Spline1d(const std::vector<double>& x_knots, const uint32_t order);

  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

// 用参数矩阵设置各段系数(param_matrix 每行对应一段)
  bool set_splines(const Eigen::MatrixXd& param_matrix, const uint32_t order);

  const std::vector<double>& x_knots() const;
  uint32_t spline_order() const;

  const std::vector<Spline1dSeg>& splines() const;

 private:
  size_t FindSegStartIndex(const double x) const;

 private:
  std::vector<Spline1dSeg> splines_;
  std::vector<double> x_knots_;
  uint32_t spline_order_;
};
}  // namespace common
