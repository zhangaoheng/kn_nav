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
// 文件名: spline2d_seg.h
// 用途:   二维样条的单个分段: x(t) 与 y(t) 各用一个多项式 (PolynomialXd)
//         表达, 并缓存其一至三阶导数多项式, 提供 t 处求值接口。
//         一条 Spline2d 由多个该分段按节点拼接而成。
// 结构:   Spline2dSeg 类, 内部持有 x/y 两个多项式及其导数多项式
// ============================================================================
#pragma once

#include <utility>
#include <vector>

#include "common/smoothing/polynomialxd.h"

namespace common {

// 二维样条分段: 阶数指参数个数 (非最高次数), 求值基于局部参数 t。
class Spline2dSeg {
 public:
  // order represent the number of parameters (not the highest order);
  explicit Spline2dSeg(const uint32_t order);
  explicit Spline2dSeg(const std::vector<double>& x_param,
                       const std::vector<double>& y_param);
  ~Spline2dSeg() = default;

  bool SetParams(const std::vector<double>& x_param,
                 const std::vector<double>& y_param);

  std::pair<double, double> operator()(const double t) const;
  double x(const double t) const;
  double y(const double t) const;
  double DerivativeX(const double t) const;
  double DerivativeY(const double t) const;
  double SecondDerivativeX(const double t) const;
  double SecondDerivativeY(const double t) const;
  double ThirdDerivativeX(const double t) const;
  double ThirdDerivativeY(const double t) const;

  const PolynomialXd& spline_func_x() const;
  const PolynomialXd& spline_func_y() const;
  const PolynomialXd& DerivativeX() const;
  const PolynomialXd& DerivativeY() const;
  const PolynomialXd& SecondDerivativeX() const;
  const PolynomialXd& SecondDerivativeY() const;
  const PolynomialXd& ThirdDerivativeX() const;
  const PolynomialXd& ThirdDerivativeY() const;

 private:
  PolynomialXd spline_func_x_;
  PolynomialXd spline_func_y_;
  PolynomialXd derivative_x_;
  PolynomialXd derivative_y_;
  PolynomialXd second_derivative_x_;
  PolynomialXd second_derivative_y_;
  PolynomialXd third_derivative_x_;
  PolynomialXd third_derivative_y_;
};
}  // namespace common
