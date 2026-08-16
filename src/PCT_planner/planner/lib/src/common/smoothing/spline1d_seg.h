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
// 文件名: spline1d_seg.h
// 用途:   一维样条的单个分段: 用 PolynomialXd 表示本段多项式,
//         并预缓存 1~3 阶导数多项式, 提供求值接口
// 结构:   Spline1dSeg 类, 含 spline_func_ 及一至三阶导数多项式
// 依赖:   common/smoothing/polynomialxd.h
// ============================================================================

#pragma once

#include <vector>

#include "common/smoothing/polynomialxd.h"

// 一维样条段: 封装本段多项式及其导数, x 为本段内坐标
namespace common {

class Spline1dSeg {
 public:
  // order represents the highest order.
  explicit Spline1dSeg(const uint32_t order);
  explicit Spline1dSeg(const std::vector<double>& params);
  ~Spline1dSeg() = default;

// 设置本段多项式系数, 并同步更新各阶导数多项式
  void SetParams(const std::vector<double>& params);
  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

  const PolynomialXd& spline_func() const;
  const PolynomialXd& Derivative() const;
  const PolynomialXd& SecondOrderDerivative() const;
  const PolynomialXd& ThirdOrderDerivative() const;

 private:
  inline void SetSplineFunc(const PolynomialXd& spline_func);

  PolynomialXd spline_func_;
  PolynomialXd derivative_;
  PolynomialXd second_order_derivative_;
  PolynomialXd third_order_derivative_;
};
}  // namespace common
