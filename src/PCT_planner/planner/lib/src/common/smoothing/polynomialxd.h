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
// 文件名: polynomialxd.h
// 用途:   一元多项式 y = a0 + a1*x + a2*x^2 + ... + an*x^n 的表示与运算,
//         提供求值、求导、积分等操作, 是样条(spline)系数的基本载体
// 结构:   PolynomialXd 类, 内部仅保存系数向量 params_
// ============================================================================

#pragma once

#include <cinttypes>
#include <vector>

// 一元多项式: 支持求值、求导(DerivedFrom)与积分(IntegratedFrom)
namespace common {

/**
 * @class PolynomialXd
 * @brief y = a0 + a1*x + a2*x2^2 + ... + an*xn^n
 */
class PolynomialXd {
 public:
  PolynomialXd() = default;
  explicit PolynomialXd(const std::uint32_t order);
  explicit PolynomialXd(const std::vector<double>& params);

  ~PolynomialXd() = default;

  double operator()(const double x) const;
  double operator[](const std::uint32_t index) const;

  void SetParams(const std::vector<double>& params);

// 由当前多项式求导/积分生成新多项式(积分可指定常数项 intercept)
  static PolynomialXd DerivedFrom(const PolynomialXd& base);
  static PolynomialXd IntegratedFrom(const PolynomialXd& base,
                                     const double intercept = 0.0);

  std::uint32_t order() const;

  const std::vector<double>& params() const;

 private:
  std::vector<double> params_;
};
}  // namespace common
