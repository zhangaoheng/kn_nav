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
// 文件名: spline1d_kernel.h
// 用途:   一维样条优化问题的目标函数构造器(二次规划): 以二次型矩阵 H
//         与一次项 g 表示代价 0.5*x'*H*x + g'*x, 支持导数平滑、
//         正则化与参考线贴合等代价项
// 结构:   Spline1dKernel 类, 内部维护 kernel_matrix_ 与 gradient_
// 依赖:   common/base/type.h
// ============================================================================

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cinttypes>
#include <vector>

#include "common/base/type.h"

// 一维样条核: 将各代价项累加进二次型矩阵与梯度, 供 QP 求解
namespace common {

class Spline1dKernel {
 public:
  Spline1dKernel() = default;
  Spline1dKernel(const std::vector<double>& x_knots,
                 const uint32_t spline_order);

// 直接累加一个核矩阵(AddGradient 累加一次项梯度)
  void AddKernel(const Eigen::MatrixXd& kernel);
  void AddGradient(const Eigen::MatrixXd& gradient);

// 添加正则化项(对对角元素加 regularized_param, 提高数值稳定性)
  void AddRegularization(const double regularized_param);

// 添加 1~3 阶导数平滑代价(可分别调用, weight 为该项权重)
  void AddDerivativeKernelMatrix(const double weight);
  void AddSecondOrderDerivativeMatrix(const double weight);
  void AddThirdOrderDerivativeMatrix(const double weight);

// 添加参考线贴合代价: 让样条尽量接近参考点 (x_coord, ref_fx)
  bool AddReferenceLineKernelMatrix(const std::vector<double>& x_coord,
                                    const std::vector<double>& ref_fx,
                                    const double weight);

  bool AddDerivativeReferenceLineKernelMatrix(
      const std::vector<double>& x_coord, const std::vector<double>& ref_dfx,
      const double weight);

  const Eigen::MatrixXd& kernel_matrix() const;
  const Eigen::MatrixXd& gradient() const;

 private:
// 通用实现: 按导数阶数 n 构造平滑核矩阵并按权重缩放
  void AddNthDerivativekernelMatrix(const uint32_t n, const double weight);

  size_t FindSegStartIndex(const double x) const;

 private:
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd gradient_;

  std::vector<double> x_knots_;

  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t total_params_ = 0;
};
}  // namespace common
