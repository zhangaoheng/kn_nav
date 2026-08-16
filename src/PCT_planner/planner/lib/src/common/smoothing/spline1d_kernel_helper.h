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
// 文件名: spline1d_kernel_helper.h
// 用途:   样条核矩阵计算助手(单例): 计算各阶导数惩罚对应的积分核矩阵,
//         并用哈希表缓存, 避免重复计算
// 结构:   Spline1dKernelHelper 单例类(Instance()), 内部 kernel_map_ 缓存
// ============================================================================

#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_map>

// 单例工具: 根据样条阶数与导数阶数计算核矩阵, 结果按参数缓存
namespace common {

class Spline1dKernelHelper {
 public:
  static Spline1dKernelHelper& Instance();

// 获取指定样条阶数、n 阶导数与积分长度下的核矩阵(带缓存)
  Eigen::MatrixXd Kernel(const uint32_t spline_order,
                         const uint32_t nth_derivative,
                         const double integral_length);

 private:
  Spline1dKernelHelper();

  Spline1dKernelHelper(const Spline1dKernelHelper&) = delete;

  Spline1dKernelHelper& operator=(const Spline1dKernelHelper&) = delete;

  Eigen::MatrixXd DerivativeKernel(const uint32_t num_of_params,
                                   const double accumulated_x);
  Eigen::MatrixXd SecondOrderDerivativeKernel(const uint32_t num_of_params,
                                              const double accumulated_x);
  Eigen::MatrixXd ThirdOrderDerivativeKernel(const uint32_t num_of_params,
                                             const double accumulated_x);

  void IntegratedTermMatrix(const uint32_t num_of_params,
                            const uint16_t derivative, const double x,
                            Eigen::MatrixXd* term_matrix) const;

  void CalculateFx(const uint32_t num_of_params);
  void CalculateDerivative(const uint32_t num_of_params);
  void CalculateSecondOrderDerivative(const uint32_t num_of_params);
  void CalculateThirdOrderDerivative(const uint32_t num_of_params);

  void BuildKernelMap();

 private:
  std::unordered_map<std::string, Eigen::MatrixXd> kernel_map_;

  Eigen::MatrixXd kernel_fx_;
  Eigen::MatrixXd kernel_derivative_;
  Eigen::MatrixXd kernel_second_order_derivative_;
  Eigen::MatrixXd kernel_third_order_derivative_;
};
}  // namespace common
