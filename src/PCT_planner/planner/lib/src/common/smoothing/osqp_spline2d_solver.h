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
// 文件名: osqp_spline2d_solver.h
// 用途:   基于 OSQP 求解器的二维样条求解器 (Spline2dSolver 的派生实现):
//         x/y 两个方向各用一组一维样条表达, 平滑问题组装成二次规划 (QP)
//         后经 OsqpInterface 交给 osqp 求解, 用于路径的二维平滑。
// 结构:   OsqpSpline2dSolver 类, 仅声明接口, 实现见 osqp_spline2d_solver.cc
// 流程:   构造 -> mutable_kernel()/mutable_constraint() 配置目标与约束 ->
//         Solve() 求解 -> spline() 取回平滑样条
// ============================================================================
#pragma once

#include <vector>

#include "common/smoothing/spline2d_solver.h"

namespace common {

// OSQP 版二维样条求解器: 将基类 Spline2dSolver 的纯虚接口用 osqp 落地,
// 本头文件只声明接口, QP 组装与求解细节在 .cc 中实现。
class OsqpSpline2dSolver final : public Spline2dSolver {
 public:
  OsqpSpline2dSolver(const std::vector<double>& t_knots, const uint32_t order);

  ~OsqpSpline2dSolver() = default;

// 用新的节点序列与阶次重建样条/核/约束三对象, 使求解器可复用。
  void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

  // customize setup
  Spline2dConstraint* mutable_constraint() override;
  Spline2dKernel* mutable_kernel() override;
  Spline2d* mutable_spline() override;

  // solve
// 触发求解: 取出核矩阵 P、约束矩阵 A 与梯度 q, 组装上下界后调用
// OsqpInterface::Solve 求解 QP, 成功后把解写入样条系数并返回 true。
  bool Solve() override;

  // extract
  const Spline2d& spline() const override;
};
}  // namespace common
