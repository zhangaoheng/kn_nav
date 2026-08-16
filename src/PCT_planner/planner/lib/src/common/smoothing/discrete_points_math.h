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
// 文件名: discrete_points_math.h
// 用途:   离散路径点几何计算: 由一系列二维点计算航向、累积弧长、
//         曲率及其导数(路径 profile), 供样条优化与轨迹生成使用
// 结构:   DiscretePointsMath 静态工具类
// ============================================================================

#pragma once

#include <utility>
#include <vector>

namespace common {

// 纯静态工具类: 输入 xy_points, 输出 headings/accumulated_s/kappas/dkappas
class DiscretePointsMath {
 public:
  DiscretePointsMath() = delete;

  static bool ComputePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);
};

}  // namespace common
