/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

// ============================================================================
// 文件名: type.h
// 用途:   基础类型别名: 定义带 Eigen 对齐分配器的 std::vector 别名,
//         避免包含固定尺寸 Eigen 类型的 vector 出现内存对齐问题
// 结构:   vector_Eigen<T> 模板别名, 及其二维/三维特化别名
// 依赖:   Eigen
// ============================================================================

#pragma once

#include <vector>
#include <Eigen/Dense>

// 带 Eigen::aligned_allocator 的 vector 别名模板(存储 Eigen 固定尺寸类型时必需)
template<typename T>
using vector_Eigen = std::vector<T, Eigen::aligned_allocator<T>>;

using vector_Eigen2d = vector_Eigen<Eigen::Vector2d>;
using vector_Eigen3d = vector_Eigen<Eigen::Vector3d>;