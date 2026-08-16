/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

namespace common {

double NormalizeAngle(const double angle);

double InterpolateAngle(const double a0, const double t0, const double a1,
                        const double t1, const double t);

int RandomInt(const int size);

double RandomDouble(const double lb, const double ub);

// 由位置一/二阶导数计算曲率 kappa, 分子为叉积、分母为速度模的三次方。
double Curvature(const double dx, const double d2x, const double dy,
                 const double d2y);

// 由位置一至三阶导数计算曲率对弧长的导数, 即曲率变化率。
double CurvatureDerivative(const double dx, const double d2x, const double d3x,
                           const double dy, const double d2y, const double d3y);
}  // namespace common
