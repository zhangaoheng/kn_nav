// ============================================================
// 文件：gradient_descent_optimizer.h
// 模块：bspline_opt（B 样条轨迹优化）
// 职责：提供基于梯度下降（含 Armijo 线搜索）的无约束优化器
//       GradientDescentOptimizer，作为 L-BFGS 之外的可选求解器。
// 接口：通过函数指针 objfunDef 注入代价函数，调用 optimize()
//       迭代求解变量的最优值。
// ============================================================
#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <iostream>
#include <limits>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

// GradientDescentOptimizer：简单梯度下降求解器。
// 代价函数以 objfunDef 回调形式注入（返回代价值并回填梯度），
// 通过 force_return 标志支持外部提前终止（如检测到碰撞）。
// 停止条件：达到最大迭代/求值次数、梯度模长小于 min_grad 等。
class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);
  enum RESULT
  {
    FIND_MIN,
    FAILED,
    RETURN_BY_ORDER,
    REACH_MAX_ITERATION
  };

  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  {
    variable_num_ = v_num;
    objfun_ = objf;
    f_data_ = f_data;
  };

  void set_maxiter(int limit) { iter_limit_ = limit; }
  void set_maxeval(int limit) { invoke_limit_ = limit; }
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; }
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; }
  void set_min_grad(double min_grad) { min_grad_ = min_grad; }

  // optimize：从 x_init_optimal 出发迭代求解，返回最优状态与代价值；
  // 结果状态由 RESULT 枚举表示（找到极小点/失败/按外部指令返回/
  // 达到迭代上限）。
  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  int variable_num_{0};
  int iter_limit_{std::numeric_limits<int>::max()};
  int invoke_limit_{std::numeric_limits<int>::max()};
  double xtol_rel_;
  double xtol_abs_;
  double min_grad_;
  double time_limit_;
  void *f_data_;
  objfunDef objfun_;
};

#endif
