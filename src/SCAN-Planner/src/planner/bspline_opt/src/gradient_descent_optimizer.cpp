// ============================================================
// 文件：gradient_descent_optimizer.cpp
// 模块：bspline_opt（B 样条轨迹优化）
// 职责：实现简单梯度下降优化器。
// 算法：每次迭代沿负梯度方向取步长 alpha（由相邻两步的
//       s=y 信息估计，类似 Barzilai-Borwein 步长），再用
//       Armijo 条件做线搜索收缩步长，保证目标函数充分下降；
//       以梯度模长或迭代次数作为停止条件。
// ============================================================
#include <bspline_opt/gradient_descent_optimizer.h>

#define RESET "\033[0m"
#define RED "\033[31m"

GradientDescentOptimizer::RESULT
// optimize：梯度下降主循环。
// 1) 校验 min_grad_/iter_limit_ 参数是否合理；
// 2) 首次迭代的步长 alpha0 按“首步位移不超过 0.1m”缩放；
// 3) 交替更新 x_k / x_kp1，alpha 由 s·y / y·y 估计（类
//    Barzilai-Borwein 步长），并以 Armijo 条件（充分下降）
//    做线搜索收缩步长；
// 4) 梯度模长小于 min_grad_ 时视为收敛（FIND_MIN）；
//    代价函数置 force_return 时按外部指令返回（RETURN_BY_ORDER）。
GradientDescentOptimizer::optimize(Eigen::VectorXd &x_init_optimal, double &opt_f)
{
    if (min_grad_ < 1e-10)
    {
        cout << RED << "min_grad_ is invalid:" << min_grad_ << RESET << endl;
        return FAILED;
    }
    if (iter_limit_ <= 2)
    {
        cout << RED << "iter_limit_ is invalid:" << iter_limit_ << RESET << endl;
        return FAILED;
    }

    void *f_data = f_data_;
    int iter = 2;
    int invoke_count = 2;
    bool force_return;
    Eigen::VectorXd x_k(x_init_optimal), x_kp1(x_init_optimal.rows());
    double cost_k, cost_kp1, cost_min;
    Eigen::VectorXd grad_k(x_init_optimal.rows()), grad_kp1(x_init_optimal.rows());

    cost_k = objfun_(x_k, grad_k, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    cost_min = cost_k;
    double max_grad = max(abs(grad_k.maxCoeff()), abs(grad_k.minCoeff()));
    constexpr double MAX_MOVEMENT_AT_FIRST_ITERATION = 0.1; // meter
    double alpha0 = max_grad < MAX_MOVEMENT_AT_FIRST_ITERATION ? 1.0 : (MAX_MOVEMENT_AT_FIRST_ITERATION / max_grad);
    x_kp1 = x_k - alpha0 * grad_k;
    cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    if (cost_min > cost_kp1)
        cost_min = cost_kp1;

    /*** start iteration ***/
    while (++iter <= iter_limit_ && invoke_count <= invoke_limit_)
    {
        Eigen::VectorXd s = x_kp1 - x_k;
        Eigen::VectorXd y = grad_kp1 - grad_k;
        double alpha = s.dot(y) / y.dot(y);
        if (isnan(alpha) || isinf(alpha))
        {
            cout << RED << "step size invalid! alpha=" << alpha << RESET << endl;
            return FAILED;
        }

        if (iter % 2) // to avoid copying operations
        {
            do
            {
                x_k = x_kp1 - alpha * grad_kp1;
                cost_k = objfun_(x_k, grad_k, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                alpha *= 0.5;
            } while (cost_k > cost_kp1 - 1e-4 * alpha * grad_kp1.transpose() * grad_kp1); // Armijo condition

            if (grad_k.norm() < min_grad_)
            {
                opt_f = cost_k;
                return FIND_MIN;
            }
        }
        else
        {
            do
            {
                x_kp1 = x_k - alpha * grad_k;
                cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                alpha *= 0.5;
            } while (cost_kp1 > cost_k - 1e-4 * alpha * grad_k.transpose() * grad_k); // Armijo condition

            if (grad_kp1.norm() < min_grad_)
            {
                opt_f = cost_kp1;
                return FIND_MIN;
            }
        }
    }

    opt_f = iter_limit_ % 2 ? cost_k : cost_kp1;
    return REACH_MAX_ITERATION;
}
