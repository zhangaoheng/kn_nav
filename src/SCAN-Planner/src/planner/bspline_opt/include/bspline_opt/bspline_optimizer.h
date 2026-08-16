// ============================================================
// 文件：bspline_optimizer.h
// 模块：bspline_opt（B 样条轨迹优化）
// 职责：声明 B 样条轨迹优化器 BsplineOptimizer，以及碰撞回弹
//       （Rebound）优化所需的控制点附带数据 ControlPoints。
// 数据流：规划器把粗轨迹控制点、引导路径与 PCT 走廊路径传入，
//       优化器在离散控制点序列上构造代价函数并用 L-BFGS 求解，
//       返回优化后的控制点矩阵。
// 关键点：代价函数含平滑（jerk）、碰撞距离、动态可行性、
//       曲线拟合与 PCT 走廊偏离五项，权重由参数加载。
// ============================================================
#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>
#include "bspline_opt/lbfgs.hpp"

// Gradient and elastic band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace scan_planner
{

  // ControlPoints：碰撞回弹（Rebound）优化使用的控制点附带数据。
  // base_point/direction 一一对应：base_point 为碰撞点（方向向量
  // 的起点），direction 为归一化的推开方向，距离代价据此把控制点
  // 推离障碍物；flag_temp 为临时标记（如是否已生成回弹方向），
  // 使用前需初始化。
  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the start of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it every time before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

  // BsplineOptimizer：B 样条轨迹优化器（核心类）。
  // 对外主接口为 BsplineOptimizeTraj()，内部按两个阶段工作：
  //   1) Rebound（回弹）阶段：先碰撞分段 + A* 绕障，再最小化
  //      平滑/碰撞/可行性/走廊代价；
  //   2) Refine（精修）阶段：在保证无碰撞的前提下向引导路径
  //      拟合（fitness 代价）。
  // 控制点按 3 行 N 列矩阵存储，每列是一个三维控制点。
  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    // 设置占据栅格地图（碰撞查询与射线检测的环境）。
    void setEnvironment(const GridMap::Ptr &env);
    // 从 ROS 2 参数服务器加载优化权重与速度/加速度上限等参数。
    void setParam(rclcpp::Node *node);
    // 主入口：给定初始控制点与时间间隔，按 cost_function 选择
    // Rebound/Refine 流程执行优化，返回优化后的控制点矩阵。
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setCorridorPath(const vector<Eigen::Vector3d> &corridor_path,
                         double max_deviation);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void optimize();

    Eigen::MatrixXd getControlPoints();

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

  private:
    GridMap::Ptr grid_map_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> corridor_path_;
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting
    double lambda_corridor_;       // PCT path corridor weight
    double corridor_max_deviation_;
    int a;
    //
    double dist0_;             // safe distance
    double max_vel_, max_acc_; // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    ControlPoints cps_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcCorridorCost(const Eigen::MatrixXd &q, double &cost,
                          Eigen::MatrixXd &gradient);
    Eigen::Vector3d nearestCorridorPoint(const Eigen::Vector3d &point) const;
    bool check_collision_and_rebound(void);
    double estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    double estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const;

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benchmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner
#endif
