// ============================================================================
// 文件名：planning_visualization.h
// 用途：SCAN 局部规划的可视化工具类头文件。把规划过程的关键几何信息
//       （目标点、全局路径、初始路径、最优轨迹、A* 搜索路径、箭头等）以
//       RViz Marker/MarkerArray 形式发布，便于调试与演示。
// 结构：
//   - PlanningVisualization：唯一可视化类，持有各话题的 Marker 发布器
//   - display* 系列：按内容类型组织的一次性可视化接口
// 依赖：uniform_bspline（轨迹求值）、polynomial_traj、visualization_msgs
// ============================================================================
#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <traj_utils/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace scan_planner
{
  // 可视化工具类：按话题分类创建 Marker 发布器（目标点/全局列表/初始列表/
  // 最优列表/A* 列表），并提供对应的显示接口；所有接口只读轨迹数据，不改状态。
  class PlanningVisualization
  {
  private:
    using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>;
    using MarkerArrayPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;
    rclcpp::Node *node_{nullptr};
    std::string frame_id_{"map"};

    MarkerPublisher::SharedPtr goal_point_pub;
    MarkerPublisher::SharedPtr global_list_pub;
    MarkerPublisher::SharedPtr init_list_pub;
    MarkerPublisher::SharedPtr optimal_list_pub;
    MarkerPublisher::SharedPtr a_star_list_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    explicit PlanningVisualization(rclcpp::Node *node);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    // 把一组三维点以离散 Marker（球/点）形式发布到指定发布器。
    void displayMarkerList(const MarkerPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);
    // 生成折线路径 MarkerArray（显示为连线），供全局/初始路径显示复用。
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // 生成箭头 MarkerArray（每点一个带方向的箭头），用于显示速度/方向信息。
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // 显示目标点（球体 Marker）。
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    // 显示全局参考路径（折线）。
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    // 显示局部重规划的初始路径点集。
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    // 显示优化后的控制点序列。
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    // 显示最终最优 B 样条轨迹（沿轨迹采样成折线）。
    void displayOptimalTraj(UniformBspline position_traj, int id);
    // 显示 A* 搜索得到的多条路径（用于展示搜索过程）。
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    // 把一组点以箭头形式发布到指定发布器。
    void displayArrowList(const MarkerArrayPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list,
                          double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, scan_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, scan_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace scan_planner
#endif
