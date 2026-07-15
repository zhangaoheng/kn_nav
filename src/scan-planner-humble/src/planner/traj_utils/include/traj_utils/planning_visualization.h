#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/polynomial_traj.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace scan_planner
{
  class PlanningVisualization
  {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string frame_id_{"world"};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr guide_vector_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_state_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(const rclcpp::Node::SharedPtr& node);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                           const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayOptimalTraj(UniformBspline position_traj, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                          const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
  };
} // namespace scan_planner
#endif
