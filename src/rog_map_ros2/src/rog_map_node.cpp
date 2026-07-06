#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rog_map/rog_map.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rog_map_node");
  auto rog_map = std::make_shared<rog_map::ROGMap>(node);

  (void)rog_map;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
