#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include"subscribe2location.hpp"
#include "rclcpp/rclcpp.hpp"  // <--- 必须添加这一行
#include <Eigen/Dense>
#include <thread>
#include <memory>
namespace io
{
class ROS2
{
public:
  ROS2();

  ~ROS2();


  LocationInfo subscribe();

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);

    auto publisher = node->create_publisher<T>(topic_name, queue_size);

    // 运行一个单独的线程来 spin 这个节点，确保消息可以被正确发布
    std::thread([node]() { rclcpp::spin(node); }).detach();

    return publisher;
  }

private:
  std::shared_ptr<Subscribe2Location> subscribe2location_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;


};

}  // namespace io
#endif