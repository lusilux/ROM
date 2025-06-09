#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <numeric>

class ContextLabelPublisher : public rclcpp::Node
{
public:
  ContextLabelPublisher()
  : Node("context_label_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/context_label", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ContextLabelPublisher::laser_callback, this, std::placeholders::_1));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Filtro simple para ignorar NaNs o infs
    std::vector<float> valid_ranges;
    for (float r : msg->ranges) {
      if (std::isfinite(r)) {
        valid_ranges.push_back(r);
      }
    }

    if (valid_ranges.empty()) return;

    // Estimación de área: promedio de distancias * ángulo total / 2 (muy simplificado)
    float avg_range = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0f) / valid_ranges.size();
    float angle_span = msg->angle_max - msg->angle_min;
    float estimated_area = 0.5f * avg_range * avg_range * angle_span; // área de un sector circular

    std::string label;
    if (estimated_area > 5.0)
      label = "zona_abierta";
    else if (estimated_area > 3.5)
      label = "zona_media";
    else
      label = "zona_estrecha";

    auto message = std_msgs::msg::String();
    message.data = label;

    RCLCPP_INFO(this->get_logger(), "Área estimada: %.2f -> Etiqueta: %s", estimated_area, label.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContextLabelPublisher>());
  rclcpp::shutdown();
  return 0;
}

