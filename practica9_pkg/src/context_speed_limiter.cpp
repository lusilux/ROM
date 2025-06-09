#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

#include <string>

class ContextSpeedLimiter : public rclcpp::Node
{
public:
  ContextSpeedLimiter()
  : Node("context_speed_limiter"), last_speed_(-1.0)
  {
    RCLCPP_INFO(this->get_logger(), "Context Speed Limiter node started.");

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/context_label", 10,
      std::bind(&ContextSpeedLimiter::label_callback, this, std::placeholders::_1));

    client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/controller_server/set_parameters");
  }

private:
  void label_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string label = msg->data;
    
    RCLCPP_INFO(this->get_logger(), "Recibida etiqueta: %s", msg->data.c_str());

    // Define velocidades por contexto
    double speed;
    if (label == "zona_abierta")
      speed = 0.26;
    else if (label == "zona_media")
      speed = 0.18;
    else if (label == "zona_estrecha")
      speed = 0.08;
    else {
      RCLCPP_WARN(this->get_logger(), "Etiqueta desconocida: '%s'", label.c_str());
      return;
    }

    // Evitar reenvío si la velocidad no cambia
    if (speed == last_speed_)
      return;

    last_speed_ = speed;

    // Esperar al servicio
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Servicio '/controller_server/set_parameters' no disponible.");
      return;
    }

    // Crear parámetro
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter param;
    param.name = "FollowPath.max_vel_x";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = speed;
    request->parameters.push_back(param);

    auto future = client_->async_send_request(request,
	[this, speed, label](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture result_future) {
	    try {
	      (void)result_future.get();  // solo comprobar que no falló
	      RCLCPP_INFO(this->get_logger(), "Velocidad ajustada a %.2f m/s por contexto '%s'", speed, label.c_str());
	    } catch (const std::exception &e) {
	      RCLCPP_ERROR(this->get_logger(), "Error al cambiar parámetro: %s", e.what());
	    }
	});
  }

  double last_speed_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContextSpeedLimiter>());
  rclcpp::shutdown();
  return 0;
}

