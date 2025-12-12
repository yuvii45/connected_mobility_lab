#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cpm_lab_lab_msgs/msg/vehicle_observation.hpp>

using namespace std::chrono_literals;

class DummyVehicleObservationPublisher : public rclcpp::Node
{
public:
    DummyVehicleObservationPublisher()
    : Node("dummy_vehicle_observation_publisher")
    {
        publisher_ = this->create_publisher<cpm_lab_lab_msgs::msg::VehicleObservation>(
            "/lab/vehicle_1/vehicleObservation", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&DummyVehicleObservationPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Dummy VehicleObservation publisher started.");
    }

private:
    void timer_callback()
    {
        cpm_lab_lab_msgs::msg::VehicleObservation msg;

        // Fill with dummy values
        msg.vehicle_id = 1;

        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        // Valid after timestamp (slightly behind current time)
        msg.valid_after_stamp = this->now() - rclcpp::Duration(0, 50000000);  // 50ms

        // Dummy pose
        msg.pose.position.x = 1.23;
        msg.pose.position.y = 4.56;
        msg.pose.position.z = 0.0;

        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<cpm_lab_lab_msgs::msg::VehicleObservation>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyVehicleObservationPublisher>());
    rclcpp::shutdown();
    return 0;
}
